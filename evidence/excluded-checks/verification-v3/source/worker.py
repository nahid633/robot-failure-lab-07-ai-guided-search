"""Bounded closed-loop test; intentional horizon cancellation is not goal failure."""
import argparse,atexit,copy,json,math,threading,time
from pathlib import Path
import core


def main():
    p=argparse.ArgumentParser();p.add_argument('--scenario',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
    s=json.loads(a.scenario.read_text());core.configure(s)
    from observer import Observer
    from support import parameter_snapshot
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from nav2_simple_commander.robot_navigator import BasicNavigator
    from geometry_msgs.msg import PoseStamped
    from ros_gz_interfaces.srv import SetEntityPose
    from search import HORIZON
    rclpy.init();o=Observer('restamped',a.output);atexit.register(o.save)
    executor=SingleThreadedExecutor();executor.add_node(o)
    def spin():
        try:executor.spin()
        except Exception as error:o.error=repr(error);(a.output/'OBSERVER_ERROR.txt').write_text(o.error)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    nav=BasicNavigator();nav.set_parameters([Parameter('use_sim_time',value=True)])
    def pose(x,y):
        v=PoseStamped();v.header.frame_id='map';v.header.stamp=nav.get_clock().now().to_msg();v.pose.position.x=x;v.pose.position.y=y;v.pose.orientation.w=1.;return v
    nav.setInitialPose(pose(*core.START));nav.waitUntilNav2Active();began=time.monotonic()
    while not all(o.data[k] for k in ('world','obstacle','localized','scans')) or o.sensor_offset is None:
        rclpy.spin_once(nav,timeout_sec=.05)
        if time.monotonic()-began>12:raise RuntimeError('Missing instrument stream')
    setter=nav.create_client(SetEntityPose,'/world/default/set_pose');assert setter.wait_for_service(timeout_sec=5)
    effective=parameter_snapshot(nav);(a.output/'effective-before.json').write_text(json.dumps(effective,indent=2))
    assert effective['/controller_server']['general_goal_checker.xy_goal_tolerance']==.25
    assert effective['/collision_monitor']['source_timeout']==1. and effective['/collision_monitor']['scan.enabled']
    settle=o.now()
    while o.now()-settle<1.5:
        rclpy.spin_once(nav,timeout_sec=.05)
        if time.monotonic()-began>35:raise RuntimeError('Clock stalled before goal')
    graph=o.graph();assert core.routing_valid(graph),graph
    initial=o.data['world'][-1];assert math.hypot(initial['x']-core.START[0],initial['y']-core.START[1])<.05
    assert math.hypot(o.data['obstacle'][-1]['x']+8,o.data['obstacle'][-1]['y']+8)<.005
    (a.output/'READY.json').write_text(json.dumps(dict(t=o.now(),graph=graph,initial=initial,sensor_offset=o.sensor_offset)))
    start=o.now();wall=time.monotonic();(a.output/'GOAL_SUBMITTED.json').write_text(json.dumps(dict(t=start,goal=core.GOAL)))
    accepted=bool(nav.goToPose(pose(*core.GOAL)));assert accepted
    moving=None;future=None;request_time=None;natural_terminal=None;feedback=[]
    while True:
        now=o.now()
        if o.error:raise RuntimeError(o.error)
        complete=nav.isTaskComplete()
        if complete and natural_terminal is None:natural_terminal=dict(t=now,status=int(nav.result_future.result().status))
        if natural_terminal is not None and o.window is None:raise RuntimeError('Task ended before scenario trigger; retain outcome, do not retry')
        with o.lock:
            # isTaskComplete may spin ROS callbacks. Timestamp the trigger only
            # after those callbacks, while the scan-routing lock is held.
            now=o.now()
            latest=o.data['odom'][-1];wp=o.data['world'][-1]
            if now-latest['stamp']<.15 and latest['speed']>=.05:moving=now if moving is None else moving
            else:moving=None
            if o.window is None and now-start>=3 and wp['x']>=core.TRIGGER_X and .25<=wp['y']<=.85 and moving is not None and now-moving>=.3:
                o.window=now;o.cached=copy.deepcopy(o.last_scan)
                (a.output/'FAULT_START.json').write_text(json.dumps(dict(t=now,world_pose=wp,cached_origin=o.cached[1],cached_stamp=o.stamp(o.cached[0]),cached_hash=o.cached[2],scenario=s)))
                print('Scenario starts at goal +{:.3f}s'.format(now-start),flush=True)
            if o.window is not None and now>=o.window+core.APPEAR_DELAY and future is None:
                request=SetEntityPose.Request();request.entity.name='rfl05_obstacle';request.entity.type=2
                request.pose.position.x=core.OBSTACLE[0];request.pose.position.y=core.OBSTACLE[1];request.pose.position.z=.35;request.pose.orientation.w=1.
                request_time=now;future=setter.call_async(request)
            if future is not None and future.done() and not future.result().success:raise RuntimeError('Obstacle placement refused')
            if request_time is not None and now-request_time>1 and o.appearance is None:raise RuntimeError('Obstacle appearance not confirmed')
        f=nav.getFeedback()
        if f:feedback.append(dict(t=now,x=f.current_pose.pose.position.x,y=f.current_pose.pose.position.y,recoveries=int(f.number_of_recoveries)))
        if o.window is not None and now>=o.window+HORIZON:break
        if (o.window is None and now-start>45) or time.monotonic()-wall>150:raise RuntimeError('Scenario/clock deadline; retained attempt')
    cutoff=o.window+HORIZON;observed_cutoff=o.now();assert o.appearance is not None
    (a.output/'OBSERVATION_CUTOFF.json').write_text(json.dumps(dict(t=cutoff,observed_t=observed_cutoff,natural_terminal=natural_terminal)))
    cancel_requested=natural_terminal is None
    if cancel_requested:nav.cancelTask()
    tail=time.monotonic()
    while o.now()<observed_cutoff+1.8:
        rclpy.spin_once(nav,timeout_sec=.05)
        if o.error:raise RuntimeError(o.error)
        if time.monotonic()-tail>30:raise RuntimeError('Tail stalled')
    response=nav.result_future.result() if nav.result_future.done() else None
    after=parameter_snapshot(nav);(a.output/'effective-after.json').write_text(json.dumps(after,indent=2));assert effective==after
    executor.shutdown();thread.join(timeout=3)
    if o.error:raise RuntimeError(o.error)
    o.save();end=o.now()
    contacts=[c for c in o.contacts if c['robot_obstacle_pairs'] and o.appearance<=c['stamp']<=cutoff]
    truth=[r for r in o.data['world'] if o.appearance<=r['stamp']<=cutoff]
    record=dict(scenario=s,start_sim_s=start,window_start=o.window,appearance=o.appearance,
        cutoff=cutoff,observed_cutoff=observed_cutoff,observation_end=end,obstacle_request_time=request_time,
        obstacle_service_success=bool(future.done() and future.result().success),goal_accepted=accepted,
        natural_terminal=natural_terminal,cancel_requested=cancel_requested,terminal_action_status=getattr(response,'status',None),
        label=int(bool(contacts)),contact_messages_in_label_window=len(contacts),
        first_contact_after_appearance_s=contacts[0]['stamp']-o.appearance if contacts else None,
        min_sampled_envelope_clearance_m=min(core.clearance(r) for r in truth),
        stop_after_appearance_s=(lambda t:None if t is None else t-o.appearance)(core.sustained_stop(o.data['odom'],o.appearance,cutoff)),
        graph=graph,graph_end=o.graph(),sensor_offset=o.sensor_offset,worker_wall_s=time.monotonic()-wall)
    (a.output/'feedback.json').write_text(json.dumps(feedback));(a.output/'result.json').write_text(json.dumps(record,indent=2))
    print(json.dumps(record),flush=True);atexit.unregister(o.save);nav.destroy_node();o.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
