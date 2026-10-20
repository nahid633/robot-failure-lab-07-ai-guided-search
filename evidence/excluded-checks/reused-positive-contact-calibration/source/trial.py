#!/usr/bin/env python3
"""One original navigation goal with captured fault provenance and world geometry."""
import argparse,atexit,base64,copy,csv,hashlib,json,math,struct,threading,time
from collections import deque
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import serialize_message
from rclpy.time import Time
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped,Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer,TransformListener
from nav2_msgs.msg import CollisionMonitorState
from ros_gz_interfaces.msg import Contacts
from ros_gz_interfaces.srv import SetEntityPose
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from core import *
from support import parameter_snapshot,endpoint_metrics


def yaw(q):return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
def payload_hash(msg):
    # Canonical field representation excludes stamp and transport padding.
    frame=msg.header.frame_id.encode('utf8')
    fields=(msg.angle_min,msg.angle_max,msg.angle_increment,msg.time_increment,msg.scan_time,msg.range_min,msg.range_max)
    data=struct.pack('<I',len(frame))+frame+struct.pack('<7fII',*fields,len(msg.ranges),len(msg.intensities))
    data+=struct.pack('<{}f'.format(len(msg.ranges)),*msg.ranges)+struct.pack('<{}f'.format(len(msg.intensities)),*msg.intensities)
    return hashlib.sha256(data).hexdigest()


class Observer(Node):
    def __init__(self,condition,output):
        super().__init__('failure_lab_05_observer');self.set_parameters([Parameter('use_sim_time',value=True)])
        self.condition=condition;self.output=output;self.lock=threading.RLock();self.error=None
        self.window=None;self.cached=None;self.last_scan=None;self.appearance=None;self.sensor_offset=None
        self.data={n:[] for n in ('world','obstacle','localized','odom','scans','published','receipts','delivery_gaps','commands','input_commands','collision','probes')}
        self.raw=[];self.output_raw=[];self.contacts=[];self.pending={b:deque() for b in TOPICS}
        self.pubs={b:self.create_publisher(LaserScan,t,qos_profile_sensor_data) for b,t in TOPICS.items()}
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        for b,t in TOPICS.items():self.create_subscription(LaserScan,t,lambda m,b=b:self.receipt(m,b),qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/rfl05/truth',lambda m:self.world(m,'world','turtlebot3_waffle'),qos_profile_sensor_data)
        self.create_subscription(TFMessage,'/rfl05/obstacle_pose',lambda m:self.world(m,'obstacle','rfl05_obstacle'),qos_profile_sensor_data)
        self.create_subscription(Odometry,'/odom',self.motion,qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.command(m,'commands'),qos_profile_sensor_data)
        self.create_subscription(Twist,'/cmd_vel_smoothed',lambda m:self.command(m,'input_commands'),qos_profile_sensor_data)
        self.create_subscription(CollisionMonitorState,'/collision_monitor_state',self.state,10)
        self.create_subscription(Contacts,'/rfl05/contacts',self.contact,qos_profile_sensor_data)
        self.buffer=Buffer(cache_time=Duration(seconds=20));self.listener=TransformListener(self.buffer,self,spin_thread=False)
        self.create_timer(.05,self.probe)

    def now(self):return self.get_clock().now().nanoseconds/1e9
    @staticmethod
    def stamp(m):return m.header.stamp.sec+m.header.stamp.nanosec/1e9

    def world(self,msg,key,name):
        with self.lock:
            for tr in msg.transforms:
                if tr.child_frame_id.split('::')[-1]!=name:continue
                p=tr.transform.translation
                r=dict(t=self.now(),stamp=self.stamp(tr),x=p.x,y=p.y,yaw=yaw(tr.transform.rotation),frame=tr.header.frame_id,child=tr.child_frame_id)
                self.data[key].append(r)
                if key=='obstacle' and math.hypot(p.x-OBSTACLE[0],p.y-OBSTACLE[1])<.005 and self.appearance is None:self.appearance=r['stamp']

    def probe(self):
        with self.lock:
            now=self.now()
            try:
                tr=self.buffer.lookup_transform('map','base_link',Time());p=tr.transform.translation
                self.data['localized'].append(dict(t=now,stamp=self.stamp(tr),x=p.x,y=p.y,yaw=yaw(tr.transform.rotation)))
                if self.sensor_offset is None:
                    sensor=self.buffer.lookup_transform('base_link','base_scan',Time());v=sensor.transform.translation
                    self.sensor_offset=(v.x,v.y,yaw(sensor.transform.rotation))
            except Exception:pass
            ok=self.buffer.can_transform('map','odom',Time(seconds=now,clock_type=self.get_clock().clock_type),timeout=Duration())
            self.data['probes'].append(dict(t=now,map_odom_ok=int(ok)))

    def scan(self,msg):
        with self.lock:
            now=self.now();stamp=self.stamp(msg);seq=len(self.data['scans']);digest=payload_hash(msg)
            hits=0;wp=self.data['world'][-1] if self.data['world'] else None
            if self.appearance is not None and wp is not None and self.sensor_offset is not None and abs(stamp-wp['stamp'])<.15:
                dx,dy,dyaw=self.sensor_offset;c=math.cos(wp['yaw']);s=math.sin(wp['yaw'])
                sx=wp['x']+c*dx-s*dy;sy=wp['y']+s*dx+c*dy
                for i,r in enumerate(msg.ranges):
                    if not math.isfinite(r) or not msg.range_min<=r<=msg.range_max:continue
                    angle=wp['yaw']+dyaw+msg.angle_min+i*msg.angle_increment
                    x=sx+r*math.cos(angle);y=sy+r*math.sin(angle)
                    if abs(x-OBSTACLE[0])<=OBSTACLE_SIZE[0]/2+.04 and abs(y-OBSTACLE[1])<=OBSTACLE_SIZE[1]/2+.04:hits+=1
            self.data['scans'].append(dict(t=now,stamp=stamp,seq=seq,payload_hash=digest,obstacle_hits=hits))
            self.raw.append(dict(seq=seq,cdr=base64.b64encode(serialize_message(msg)).decode()))
            self.publish('localization',msg,seq,stamp,digest)
            kind=policy(self.condition,now,self.window)
            if kind!='missing':
                source=msg;origin=seq;origin_stamp=stamp
                if kind in ('frozen','restamped'):
                    source=copy.deepcopy(self.cached[0]);origin=self.cached[1];origin_stamp=self.stamp(source)
                    if kind=='restamped':source.header.stamp=copy.deepcopy(msg.header.stamp)
                self.publish('obstacles',source,origin,origin_stamp,digest if kind=='fresh' else self.cached[2])
            self.last_scan=(copy.deepcopy(msg),seq,digest)

    def publish(self,branch,msg,origin,origin_stamp,digest):
        seq=len(self.data['published']);row=dict(t=self.now(),branch=branch,seq=seq,origin=origin,
                origin_stamp=origin_stamp,stamp=self.stamp(msg),payload_hash=digest)
        self.data['published'].append(row);self.pending[branch].append(row)
        self.output_raw.append(dict(seq=seq,cdr=base64.b64encode(serialize_message(msg)).decode()))
        self.pubs[branch].publish(msg)

    def receipt(self,msg,branch):
        with self.lock:
            if not self.pending[branch]:raise RuntimeError('Unaccounted branch delivery')
            digest=payload_hash(msg);expected=None
            while self.pending[branch]:
                candidate=self.pending[branch].popleft()
                if self.stamp(msg)==candidate['stamp'] and digest==candidate['payload_hash']:
                    expected=candidate;break
                self.data['delivery_gaps'].append(dict(t=self.now(),branch=branch,seq=candidate['seq']))
            if expected is None:raise RuntimeError('Unaccounted or changed branch packet')
            self.data['receipts'].append(dict(t=self.now(),branch=branch,seq=expected['seq'],stamp=self.stamp(msg),payload_hash=digest))

    def motion(self,msg):
        with self.lock:
            v=msg.twist.twist;self.data['odom'].append(dict(t=self.now(),stamp=self.stamp(msg),speed=math.hypot(v.linear.x,v.linear.y),w=v.angular.z))
    def command(self,msg,key):
        with self.lock:self.data[key].append(dict(t=self.now(),speed=math.hypot(msg.linear.x,msg.linear.y),w=msg.angular.z))
    def state(self,msg):
        with self.lock:self.data['collision'].append(dict(t=self.now(),action_type=int(msg.action_type),polygon=msg.polygon_name))
    def contact(self,msg):
        with self.lock:
            pairs=[dict(a=c.collision1.name,b=c.collision2.name,depths=list(c.depths)) for c in msg.contacts]
            robot=[c for c in pairs if ('turtlebot3_waffle' in c['a'] or 'turtlebot3_waffle' in c['b']) and ('rfl05_obstacle' in c['a'] or 'rfl05_obstacle' in c['b'])]
            self.contacts.append(dict(t=self.now(),stamp=self.stamp(msg),pairs=pairs,robot_obstacle_pairs=robot))
    def graph(self):
        return {t:dict(publishers=[s.node_name for s in self.get_publishers_info_by_topic(t)],subscribers=[s.node_name for s in self.get_subscriptions_info_by_topic(t)]) for t in ('/scan',*TOPICS.values(),'/rfl05/truth','/rfl05/obstacle_pose','/rfl05/contacts')}
    def save(self):
        with self.lock:
            for name,data in self.data.items():
                if data:
                    with (self.output/(name+'.csv')).open('w',newline='') as f:
                        writer=csv.DictWriter(f,fieldnames=list(data[0]));writer.writeheader();writer.writerows(data)
            for name,data in [('raw-scans',self.raw),('published-scans',self.output_raw),('contacts',self.contacts)]:
                with (self.output/(name+'.jsonl')).open('w') as f:
                    for row in data:f.write(json.dumps(row)+'\n')


def main():
    p=argparse.ArgumentParser();p.add_argument('--condition',choices=CONDITIONS,required=True);p.add_argument('--output',type=Path,required=True);p.add_argument('--calibration',action='store_true');a=p.parse_args()
    rclpy.init();o=Observer(a.condition,a.output);atexit.register(o.save)
    executor=SingleThreadedExecutor();executor.add_node(o)
    def spin():
        try:executor.spin()
        except Exception as error:
            o.error=repr(error);(a.output/'OBSERVER_ERROR.txt').write_text(o.error)
    thread=threading.Thread(target=spin,daemon=True);thread.start()
    nav=BasicNavigator();nav.set_parameters([Parameter('use_sim_time',value=True)])
    def pose(x,y):
        v=PoseStamped();v.header.frame_id='map';v.header.stamp=nav.get_clock().now().to_msg();v.pose.position.x=x;v.pose.position.y=y;v.pose.orientation.w=1.;return v
    nav.setInitialPose(pose(*START));nav.waitUntilNav2Active();began=time.monotonic()
    while not all(o.data[k] for k in ('world','obstacle','localized','scans')) or o.sensor_offset is None:
        rclpy.spin_once(nav,timeout_sec=.05)
        if time.monotonic()-began>12:raise RuntimeError('Missing instrument stream')
    setter=nav.create_client(SetEntityPose,'/world/default/set_pose')
    assert setter.wait_for_service(timeout_sec=5)
    effective=parameter_snapshot(nav);(a.output/'effective-before.json').write_text(json.dumps(effective,indent=2))
    assert effective['/controller_server']['general_goal_checker.xy_goal_tolerance']==.25
    assert effective['/collision_monitor']['source_timeout']==1. and effective['/collision_monitor']['scan.enabled']
    settle=o.now()
    while o.now()-settle<1.5:
        rclpy.spin_once(nav,timeout_sec=.05)
        if time.monotonic()-began>35:raise RuntimeError('Clock stalled before goal')
    graph=o.graph();assert routing_valid(graph),graph
    initial=o.data['world'][-1];assert math.hypot(initial['x']-START[0],initial['y']-START[1])<.05
    assert math.hypot(o.data['obstacle'][-1]['x']+8,o.data['obstacle'][-1]['y']+8)<.005
    (a.output/'READY.json').write_text(json.dumps(dict(t=o.now(),graph=graph,initial=initial,sensor_offset=o.sensor_offset),indent=2))
    def place(x,y):
        request=SetEntityPose.Request();request.entity.name='rfl05_obstacle';request.entity.type=2
        request.pose.position.x=x;request.pose.position.y=y;request.pose.position.z=.35;request.pose.orientation.w=1.
        return setter.call_async(request)
    if a.calibration:
        future=place(initial['x'],initial['y']);start=o.now();wall=time.monotonic()
        while o.now()-start<2.:
            rclpy.spin_once(nav,timeout_sec=.05)
            if time.monotonic()-wall>15:raise RuntimeError('Contact calibration stalled')
        assert future.done() and future.result().success
        assert any(c['robot_obstacle_pairs'] for c in o.contacts),'No obstacle-specific positive contact'
        (a.output/'CALIBRATION.json').write_text(json.dumps(dict(passed=True,contact_messages=len(o.contacts),positive_messages=sum(bool(c['robot_obstacle_pairs']) for c in o.contacts)),indent=2))
        print('CALIBRATION PASSED',flush=True)
    else:
        start=o.now();wall=time.monotonic();(a.output/'GOAL_SUBMITTED.json').write_text(json.dumps(dict(t=start,goal=GOAL)))
        accepted=bool(nav.goToPose(pose(*GOAL)));feedback=[];moving=None;future=None;request_time=None;timed_out=False
        while accepted and not nav.isTaskComplete():
            now=o.now()
            if o.error:raise RuntimeError(o.error)
            with o.lock:
                latest=o.data['odom'][-1];wp=o.data['world'][-1]
                if now-latest['stamp']<.15 and latest['speed']>=.05:moving=now if moving is None else moving
                else:moving=None
                if o.window is None and now-start>=3 and wp['x']>=TRIGGER_X and .25<=wp['y']<=.85 and moving is not None and now-moving>=.3:
                    o.window=now;o.cached=copy.deepcopy(o.last_scan)
                    (a.output/'FAULT_START.json').write_text(json.dumps(dict(t=now,world_pose=wp,cached_origin=o.cached[1],cached_stamp=o.stamp(o.cached[0]),cached_hash=o.cached[2]),indent=2))
                    print('Fault begins at goal +{:.3f}s'.format(now-start),flush=True)
                if o.window is not None and now>=o.window+APPEAR_DELAY and future is None:
                    request_time=now;future=place(*OBSTACLE)
                if future is not None and future.done() and not future.result().success:raise RuntimeError('Obstacle service refused placement')
                if request_time is not None and now-request_time>1 and o.appearance is None:raise RuntimeError('Obstacle world pose did not confirm appearance')
            f=nav.getFeedback()
            if f:feedback.append(dict(t=now,x=f.current_pose.pose.position.x,y=f.current_pose.pose.position.y,recoveries=int(f.number_of_recoveries)))
            if now-start>=TASK_LIMIT or time.monotonic()-wall>=TASK_WALL_LIMIT:timed_out=True;nav.cancelTask();break
        end=o.now();result=nav.getResult() if accepted else TaskResult.FAILED
        response=nav.result_future.result() if accepted and nav.result_future.done() else None;ar=response.result if response else None
        outcome='timeout' if timed_out else ('succeeded' if result==TaskResult.SUCCEEDED else 'failed')
        (a.output/'TASK_TERMINAL.json').write_text(json.dumps(dict(t=end,outcome=outcome)))
        if o.window is None or o.appearance is None:raise RuntimeError('Missing fault/obstacle trigger; retained task outcome')
        tail=time.monotonic();until=max(end+1.8,o.window+6)
        while o.now()<until:
            rclpy.spin_once(nav,timeout_sec=.05)
            if o.error:raise RuntimeError(o.error)
            if time.monotonic()-tail>30:raise RuntimeError('Tail stalled')
        after=parameter_snapshot(nav);(a.output/'effective-after.json').write_text(json.dumps(after,indent=2));assert effective==after
        executor.shutdown();thread.join(timeout=3)
        if o.error:raise RuntimeError(o.error)
        o.save();observation_end=o.now()
        metrics=summarize(o.data['world'],o.data['odom'],o.window,o.appearance,end,observation_end)
        endpoint=endpoint_metrics(o.data['world'],o.data['localized'],start,end,outcome)
        record=dict(condition=a.condition,start_sim_s=start,end_sim_s=end,elapsed_sim_s=end-start,elapsed_wall_s=time.monotonic()-wall,
            window_start=o.window,appearance=o.appearance,obstacle_request_time=request_time,obstacle_service_success=bool(future and future.done() and future.result().success),
            observation_end=observation_end,outcome=outcome,goal_accepted=accepted,terminal_action_status=getattr(response,'status',None),
            action_error_code=getattr(ar,'error_code',None),recoveries=max((r['recoveries'] for r in feedback),default=None),
            metrics=metrics,endpoint=endpoint,graph=graph,graph_end=o.graph(),sensor_offset=o.sensor_offset,
            contact_messages=len(o.contacts),robot_contact_messages=sum(bool(c['robot_obstacle_pairs']) for c in o.contacts))
        (a.output/'feedback.json').write_text(json.dumps(feedback));(a.output/'result.json').write_text(json.dumps(record,indent=2)+'\n');print(json.dumps(record),flush=True)
    if a.calibration:executor.shutdown();thread.join(timeout=3)
    o.save();atexit.unregister(o.save);nav.destroy_node();o.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
