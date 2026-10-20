"""Unchanged Lab04 parameter/endpoint/process helpers, retained locally for standalone use."""
import math,os,signal,subprocess,time
import rclpy
from rclpy.parameter import parameter_value_to_python
from rcl_interfaces.srv import GetParameters,ListParameters
from core import GOAL,POSITION_LIMIT

def parameter_snapshot(nav):
    def call(node,service,kind,request):
        client=nav.create_client(kind,f'{node}/{service}')
        if not client.wait_for_service(timeout_sec=5): raise RuntimeError(f'Missing {node}/{service}')
        future=client.call_async(request); began=time.monotonic()
        while not future.done():
            rclpy.spin_once(nav,timeout_sec=.05)
            if time.monotonic()-began>8: raise RuntimeError(f'Parameter timeout: {node}')
        result=future.result();nav.destroy_client(client);return result
    result={}
    for node in ('/controller_server','/amcl','/planner_server','/velocity_smoother','/collision_monitor',
                 '/local_costmap/local_costmap','/global_costmap/global_costmap'):
        names=call(node,'list_parameters',ListParameters,ListParameters.Request()).result.names
        response=call(node,'get_parameters',GetParameters,GetParameters.Request(names=names))
        if len(response.values)==len(names):
            result[node]={k:parameter_value_to_python(v) for k,v in zip(names,response.values)}
        else:
            # rclcpp can return an empty bulk response if one declared parameter
            # has no initialized value. Never silently record the whole node as {}.
            result[node]={}
            for name in names:
                single=call(node,'get_parameters',GetParameters,GetParameters.Request(names=[name]))
                if len(single.values)==1:
                    result[node][name]=parameter_value_to_python(single.values[0])
                elif not single.values:
                    result[node][name]={'unavailable':'Declared parameter returned no value; see node log for uninitialized declarations.'}
                else:raise RuntimeError(f'Unexpected parameter response: {node}/{name}')
    return result


def distance(row):
    return math.hypot(row['x'] - GOAL[0], row['y'] - GOAL[1])


def endpoint_metrics(truth, localized, start, end, outcome):
    # Truth stamp is simulation acquisition time, not ROS receipt time.
    def terminal(rows):
        eligible = [r for r in rows if start <= r['stamp'] <= end]
        if not eligible or end - eligible[-1]['stamp'] > 0.15:
            raise ValueError('Missing/far-stale terminal pose')
        return eligible[-1]
    tr = terminal(truth)
    loc = terminal(localized)
    settled = [r for r in truth if end + 1.0 <= r['stamp'] <= end + 1.5]
    if len(settled) < 8 or settled[0]['stamp'] > end + 1.1 or settled[-1]['stamp'] < end + 1.4:
        raise ValueError('Incomplete settled-position window')
    if max(b['stamp'] - a['stamp'] for a, b in zip(settled, settled[1:])) > 0.15:
        raise ValueError('Sparse settled-position window')
    route = [r for r in truth if start <= r['stamp'] <= end]
    peak = max(distance(r) for r in settled)
    return dict(terminal_world_error_m=distance(tr), terminal_localized_error_m=distance(loc),
                terminal_world_stamp=tr['stamp'], terminal_localized_stamp=loc['stamp'],
                terminal_world_x=tr['x'], terminal_world_y=tr['y'],
                terminal_world_yaw_error_rad=abs(math.atan2(math.sin(tr['yaw']), math.cos(tr['yaw']))),
                settled_world_error_max_m=peak,
                settled_world_error_mean_m=sum(distance(r) for r in settled) / len(settled),
                settled_position_pass=bool(outcome == 'succeeded' and peak <= POSITION_LIMIT),
                world_path_length_m=sum(math.hypot(b['x']-a['x'], b['y']-a['y']) for a,b in zip(route,route[1:])),
                settled_samples=len(settled))

def stop(process):
    try:os.killpg(process.pid,signal.SIGINT)
    except ProcessLookupError:return
    try:process.wait(timeout=12)
    except subprocess.TimeoutExpired:pass
    try:os.killpg(process.pid,signal.SIGKILL)
    except ProcessLookupError:pass
    process.wait(timeout=5)

