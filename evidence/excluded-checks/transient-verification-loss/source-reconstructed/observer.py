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

