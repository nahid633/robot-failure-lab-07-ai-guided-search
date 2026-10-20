import copy,math,unittest
from core import *
from trial import payload_hash
from sensor_msgs.msg import LaserScan
from rclpy.serialization import serialize_message,deserialize_message


class Checks(unittest.TestCase):
    def test_half_open_fault_and_fresh_recovery(self):
        for c in CONDITIONS:
            self.assertEqual(policy(c,4.999,5.),'fresh')
            self.assertEqual(policy(c,5.,5.),c)
            self.assertEqual(policy(c,6.999,5.),c)
            self.assertEqual(policy(c,7.,5.),'fresh')
    def test_invalid_condition(self):
        with self.assertRaises(ValueError):policy('unknown',5,5)
    def test_restamping_does_not_change_payload_hash(self):
        m=LaserScan();m.header.frame_id='base_scan';m.header.stamp.sec=1;m.ranges=[1.,2.,float('inf')]
        old=payload_hash(m);n=copy.deepcopy(m);n.header.stamp.sec=3
        self.assertEqual(old,payload_hash(n));n.ranges[0]=.8;self.assertNotEqual(old,payload_hash(n))
        self.assertEqual(m.header.stamp.sec,1)
    def test_payload_hash_survives_cdr_roundtrip_and_padding(self):
        m=LaserScan();m.header.frame_id='base_scan';m.header.stamp.sec=6;m.header.stamp.nanosec=201000000
        m.ranges=[float(i)/7 for i in range(360)];m.intensities=[0.]*360;m.angle_increment=.01745
        for i in range(12):
            received=deserialize_message(serialize_message(m),LaserScan)
            self.assertEqual(payload_hash(received),payload_hash(m))
    def test_polygon_clearance_not_center_distance(self):
        a=rectangle(0,0,0,(-.5,.5,-.5,.5));b=rectangle(2,0,0,(-.5,.5,-.5,.5))
        self.assertAlmostEqual(polygon_clearance(a,b),1.)
        d=rectangle(2,2,0,(-.5,.5,-.5,.5));self.assertAlmostEqual(polygon_clearance(a,d),math.sqrt(2))
    def test_yaw_changes_envelope_clearance(self):
        b=rectangle(0,1,0,(-.25,.25,-.25,.25))
        self.assertGreater(polygon_clearance(rectangle(0,0,0,(-1,1,-.1,.1)),b),.6)
        self.assertEqual(polygon_clearance(rectangle(0,0,math.pi/2,(-1,1,-.1,.1)),b),0)
    def test_overlap_and_touch_are_zero_not_contact_evidence(self):
        a=rectangle(0,0,0,(-.5,.5,-.5,.5))
        self.assertEqual(polygon_clearance(a,a),0)
        self.assertEqual(polygon_clearance(a,rectangle(1,0,0,(-.5,.5,-.5,.5))),0)
    def test_sparse_or_rotating_motion_does_not_count_stop(self):
        self.assertIsNone(sustained_stop([dict(stamp=i*.2,speed=0,w=0) for i in range(5)],0,1))
        self.assertIsNone(sustained_stop([dict(stamp=i*.05,speed=0,w=.1) for i in range(10)],0,1))
        self.assertAlmostEqual(sustained_stop([dict(stamp=i*.05,speed=0,w=0) for i in range(10)],0,1),0)
    def test_no_motion_stop_is_missing_not_zero_latency(self):
        truth=[dict(stamp=i*.05,x=-1.2+i*.001,y=.55,yaw=0) for i in range(30)]
        odom=[dict(stamp=i*.05,speed=.1,w=0) for i in range(30)]
        r=summarize(truth,odom,0,.2,1,1.4)
        self.assertIsNone(r['stop_after_appearance_s']);self.assertIsNone(r['path_to_first_stop_m'])
    def test_routing_does_not_mutate_upstream(self):
        a={'amcl':{'ros__parameters':{'scan_topic':'scan'}},'other':{'topic':'/scan'}};b=copy.deepcopy(a)
        r=routed_config(a);self.assertEqual(a,b);self.assertEqual(r['other']['topic'],TOPICS['obstacles'])


if __name__=='__main__':unittest.main()
