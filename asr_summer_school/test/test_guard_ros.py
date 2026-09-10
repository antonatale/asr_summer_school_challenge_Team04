"""Real ROS transport integration; run only on localhost test domain 87."""
import os
import subprocess
import signal
import time
import unittest
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class GuardIntegration(unittest.TestCase):
    def test_live_gate(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'87')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'),'1')
        process=subprocess.Popen(['ros2','run','asr_summer_school','motion_guard.py'],start_new_session=True)
        rclpy.init();n=Node('guard_test');outputs=[]
        scan=n.create_publisher(LaserScan,'/scan',10)
        permit=n.create_publisher(Bool,'/mission/permit',10)
        velocity=n.create_publisher(Twist,'/autonomy/cmd_vel',10)
        n.create_subscription(Twist,'/cmd_vel',lambda msg:outputs.append((time.monotonic(),msg.linear.x,msg.angular.z)),10)
        client=n.create_client(SetBool,'/safety/enable')
        def pump(seconds, heartbeat=True, scans=True, commands=True, distance=2.):
            end=time.monotonic()+seconds
            while time.monotonic()<end:
                if scans:
                    msg=LaserScan();msg.range_min=.1;msg.range_max=10.;msg.ranges=[distance]*360;scan.publish(msg)
                if heartbeat:permit.publish(Bool(data=True))
                if commands:
                    msg=Twist();msg.linear.x=.8;msg.angular.z=2.;velocity.publish(msg)
                rclpy.spin_once(n,timeout_sec=.02)
        try:
            self.assertTrue(client.wait_for_service(timeout_sec=10.))
            pump(1.)
            self.assertTrue(outputs)
            self.assertTrue(all(v==0 and w==0 for _,v,w in outputs))
            future=client.call_async(SetBool.Request(data=True))
            rclpy.spin_until_future_complete(n,future,timeout_sec=3.)
            self.assertTrue(future.result().success)
            outputs.clear();pump(.5)
            self.assertTrue(any(v>0 for _,v,w in outputs))
            self.assertTrue(all(v<=.12 and abs(w)<=.45 for _,v,w in outputs))
            outputs.clear();pump(1.2,heartbeat=False)
            self.assertTrue(all(v==0 and w==0 for _,v,w in outputs[-5:]))
            pump(.3);outputs.clear();pump(1.2,scans=False)
            self.assertTrue(all(v==0 and w==0 for _,v,w in outputs[-5:]))
            pump(.3);outputs.clear();pump(1.,commands=False)
            self.assertTrue(all(v==0 and w==0 for _,v,w in outputs[-5:]))
            outputs.clear();pump(.5,distance=.15)
            self.assertTrue(all(v==0 and w==0 for _,v,w in outputs[-5:]))
            future=client.call_async(SetBool.Request(data=False))
            rclpy.spin_until_future_complete(n,future,timeout_sec=3.)
            self.assertTrue(future.result().success)
            outputs.clear();pump(.5)
            self.assertTrue(all(v==0 and w==0 for _,v,w in outputs[-5:]))
        finally:
            os.killpg(process.pid,signal.SIGINT);process.wait(timeout=5.)
            n.destroy_node();rclpy.shutdown()

if __name__=='__main__':unittest.main()
