#!/usr/bin/env python3
"""Read-only ROS recorder. Run with robot_mission.sh diagnose [seconds]."""
import json
import math
import time
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image, CameraInfo
from std_msgs.msg import String
from apriltag_msgs.msg import AprilTagDetectionArray
from landmark_msgs.msg import LandmarkArray


def main():
    import argparse
    parser=argparse.ArgumentParser()
    parser.add_argument('seconds',type=float,nargs='?',default=15.)
    args=parser.parse_args()
    rclpy.init();n=Node('mission_read_only_diagnostics')
    counts=defaultdict(int);peaks=defaultdict(float);latest={};positions=[];start=time.monotonic()
    tag_ids=set();landmark_ids=set()
    def cmd(topic,m):
        counts[topic]+=1
        peaks[topic+'/linear']=max(peaks[topic+'/linear'],abs(m.linear.x))
        peaks[topic+'/angular']=max(peaks[topic+'/angular'],abs(m.angular.z))
        counts[topic+'/nonzero_linear']+=int(abs(m.linear.x)>.001)
    def odom(m):
        counts['odom']+=1;p=m.pose.pose.position;positions.append((p.x,p.y))
    def scan(m):
        counts['scan']+=1
        valid=[r for r in m.ranges if math.isfinite(r) and m.range_min<=r<=m.range_max]
        latest['scan_min_m']=min(valid) if valid else None
        counts['scan_under_20cm']+=int(bool(valid) and min(valid)<.20)
    def msg(topic,m):
        counts[topic]+=1
        if topic=='image':latest['image']=[m.width,m.height,m.encoding]
        if topic=='detections':tag_ids.update(t.id for t in m.detections)
        if topic=='landmarks':landmark_ids.update(t.id for t in m.landmarks)
        if topic=='mission':latest['mission']=json.loads(m.data)
        if topic=='guard':latest['guard']=json.loads(m.data)
    for topic in ['/autonomy/cmd_vel','/cmd_vel']:
        n.create_subscription(Twist,topic,lambda m,t=topic:cmd(t,m),qos_profile_sensor_data)
    n.create_subscription(Odometry,'/odom',odom,qos_profile_sensor_data)
    n.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    for typ,topic,key in [(Image,'/camera/color/image_raw','image'),
                          (CameraInfo,'/camera/color/camera_info','camera_info'),
                          (AprilTagDetectionArray,'/camera/detections','detections'),
                          (LandmarkArray,'/camera/landmarks','landmarks'),
                          (String,'/mission/status','mission'),(String,'/safety/status','guard')]:
        n.create_subscription(typ,topic,lambda m,k=key:msg(k,m),qos_profile_sensor_data)
    try:
        while rclpy.ok() and time.monotonic()-start<min(180.,max(1.,args.seconds)):
            rclpy.spin_once(n,timeout_sec=.1)
        displacement=math.dist(positions[0],positions[-1]) if positions else None
        path=sum(math.dist(a,b) for a,b in zip(positions,positions[1:])) if positions else None
        print(json.dumps(dict(seconds=time.monotonic()-start,counts=dict(counts),peaks=dict(peaks),
                              odom_displacement_m=displacement,odom_path_m=path,
                              detected_ids=sorted(tag_ids),localized_ids=sorted(landmark_ids),latest=latest),indent=2))
    finally:
        n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
