#!/usr/bin/env python3
"""Read-only ROS recorder. Run with robot_mission.sh diagnose [seconds]."""
import json
import time
from collections import defaultdict, Counter
from diagnostic_metrics import WindowMetrics, graph_warnings
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
    counts=defaultdict(int);peaks=defaultdict(float);latest={};start=time.monotonic()
    metrics=WindowMetrics();graph={};next_graph=start;warnings_seen=set()
    tag_ids=set();landmark_ids=set()
    def cmd(topic,m):
        counts[topic]+=1
        peaks[topic+'/linear']=max(peaks[topic+'/linear'],abs(m.linear.x))
        peaks[topic+'/angular']=max(peaks[topic+'/angular'],abs(m.angular.z))
        counts[topic+'/nonzero_linear']+=int(abs(m.linear.x)>.001)
    def odom(m):
        counts['odom']+=1;p=m.pose.pose.position
        stamp=m.header.stamp.sec+m.header.stamp.nanosec/1e9
        metrics.odom(stamp,p.x,p.y,m.twist.twist.linear.x,m.twist.twist.angular.z)
    def scan(m):
        counts['scan']+=1
        nearest=metrics.scan(m.ranges,m.range_min,m.range_max)
        latest['scan_min_m']=nearest
        counts['scan_under_20cm']+=int(nearest is not None and nearest<.20)
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
            if time.monotonic()>=next_graph:
                nodes=Counter((ns.rstrip('/')+'/'+name) for name,ns in n.get_node_names_and_namespaces())
                duplicates=sorted(name for name,count in nodes.items() if count>1)
                publishers=[p.node_namespace.rstrip('/')+'/'+p.node_name for p in n.get_publishers_info_by_topic('/cmd_vel')]
                graph={'cmd_vel_publishers':publishers,'duplicate_nodes':duplicates}
                warnings_seen.update(graph_warnings(publishers,duplicates))
                next_graph=time.monotonic()+5.
        print(json.dumps(dict(seconds=time.monotonic()-start,counts=dict(counts),peaks=dict(peaks),
                              **metrics.report(),graph=graph,warnings=sorted(warnings_seen),
                              detected_ids=sorted(tag_ids),localized_ids=sorted(landmark_ids),latest=latest),indent=2))
    finally:
        n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
