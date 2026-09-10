#!/usr/bin/env python3
"""Independent, fail-closed velocity gate. Starts DISABLED on every launch."""
import json
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool
from asr_summer_school.mission_core import limited_command


class MotionGuard(Node):
    def __init__(self):
        super().__init__('motion_guard')
        self.last_report=0.
        self.report=self.create_publisher(String,'/safety/status',10)
        self.enabled = False
        self.permit = self.scan_at = self.command_at = -math.inf
        self.nearest = 0.
        self.command = Twist()
        self.output = self.create_publisher(Twist,'/cmd_vel',10)
        self.state = self.create_publisher(Bool,'/safety/enabled',10)
        self.create_subscription(Twist,'/autonomy/cmd_vel',self.velocity,10)
        self.create_subscription(Bool,'/mission/permit',self.heartbeat,10)
        self.create_subscription(LaserScan,'/scan',self.scan,qos_profile_sensor_data)
        self.create_service(SetBool,'/safety/enable',self.enable)
        self.create_timer(.05,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))

    def velocity(self,msg):
        self.command,self.command_at = msg,time.monotonic()

    def heartbeat(self,msg):
        self.permit = time.monotonic() if msg.data else -math.inf

    def scan(self,msg):
        valid = [r for r in msg.ranges if math.isfinite(r) and msg.range_min<=r<=msg.range_max]
        # All-infinity scans may indicate open space, but are not trusted at startup.
        self.nearest = min(valid) if valid else 0.
        self.scan_at = time.monotonic()

    def enable(self,request,response):
        if request.data and (time.monotonic()-self.scan_at>.7 or self.nearest<.20):
            response.success=False
            response.message='Cannot arm: scan missing or obstacle within 20 cm of laser'
            return response
        self.enabled=request.data
        self.command=Twist()
        self.command_at=-math.inf
        self.output.publish(Twist())
        response.success=True
        response.message='ARMED' if self.enabled else 'STOPPED'
        return response

    def tick(self):
        now=time.monotonic()
        v,w=limited_command(self.command.linear.x,self.command.angular.z,self.enabled,
                            now-self.permit,now-self.scan_at,now-self.command_at,self.nearest)
        out=Twist();out.linear.x=v;out.angular.z=w
        self.output.publish(out)
        self.state.publish(Bool(data=self.enabled))
        if now-self.last_report>=1.:
            reasons=[]
            if not self.enabled:reasons.append('disarmed')
            if now-self.permit>.7:reasons.append('permit_stale')
            if now-self.scan_at>.7:reasons.append('scan_stale')
            if now-self.command_at>.5:reasons.append('command_stale')
            if self.nearest<.20:reasons.append('obstacle_under_20cm')
            self.report.publish(String(data=json.dumps({'blocked_by':reasons,
                'nearest_m':self.nearest, 'input_linear':self.command.linear.x,
                'output_linear':v,'output_angular':w})))
            self.last_report=now


def main():
    rclpy.init();node=MotionGuard()
    try:rclpy.spin(node)
    except KeyboardInterrupt:pass
    finally:
        node.output.publish(Twist());node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
