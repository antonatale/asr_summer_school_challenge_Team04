#!/usr/bin/env python3
"""Supervised exploration baseline: explicit start, deadline, bounded Nav2 actions."""
import json
import math
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock, ClockType
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from sensor_msgs.msg import LaserScan, BatteryState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger, SetBool
from tf2_ros import Buffer, TransformListener, TransformException
from landmark_msgs.msg import LandmarkArray
from asr_summer_school.mission_core import Grid, TagStore, yaw, reserve_seconds, atomic_json


class Mission(Node):
    def __init__(self):
        super().__init__('mission_orchestrator')
        self.duration=float(self.declare_parameter('duration',180.).value)
        self.radius=float(self.declare_parameter('max_radius',2.).value)
        self.margin=float(self.declare_parameter('return_margin',25.).value)
        # A momentary loss of the internal route must not end exploration; the
        # reserve is kept with a pessimistic estimate during the grace window.
        self.route_tolerance=float(self.declare_parameter('return_route_tolerance',.25).value)
        self.route_grace=float(self.declare_parameter('return_route_grace',6.).value)
        self.detour=float(self.declare_parameter('return_detour_factor',1.6).value)
        if not (30<=self.duration<=3600 and .5<=self.radius<=30 and 5<=self.margin<self.duration):
            raise ValueError('Invalid duration, radius or return margin')
        if not (0<=self.route_tolerance<=.3 and 0<=self.route_grace<=.5*self.margin and self.detour>=1.):
            raise ValueError('Invalid return route tolerance, grace or detour factor')
        self.folder=Path(os.path.expanduser(str(self.declare_parameter('output_dir','~/mission_runs').value))) / time.strftime('%Y%m%d-%H%M%S')
        self.folder.mkdir(parents=True,exist_ok=False)
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.nav=ActionClient(self,NavigateToPose,'/navigate_to_pose')
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose')
        self.guard=self.create_client(SetBool,'/safety/enable')
        self.permit=self.create_publisher(Bool,'/mission/permit',10)
        self.status=self.create_publisher(String,'/mission/status',10)
        self.create_subscription(OccupancyGrid,'/map',self.on_map,QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(LaserScan,'/scan',self.on_scan,qos_profile_sensor_data)
        self.create_subscription(BatteryState,'/battery_state',self.on_battery,qos_profile_sensor_data)
        self.create_subscription(LandmarkArray,'/camera/landmarks',self.on_tags,10)
        self.create_subscription(Bool,'/safety/enabled',self.on_guard,10)
        self.create_service(Trigger,'/mission/start',self.start)
        self.create_service(Trigger,'/mission/stop',self.stop)
        self.create_service(Trigger,'/mission/snapshot',self.snapshot)
        self.grid=self.map_msg=None;self.map_at=self.scan_at=-math.inf
        self.battery=None;self.enabled=False;self.guard_at=-math.inf
        self.tags=TagStore();self.blacklist=[]
        self.state='WAIT_READY';self.running=False;self.home=None;self.home_odom=None
        self.started=None;self.returning=False;self.return_attempts=0
        self.goal=None;self.future=None;self.result_future=None
        self.pending_kind=None;self.pending_at=0.;self.cancel_at=None
        self.target=None;self.plan_queue=[];self.goal_at=0.;self.last_plan=0.
        self.last_health=0.;self.last_checkpoint=0.;self.no_frontiers=0
        self.return_distance=math.inf;self.grid_cache=None;self.home_reached=False
        self.route_ok=False;self.route_lost_at=None
        self.last_status=0.;self.failure=None
        self.create_timer(.1,self.tick,clock=Clock(clock_type=ClockType.STEADY_TIME))
        self.event('ready_to_check',output=str(self.folder))

    def event(self,event,**data):
        record={'event':event,'state':self.state,'monotonic':time.monotonic(),**data}
        with (self.folder/'events.jsonl').open('a') as stream:
            stream.write(json.dumps(record,allow_nan=False)+'\n')
        self.get_logger().info(json.dumps(record))

    def on_map(self,msg):
        if msg.header.frame_id != 'map':return
        try:
            self.grid=Grid(msg.data,msg.info.width,msg.info.height,msg.info.resolution,
                           (msg.info.origin.position.x,msg.info.origin.position.y,yaw(msg.info.origin.orientation)))
            self.map_msg=msg;self.map_at=time.monotonic();self.last_health=0.;self.grid_cache=None
        except ValueError as error:self.get_logger().warning(str(error))

    def on_scan(self,msg):
        if msg.ranges:self.scan_at=time.monotonic()

    def on_battery(self,msg):
        self.battery=float(msg.voltage) if math.isfinite(msg.voltage) and msg.voltage>0 else None

    def on_guard(self,msg):
        self.enabled=msg.data;self.guard_at=time.monotonic()

    def pose(self,target='map',source='base_footprint',stamp=None):
        try:
            tf=self.tf.lookup_transform(target,source,stamp or Time(),timeout=Duration(seconds=0.))
            if stamp is None and source=='base_footprint':
                age=(self.get_clock().now()-Time.from_msg(tf.header.stamp)).nanoseconds/1e9
                if age<-.2 or age>1.:return None
            t=tf.transform.translation
            return (t.x,t.y,yaw(tf.transform.rotation))
        except TransformException:return None

    def home_pose(self):
        # Anchor home in odom to follow current map->odom corrections.
        if self.home_odom is None:return self.home
        transform=self.pose('map','odom')
        if transform is None:return self.home
        x,y,a=transform;hx,hy,ha=self.home_odom
        return (x+math.cos(a)*hx-math.sin(a)*hy,y+math.sin(a)*hx+math.cos(a)*hy,a+ha)

    def on_tags(self,msg):
        if not self.running:return
        stamp=Time.from_msg(msg.header.stamp)
        age=(self.get_clock().now()-stamp).nanoseconds/1e9
        if age<-.2 or age>1.:return
        transform=self.pose('map',msg.header.frame_id,stamp)
        if transform is None:return
        x,y,a=transform
        for tag in msg.landmarks:
            if tag.hamming!=0 or not (.1<tag.range<5.) or not math.isfinite(tag.bearing):continue
            self.tags.add(tag.id,x+tag.range*math.cos(a+tag.bearing),y+tag.range*math.sin(a+tag.bearing),stamp.nanoseconds/1e9)

    def ready(self):
        now=time.monotonic()
        return (self.grid is not None and now-self.map_at<12 and now-self.scan_at<.7
                and self.pose() is not None and self.nav.server_is_ready()
                and self.planner.server_is_ready() and now-self.guard_at<1.)

    def start(self,request,response):
        if self.running or self.started is not None:
            response.message='Restart mission process for a fresh run';return response
        if not self.ready() or not self.enabled:
            response.message='Not ready: map, scan, TF, Nav2 and explicitly armed safety gate required';return response
        self.home=self.pose();self.home_odom=self.pose('odom','base_footprint')
        if self.home_odom is None:
            response.message='Fresh odometry TF required';return response
        self.started=time.monotonic();self.running=True;self.state='EXPLORE'
        self.event('started',home=self.home,duration=self.duration,radius=self.radius)
        response.success=True;response.message='Mission started';return response

    def snapshot(self,request,response):
        self.checkpoint()
        response.success=(self.folder/'map.yaml').exists() and (self.folder/'tags.json').exists()
        response.message=str(self.folder)
        return response

    def stop(self,request,response):
        self.halt('operator_stop')
        response.success=True;response.message='Stop requested; motion permit revoked';return response

    def disarm(self):
        self.permit.publish(Bool(data=False))
        if self.guard.service_is_ready():self.guard.call_async(SetBool.Request(data=False))

    def halt(self,reason):
        if self.state in ('DONE','STOPPED','SAVE_FAILED') and not self.running:
            self.disarm();return
        self.failure=reason;self.running=False;self.state='STOPPED';self.disarm()
        if self.goal is not None:self.goal.cancel_goal_async()
        self.event('stopped',reason=reason)
        self.checkpoint(final=True)

    def checkpoint(self,final=False):
        try:
            atomic_json(self.folder/'tags.json',{'frame_id':'map','tags':self.tags.export()})
            if self.map_msg is not None:
                # Versioned image first, YAML pointer atomically replaced last.
                image_name='map-%d.pgm'%time.monotonic_ns()
                a=self.grid.a
                pixels=np.full(a.shape,205,dtype=np.uint8)
                pixels[(a>=0)&(a<=20)]=254;pixels[a>=65]=0
                with (self.folder/image_name).open('wb') as stream:
                    stream.write(('P5\n%d %d\n255\n'%(self.grid.w,self.grid.h)).encode())
                    stream.write(np.flipud(pixels).tobytes());stream.flush();os.fsync(stream.fileno())
                text=('image: %s\nresolution: %.9f\norigin: [%s]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n')%(image_name,self.grid.res,', '.join(map(str,self.grid.origin)))
                tmp=self.folder/'map.yaml.tmp';tmp.write_text(text);os.replace(tmp,self.folder/'map.yaml')
            atomic_json(self.folder/'summary.json',{'state':self.state,'home_reached':self.home_reached,
                        'failure':self.failure,'tags':len(self.tags.export()),'final':final,
                        'elapsed':time.monotonic()-self.started if self.started is not None else 0.,
                        'home':self.home_pose(),'map_saved':self.map_msg is not None})
        except (OSError,ValueError) as error:
            self.get_logger().error('EXPORT FAILED: '+str(error))
            if final:self.state='SAVE_FAILED'

    def stamped(self,target):
        p=PoseStamped();p.header.frame_id='map';p.header.stamp=self.get_clock().now().to_msg()
        p.pose.position.x=float(target['x']);p.pose.position.y=float(target['y'])
        p.pose.orientation.z=math.sin(target['yaw']/2);p.pose.orientation.w=math.cos(target['yaw']/2)
        return p

    def plan(self,target):
        self.target=target
        request=ComputePathToPose.Goal();request.goal=self.stamped(target);request.planner_id='GridBased'
        self.future=self.planner.send_goal_async(request);self.pending_kind='plan_accept';self.pending_at=time.monotonic()

    def reject(self,reason):
        self.event('goal_skipped',reason=reason,target=self.target)
        if self.target is not None and not self.returning:
            self.blacklist.append((self.target['x'],self.target['y'],time.monotonic()+25))
        if self.returning:
            self.return_attempts+=1
            if self.return_attempts>=3:self.halt('return_path_failed')
        self.future=self.result_future=self.goal=None;self.pending_kind=None;self.last_plan=time.monotonic()

    def begin_return(self,reason):
        if self.returning:return
        self.returning=True;self.state='RETURN_HOME';self.plan_queue=[]
        self.event('return_started',reason=reason)
        if self.goal is not None and self.pending_kind=='nav_result':
            self.goal.cancel_goal_async();self.cancel_at=time.monotonic()
        # Pending acceptance or planning is handled below; never send two navigation goals.

    def process_future(self,now):
        if self.future is None:return
        kind=self.pending_kind
        if now-self.pending_at>8 and kind!='nav_result':
            self.halt('action_timeout_'+kind);return
        if not self.future.done():return
        try:value=self.future.result()
        except Exception as error:self.halt('action_error_'+str(error));return
        if kind in ('plan_accept','nav_accept'):
            if not value.accepted:self.reject('rejected');return
            self.goal=value;self.future=value.get_result_async()
            self.pending_kind='plan_result' if kind=='plan_accept' else 'nav_result'
            self.pending_at=now
            if kind=='nav_accept':
                self.goal_at=now
                if self.returning and not self.target.get('home'):
                    value.cancel_goal_async();self.cancel_at=now
            return
        if kind=='plan_result':
            path=value.result.path.poses
            if value.status!=GoalStatus.STATUS_SUCCEEDED or not path:
                self.reject('no_path');return
            if self.returning and not self.target.get('home'):
                self.future=self.goal=None;self.pending_kind=None;return
            distance=sum(math.hypot(b.pose.position.x-a.pose.position.x,b.pose.position.y-a.pose.position.y) for a,b in zip(path,path[1:]))
            if not self.returning and (not math.isfinite(self.return_distance) or now-self.started+distance/.09+reserve_seconds(self.return_distance,margin=self.margin)>=self.duration):
                self.future=self.goal=None;self.pending_kind=None;self.begin_return('candidate_exceeds_budget');return
            request=NavigateToPose.Goal();request.pose=self.stamped(self.target)
            self.future=self.nav.send_goal_async(request);self.pending_kind='nav_accept';self.pending_at=now
            self.nav_timeout=min(45.,max(15.,distance/.09+8.))
            self.goal=None;self.event('goal_sent',target=self.target,path_m=distance);return
        if kind=='nav_result':
            self.event('goal_result',status=value.status,target=self.target)
            was_home=self.target.get('home',False)
            self.future=self.goal=None;self.pending_kind=None;self.cancel_at=None;self.last_plan=now
            if was_home:
                position=self.pose();home=self.home_pose()
                if value.status==GoalStatus.STATUS_SUCCEEDED and position and math.hypot(position[0]-home[0],position[1]-home[1])<=.4:
                    self.home_reached=True;self.running=False;self.state='DONE';self.disarm();self.checkpoint(final=True)
                else:self.reject('return_not_verified')
            else:
                self.blacklist.append((self.target['x'],self.target['y'],now+25))

    def tick(self):
        now=time.monotonic()
        healthy=self.ready()
        active=self.running and healthy and self.enabled
        self.permit.publish(Bool(data=active))
        if now-self.last_status>1:
            self.status.publish(String(data=json.dumps({'state':self.state,'ready':healthy,'armed':self.enabled,
                    'tags':len(self.tags.export()),'remaining':max(0,self.duration-(now-self.started)) if self.started else self.duration})))
            self.last_status=now
        if not self.running:return
        if not healthy or not self.enabled:
            self.event('health_failure',map_age=now-self.map_at,scan_age=now-self.scan_at,guard_age=now-self.guard_at,pose_available=self.pose() is not None,nav_available=self.nav.server_is_ready(),planner_available=self.planner.server_is_ready(),enabled=self.enabled)
            self.halt('sensors_tf_nav_or_guard_unavailable');return
        if self.battery is not None and self.battery<10.8:
            self.halt('low_battery');return
        if now-self.started>=self.duration:
            self.halt('deadline');return
        if now-self.last_checkpoint>5:
            self.checkpoint();self.last_checkpoint=now
        position=self.pose();home=self.home_pose()
        if position is None or home is None:self.halt('pose_lost');return
        if self.grid_cache is None or now-self.last_health>2:
            _,costs=self.grid.routes(position)
            self.grid_cache=costs
            measured=self.grid.distance_to(costs,home,self.route_tolerance)
            self.last_health=now
            direct=math.hypot(position[0]-home[0],position[1]-home[1])
            if math.isfinite(measured) or direct<.20:
                self.return_distance=measured if math.isfinite(measured) else 0.
                self.route_ok=True;self.route_lost_at=None
            else:
                # A growing SLAM map, a map->odom correction or a single
                # re-marked cell can drop the internal route for a moment while
                # Nav2 still navigates.  Budget the return with a pessimistic
                # estimate instead of an infinite one and keep exploring, but
                # give up on a route that stays unavailable.
                if self.route_lost_at is None:
                    self.route_lost_at=now
                    self.event('return_route_unavailable',position=position,home=home,
                               **self.grid.route_report(costs,position,home,self.route_tolerance))
                self.return_distance=max(direct*self.detour,
                                         self.return_distance if math.isfinite(self.return_distance) else 0.)
                self.route_ok=False
                if now-self.route_lost_at>=self.route_grace:
                    self.event('return_route_lost',seconds=now-self.route_lost_at,estimate=self.return_distance)
                    self.begin_return('return_route_unavailable')
            # The reserve is checked on the estimate in use, measured or not.
            if math.isfinite(self.return_distance) and now-self.started+reserve_seconds(self.return_distance,margin=self.margin)>=self.duration:
                self.begin_return('return_reserve')
            if direct>self.radius+.2:
                self.begin_return('radius_limit')
        if now-self.started >= self.duration-self.margin:
            self.begin_return('return_margin')
        if self.cancel_at is not None and now-self.cancel_at>3:
            self.halt('cancel_not_confirmed');return
        self.process_future(now)
        if not self.running or self.future is not None:
            if self.running and self.pending_kind=='nav_result' and now-self.goal_at>getattr(self,"nav_timeout",25.) and self.cancel_at is None:
                self.goal.cancel_goal_async();self.cancel_at=now
            return
        if now-self.last_plan<2:return
        self.last_plan=now
        if self.returning:
            if math.hypot(position[0]-home[0],position[1]-home[1])<.30:
                self.home_reached=True;self.state='DONE';self.running=False;self.disarm();self.checkpoint(final=True);return
            self.plan({'x':home[0],'y':home[1],'yaw':home[2],'home':True});return
        candidates=self.grid.candidates(position,self.grid_cache,self.blacklist,now)
        candidates=[c for c in candidates if math.hypot(c['x']-home[0],c['y']-home[1])<=self.radius]
        if not candidates:
            self.no_frontiers+=1
            # Known free space can have no frontier after SLAM has already
            # marked the nearby area.  Patrol reachable cells before giving up.
            candidates=self.grid.patrol_candidates(position,self.grid_cache,self.blacklist,now,
                                                   max_radius=min(self.radius,3.0))
            candidates=[c for c in candidates if math.hypot(c["x"]-home[0],c["y"]-home[1])<=self.radius]
            if candidates:
                self.event('frontier_fallback',count=len(candidates),attempt=self.no_frontiers)
            elif self.no_frontiers>=10:
                self.event('frontier_exhausted',attempt=self.no_frontiers)
                self.begin_return('no_reachable_frontiers')
            else:
                return
            if not candidates:
                return
        self.no_frontiers=0;self.plan(candidates[0])


def main():
    rclpy.init();node=Mission()
    try:rclpy.spin(node)
    except KeyboardInterrupt:node.halt('keyboard_interrupt')
    except Exception as error:
        node.halt('unhandled_'+str(error));raise
    finally:
        node.disarm();node.destroy_node();rclpy.try_shutdown()

if __name__=='__main__':main()
