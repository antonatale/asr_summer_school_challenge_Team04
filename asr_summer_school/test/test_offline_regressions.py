"""Run actual Mission methods without ROS, with injected transport stand-ins.

These tests verify policy/control flow, not ROS ABI, DDS or physical navigation.
"""
import ast
import json
import math
import time
import unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock, patch
import numpy as np
import yaml
from asr_summer_school.mission_core import Grid, TagStore, reserve_seconds

ROOT = Path(__file__).resolve().parents[1]
tree = ast.parse((ROOT/'asr_summer_school/mission_orchestrator.py').read_text())
cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Mission')
cls.bases = []
env = dict(time=time, math=math, json=json, Bool=NS, String=NS, reserve_seconds=reserve_seconds,
           GoalStatus=NS(STATUS_SUCCEEDED=4))
exec(compile(ast.Module(body=[cls], type_ignores=[]), '<actual Mission methods>', 'exec'), env)
Mission = env['Mission']


def mission():
    n = object.__new__(Mission)
    for k,v in dict(running=True, state='EXPLORE', enabled=True, duration=150., started=90.,
                    margin=25., radius=5., battery=None, last_status=100., last_checkpoint=100.,
                    last_health=100., last_plan=90., returning=False, cancel_at=None, future=None,
                    no_frontiers=0, return_distance=0., home_reached=False, failure=None,
                    pending_kind=None, goal=None, return_attempts=0, blacklist=[], plan_queue=[],
                    route_tolerance=.25, route_grace=6., detour=1.6, route_ok=True,
                    route_lost_at=None).items():
        setattr(n,k,v)
    n.grid_cache=np.zeros((10,10))
    n.grid=Mock();n.grid.candidates.return_value=[];n.grid.patrol_candidates.return_value=[]
    n.grid.route_report.return_value={}
    n.pose=Mock(return_value=(0.,0.,0.));n.home_pose=Mock(return_value=(0.,0.,0.))
    n.ready=Mock(return_value=True)
    n.event=Mock();n.permit=Mock();n.status=Mock();n.disarm=Mock();n.checkpoint=Mock();n.plan=Mock()
    n.tags=TagStore()
    return n


class Regressions(unittest.TestCase):
    @patch('time.monotonic', return_value=100.)
    def test_empty_candidates_tenth_attempt_returns_without_crash(self, _):
        n=mission();n.no_frontiers=9;n.tick()
        self.assertEqual(n.state,'RETURN_HOME');n.plan.assert_not_called()

    @patch('time.monotonic', return_value=100.)
    def test_patrol_cannot_escape_home_radius(self, _):
        n=mission();n.radius=1.;n.grid.patrol_candidates.return_value=[dict(x=1.1,y=0.,yaw=0.)]
        n.tick();n.plan.assert_not_called()

    @patch('time.monotonic', return_value=100.)
    def test_patrol_valid_goal_is_used(self, _):
        n=mission();c=dict(x=.8,y=0.,yaw=0.);n.grid.patrol_candidates.return_value=[c]
        n.tick();n.plan.assert_called_once_with(c)

    @patch('time.monotonic', return_value=100.)
    def test_missing_return_route_at_home_is_not_infinite_reserve(self, _):
        n=mission();n.grid_cache=None;n.grid.routes.return_value=(None,np.zeros((10,10)))
        n.grid.distance_to.return_value=math.inf
        n.tick();self.assertEqual(n.return_distance,0.);self.assertEqual(n.state,'EXPLORE')

    @patch('time.monotonic', return_value=100.)
    def test_missing_return_route_keeps_exploring_during_grace(self, _):
        n=mission();n.pose.return_value=(1.,0.,0.);n.grid_cache=None
        n.grid.routes.return_value=(None,np.zeros((10,10)));n.grid.distance_to.return_value=math.inf
        n.tick()
        self.assertEqual(n.state,'EXPLORE');self.assertFalse(n.route_ok)
        # Budget with a pessimistic estimate, never with an infinite one.
        self.assertAlmostEqual(n.return_distance,1.6);self.assertEqual(n.route_lost_at,100.)

    @patch('time.monotonic', return_value=100.)
    def test_missing_return_route_sends_home_after_grace(self, _):
        n=mission();n.pose.return_value=(1.,0.,0.);n.grid_cache=None;n.route_lost_at=90.
        n.grid.routes.return_value=(None,np.zeros((10,10)));n.grid.distance_to.return_value=math.inf
        n.tick();self.assertEqual(n.state,'RETURN_HOME');self.assertTrue(n.plan.call_args.args[0]['home'])

    @patch('time.monotonic', return_value=100.)
    def test_missing_return_route_still_honours_the_time_reserve(self, _):
        # Estimate 1.6 m -> 1.6/.09+25 = 42.8 s of reserve, 110 s already spent
        # of 150: the return starts on the reserve, before the grace expires.
        n=mission();n.pose.return_value=(1.,0.,0.);n.grid_cache=None;n.started=-10.
        n.grid.routes.return_value=(None,np.zeros((10,10)));n.grid.distance_to.return_value=math.inf
        n.tick();self.assertEqual(n.state,'RETURN_HOME')
        self.assertEqual(n.event.call_args_list[-1].kwargs['reason'],'return_reserve')

    @patch('time.monotonic', return_value=100.)
    def test_recovered_route_clears_the_unavailable_state(self, _):
        n=mission();n.pose.return_value=(1.,0.,0.);n.grid_cache=None;n.route_lost_at=99.
        n.grid.routes.return_value=(None,np.zeros((10,10)));n.grid.distance_to.return_value=1.2
        n.tick()
        self.assertEqual(n.state,'EXPLORE');self.assertTrue(n.route_ok)
        self.assertIsNone(n.route_lost_at);self.assertEqual(n.return_distance,1.2)

    @patch('time.monotonic', return_value=100.)
    def test_home_route_is_measured_with_the_goal_tolerance(self, _):
        n=mission();n.pose.return_value=(1.,0.,0.);n.grid_cache=None
        n.grid.routes.return_value=(None,np.zeros((10,10)));n.grid.distance_to.return_value=1.2
        n.tick();self.assertEqual(n.grid.distance_to.call_args.args[2],n.route_tolerance)

    @patch('time.monotonic', return_value=100.)
    def test_time_margin_returns_even_without_finite_grid_route(self, _):
        n=mission();n.started=-30.;n.pose.return_value=(1.,0.,0.);n.return_distance=math.inf
        n.tick();self.assertEqual(n.state,'RETURN_HOME')

    def test_shutdown_preserves_done_report(self):
        n=mission();n.state='DONE';n.running=False;n.halt('keyboard_interrupt')
        self.assertEqual(n.state,'DONE');self.assertIsNone(n.failure);n.checkpoint.assert_not_called()

    def test_tolerances_cannot_create_rotation_dead_zone(self):
        c=yaml.safe_load((ROOT/'config/autonomy_nav2.yaml').read_text())['controller_server']['ros__parameters']
        self.assertLessEqual(c['FollowPath']['xy_goal_tolerance'],c['general_goal_checker']['xy_goal_tolerance'])
        # Symmetric sampling needs a zero-angular-velocity option.
        self.assertEqual(c['FollowPath']['vtheta_samples'] % 2,1)

    def test_camera_remaps_are_absolute_and_not_double_namespaced(self):
        source=(ROOT/'launch/apriltag_corrected.launch.py').read_text()
        pairs=[ast.literal_eval(n) for n in ast.walk(ast.parse(source)) if isinstance(n,ast.Tuple)
               and len(n.elts)==2 and all(isinstance(e,ast.Constant) for e in n.elts)]
        self.assertIn(('image_rect','/camera/color/image_raw'),pairs)
        self.assertIn(('camera_info','/camera/color/camera_info'),pairs)

    def test_blacklist_does_not_hide_entire_long_frontier(self):
        a=np.zeros((80,80),dtype=int);a[:,60:]=-1
        g=Grid(a.ravel(),80,80,.1);_,cost=g.routes((3.,4.))
        first=g.candidates((3.,4.),cost,[],0.)
        self.assertGreater(len(first),1)
        blocked=[(first[0]['x'],first[0]['y'],100.)]
        remaining=g.candidates((3.,4.),cost,blocked,1.)
        self.assertTrue(remaining)
        self.assertTrue(all(math.hypot(c['x']-blocked[0][0],c['y']-blocked[0][1])>=.6 for c in remaining))


class MapSeedTests(unittest.TestCase):
    def test_small_unknown_patch_under_robot_does_not_erase_routes(self):
        a=np.zeros((40,40),dtype=int);a[20,20]=-1
        g=Grid(a.ravel(),40,40,.05);_,cost=g.routes(g.world(20,20))
        self.assertGreater(np.isfinite(cost).sum(),100)
        self.assertFalse(np.isfinite(cost[20,20]))

    def test_seed_never_bridges_obstacle(self):
        a=np.zeros((40,40),dtype=int);a[20,20]=100
        g=Grid(a.ravel(),40,40,.05);_,cost=g.routes(g.world(20,20))
        self.assertFalse(np.isfinite(cost).any())

    def test_large_unknown_region_is_not_guessed_free(self):
        a=np.zeros((40,40),dtype=int);a[10:30,10:30]=-1
        g=Grid(a.ravel(),40,40,.05);_,cost=g.routes(g.world(20,20))
        self.assertFalse(np.isfinite(cost).any())

class TagTimestampTests(unittest.TestCase):
    def converter(self):
        from collections import deque
        class NoTransform(Exception):pass
        tree=ast.parse((ROOT/'asr_summer_school/tag_landmarks.py').read_text())
        cls=next(n for n in tree.body if isinstance(n,ast.ClassDef))
        cls.bases=[]
        env=dict(math=math,time=time,TransformException=NoTransform,
                 Time=NS(from_msg=lambda stamp:stamp),Landmark=NS,
                 LandmarkArray=lambda:NS(header=NS(),landmarks=[]))
        exec(compile(ast.Module(body=[cls],type_ignores=[]),'<actual tag conversion>','exec'),env)
        n=object.__new__(env['TagLandmarks']);n.base='base_link';n.pending=deque();n.dropped=0
        n.buffer=Mock();n.pub=Mock();n.get_logger=Mock()
        tag=NS(family='tag36h11',id=7,hamming=0,goodness=1.,decision_margin=40.)
        stamp=NS(sec=10,nanosec=1);msg=NS(header=NS(stamp=stamp),detections=[tag])
        return n,msg,NoTransform

    def test_tag_uses_image_timestamp_not_latest_transform(self):
        n,msg,_=self.converter()
        n.buffer.lookup_transform.return_value=NS(transform=NS(translation=NS(x=3.,y=4.)))
        n.receive(msg);n.flush()
        self.assertIs(n.buffer.lookup_transform.call_args.args[2],msg.header.stamp)
        output=n.pub.publish.call_args.args[0]
        self.assertEqual(output.landmarks[0].id,7);self.assertEqual(output.landmarks[0].range,5.)
        self.assertEqual(output.header.frame_id,'base_link')

    def test_late_transform_is_retried_without_duplicate_emission(self):
        n,msg,error=self.converter();n.buffer.lookup_transform.side_effect=error()
        n.receive(msg);n.flush();self.assertEqual(len(n.pending),1);n.pub.publish.assert_not_called()
        n.buffer.lookup_transform.side_effect=None
        n.buffer.lookup_transform.return_value=NS(transform=NS(translation=NS(x=1.,y=0.)))
        n.flush();n.flush();n.pub.publish.assert_called_once()

class HeadingTests(unittest.TestCase):
    def test_forward_motion_is_preferred_to_an_equivalent_turnaround(self):
        from asr_summer_school.mission_core import heading_penalty
        self.assertEqual(heading_penalty((0.,0.,0.),1.,0.),0.)
        self.assertGreater(heading_penalty((0.,0.,0.),-1.,0.),1.)

    def test_angle_wrap_does_not_create_a_full_turn(self):
        from asr_summer_school.mission_core import heading_penalty
        self.assertLess(heading_penalty((0.,0.,math.pi-.01),-1.,-.001),.01)

if __name__=='__main__':unittest.main()
