import math
import tempfile
from pathlib import Path
import unittest
import numpy as np
from asr_summer_school.mission_core import Grid, TagStore, reserve_seconds, limited_command, atomic_json


class CoreTests(unittest.TestCase):
    def test_grid_rejects_corrupt_data(self):
        for data,w,h,r in [([],2,2,.1),([0],1,1,0),([0],1,1,float('nan'))]:
            with self.assertRaises(ValueError):Grid(data,w,h,r)

    def test_rotated_origin_roundtrip(self):
        g=Grid([0]*100,10,10,.1,(2.,-1.,math.pi/2))
        for x,y in [(0,0),(2,5),(9,9)]:self.assertEqual(g.cell(*g.world(x,y)),(x,y))

    def test_negative_edge_is_outside(self):
        self.assertEqual(Grid([0]*100,10,10,.1).cell(-.001,.05),(-1,0))

    def test_unknown_has_no_routes(self):
        g=Grid([-1]*100,10,10,.1)
        _,cost=g.routes((.5,.5));self.assertFalse(np.isfinite(cost).any())

    def test_wall_disconnects_regions(self):
        a=np.zeros((30,30),dtype=int);a[:,15]=100
        g=Grid(a.ravel(),30,30,.1);_,cost=g.routes((.7,1.5))
        self.assertFalse(np.isfinite(cost[:,16:]).any())

    def test_frontiers_stand_back_and_stay_reachable(self):
        a=np.zeros((40,40),dtype=int);a[:,30:]=-1
        g=Grid(a.ravel(),40,40,.1);safe,cost=g.routes((1.,2.))
        choices=g.candidates((1.,2.),cost,[],0.)
        self.assertTrue(choices)
        for c in choices:
            x,y=g.cell(c['x'],c['y']);self.assertTrue(safe[y,x]);self.assertTrue(math.isfinite(cost[y,x]))
            self.assertLess(c['x'],2.9)

    def test_blocked_robot_has_no_routes(self):
        a=np.zeros((30,30),dtype=int);a[15,15]=100
        g=Grid(a.ravel(),30,30,.1);_,cost=g.routes((1.55,1.55))
        self.assertFalse(np.isfinite(cost).any())

    def test_robot_closer_to_a_wall_than_clearance_keeps_its_routes(self):
        # A Burger driving a mapped corridor is routinely within 23 cm of a
        # wall; that must not erase every route, home included.
        a=np.zeros((120,120),dtype=int);a[:,0]=100
        g=Grid(a.ravel(),120,120,.05)
        home=g.world(60,60)
        for cells in (3,4,5):
            _,cost=g.routes(g.world(cells,60))
            self.assertGreater(np.isfinite(cost).sum(),1000,'%d celle dal muro'%cells)
            self.assertTrue(math.isfinite(g.distance_to(cost,home)))

    def test_seed_never_bridges_through_less_clearance_than_the_robot_has(self):
        a=np.zeros((60,60),dtype=int);a[:,0]=100;a[30,1:6]=100
        g=Grid(a.ravel(),60,60,.05);safe,cost=g.routes(g.world(3,30))
        for y,x in np.argwhere(np.isfinite(cost)):
            self.assertTrue(safe[y,x] or (y,x)==(30,3))

    def test_blocked_cell_under_the_robot_still_has_no_routes(self):
        a=np.zeros((60,60),dtype=int);a[30,30]=100
        g=Grid(a.ravel(),60,60,.05);_,cost=g.routes(g.world(30,30))
        self.assertFalse(np.isfinite(cost).any())

    def test_home_cell_turned_unknown_is_reached_within_tolerance(self):
        # A SLAM update can hand back home as unknown while a valid route ends
        # a few centimetres away; the exact cell alone reads as inf.
        a=np.zeros((120,120),dtype=int);a[59:62,59:62]=-1
        g=Grid(a.ravel(),120,120,.05);_,cost=g.routes(g.world(30,60))
        home=g.world(60,60)
        self.assertFalse(math.isfinite(g.distance_to(cost,home)))
        self.assertTrue(math.isfinite(g.distance_to(cost,home,.25)))

    def test_tolerance_does_not_reach_into_an_inflated_obstacle(self):
        # An occupied blob on home stays unreachable: the tolerance accepts a
        # nearby route end, it never claims a route inside the clearance shadow.
        a=np.zeros((120,120),dtype=int);a[59:62,59:62]=100
        g=Grid(a.ravel(),120,120,.05);_,cost=g.routes(g.world(30,60))
        self.assertFalse(math.isfinite(g.distance_to(cost,g.world(60,60),.25)))

    def test_tolerance_does_not_invent_a_route_across_a_wall(self):
        a=np.zeros((60,60),dtype=int);a[:,30]=100
        g=Grid(a.ravel(),60,60,.05);_,cost=g.routes(g.world(10,30))
        self.assertFalse(math.isfinite(g.distance_to(cost,g.world(45,30),.25)))

    def test_route_report_names_the_cause_and_stays_json_safe(self):
        import json
        a=np.zeros((60,60),dtype=int);a[30,30]=100
        g=Grid(a.ravel(),60,60,.05);_,cost=g.routes(g.world(30,30))
        report=g.route_report(cost,g.world(30,30),g.world(20,20),.25)
        self.assertEqual(report['cause'],'robot_seed')
        self.assertIsNone(report['home_cost_exact'])
        json.dumps(report,allow_nan=False)
        b=np.zeros((60,60),dtype=int);b[20,20]=100
        gb=Grid(b.ravel(),60,60,.05);_,cost=gb.routes(gb.world(30,30))
        self.assertEqual(gb.route_report(cost,gb.world(30,30),gb.world(20,20),.25)['cause'],
                         'home_cell_or_connectivity')

    def test_no_unknown_no_frontiers(self):
        g=Grid([0]*900,30,30,.1);_,cost=g.routes((1.5,1.5))
        self.assertEqual(g.candidates((1.5,1.5),cost,[],0),[])

    def test_patrol_fallback_in_known_open_space(self):
        g=Grid([0]*2500,50,50,.1);_,cost=g.routes((2.5,2.5))
        choices=g.patrol_candidates((2.5,2.5),cost,[],0.,5.)
        self.assertTrue(choices)
        self.assertTrue(all(.6 <= c['distance'] <= 5. for c in choices))

    def test_reserve_fails_closed(self):
        self.assertEqual(reserve_seconds(math.inf),math.inf)
        self.assertEqual(reserve_seconds(-1),math.inf)
        self.assertEqual(reserve_seconds(3,.1,25),55.)

    def test_tag_dedup_and_outlier(self):
        tags=TagStore()
        for t in range(3):self.assertTrue(tags.add(7,1.+t*.01,2.,float(t)))
        self.assertFalse(tags.add(7,10.,10.,4.))
        self.assertFalse(tags.add(7,1.,2.,1.))
        result=tags.export();self.assertEqual(len(result),1);self.assertTrue(result[0]['confirmed'])
        self.assertAlmostEqual(result[0]['x'],1.01)

    def test_nonfinite_tag_rejected(self):
        self.assertFalse(TagStore().add(1,math.nan,2.,1.))

    def test_gate_disabled_and_expired(self):
        base=[.1,.2,True,0.,0.,0.,1.]
        for index,value in [(2,False),(3,1.),(4,1.),(5,1.),(6,.1),(0,math.nan)]:
            args=base.copy();args[index]=value;self.assertEqual(limited_command(*args),(0.,0.))

    def test_gate_caps_velocity(self):
        self.assertEqual(limited_command(1.,2.,True,0.,0.,0.,1.),(.18,.45))
        self.assertEqual(limited_command(-1.,-2.,True,0.,0.,0.,1.),(0.,-.45))

    def test_atomic_export(self):
        import json
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'tags.json';atomic_json(p,{'tags':[1]});atomic_json(p,{'tags':[2]})
            self.assertEqual(json.loads(p.read_text()),{'tags':[2]})
            self.assertFalse(p.with_suffix('.json.tmp').exists())

if __name__=='__main__':unittest.main()
