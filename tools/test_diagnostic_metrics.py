import math
import unittest
from diagnostic_metrics import WindowMetrics, graph_warnings


class MetricsTests(unittest.TestCase):
    def test_closed_route_preserves_distance(self):
        m = WindowMetrics()
        for stamp, x in [(0, 0), (.5, .1), (1, 0)]:
            m.odom(stamp, x, 0, .2, 0)
        self.assertAlmostEqual(m.report()['odom_path_m'], .2)
        self.assertEqual(m.report()['odom_displacement_m'], 0)

    def test_rotation_counts_as_moving(self):
        m = WindowMetrics()
        m.odom(0, 0, 0, 0, .4)
        m.odom(.5, 0, 0, 0, 0)
        m.odom(1, 0, 0, 0, 0)
        self.assertEqual(m.moving, .5)
        self.assertEqual(m.stopped, .5)

    def test_gap_and_reset_do_not_invent_distance(self):
        m = WindowMetrics()
        for stamp, x in [(0, 0), (10, 5), (1, 0), (1.5, .1)]:
            m.odom(stamp, x, 0, .2, 0)
        self.assertEqual(m.gaps, 2)
        self.assertAlmostEqual(m.path, .1)
        self.assertEqual(m.observed, .5)

    def test_invalid_odometry_breaks_segment(self):
        m = WindowMetrics()
        for stamp, x in [(0, 0), (.2, math.nan), (.4, .1)]:
            m.odom(stamp, x, 0, 0, 0)
        self.assertEqual(m.invalid, 1)
        self.assertIsNone(m.report()['odom_path_m'])

    def test_minimum_persists_but_is_not_collision_count(self):
        m = WindowMetrics()
        m.scan([math.nan, math.inf, .15], .1, 10)
        m.scan([2], .1, 10)
        self.assertEqual(m.report()['minimum_laser_range_m'], .15)
        self.assertEqual(m.close_scans, 1)
        self.assertIsNone(m.report()['collision_count'])

    def test_missing_scan_is_not_clear_space(self):
        m = WindowMetrics()
        self.assertIsNone(m.scan([math.inf, math.nan, .01], .1, 10))
        self.assertIsNone(m.report()['minimum_laser_range_m'])
        self.assertEqual(m.empty_scans, 1)

    def test_duplicate_publishers_are_reported(self):
        self.assertFalse(graph_warnings(['motion_guard'], []))
        self.assertEqual(len(graph_warnings(['motion_guard', 'motion_guard'], ['motion_guard'])), 2)


if __name__ == '__main__':
    unittest.main()
