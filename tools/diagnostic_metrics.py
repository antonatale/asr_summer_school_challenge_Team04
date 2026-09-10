"""Window-scoped measurements; no ROS or command publishing."""
import math


class WindowMetrics:
    def __init__(self):
        self.first = self.previous = self.last = None
        self.path = self.moving = self.stopped = self.observed = 0.
        self.gaps = self.invalid = 0
        self.scan_min = None
        self.valid_scans = self.empty_scans = self.close_scans = 0

    def odom(self, stamp, x, y, linear, angular):
        if not all(math.isfinite(v) for v in (stamp, x, y, linear, angular)):
            self.invalid += 1
            self.previous = None
            return
        point = (x, y)
        if self.first is None:
            self.first = point
        self.last = point
        if self.previous is not None:
            old_stamp, old_point, was_moving = self.previous
            dt = stamp-old_stamp
            if 0 < dt <= 1.:
                self.path += math.dist(old_point, point)
                self.observed += dt
                if was_moving:
                    self.moving += dt
                else:
                    self.stopped += dt
            else:
                # Clock resets and missing odometry must not become motion/time.
                self.gaps += 1
        self.previous = (stamp, point, abs(linear) > .01 or abs(angular) > .02)

    def scan(self, ranges, minimum, maximum):
        valid = [r for r in ranges if math.isfinite(r) and minimum <= r <= maximum]
        if not valid:
            self.empty_scans += 1
            return None
        nearest = min(valid)
        self.valid_scans += 1
        self.close_scans += int(nearest < .20)
        self.scan_min = nearest if self.scan_min is None else min(self.scan_min, nearest)
        return nearest

    def report(self):
        return dict(measurement_scope='observation_window_only',
                    odom_displacement_m=math.dist(self.first, self.last) if self.first is not None else None,
                    odom_path_m=self.path if self.observed > 0 else None,
                    odom_observed_sim_seconds=self.observed,
                    moving_sim_seconds=self.moving, stopped_sim_seconds=self.stopped,
                    odom_gap_or_clock_reset_count=self.gaps, invalid_odom_samples=self.invalid,
                    minimum_laser_range_m=self.scan_min, valid_scan_samples=self.valid_scans,
                    empty_scan_samples=self.empty_scans, scan_under_20cm=self.close_scans,
                    collision_count=None)


def graph_warnings(command_publishers, duplicate_nodes):
    warnings = []
    if len(command_publishers) > 1:
        warnings.append('Multiple /cmd_vel publishers: verify ownership before starting a mission')
    if duplicate_nodes:
        warnings.append('Duplicate ROS node names: possible leftover or parallel launch')
    return warnings
