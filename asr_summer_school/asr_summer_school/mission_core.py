"""Geometry, frontier policy and exports; no ROS side effects."""
import heapq
import json
import math
import os
from pathlib import Path
import statistics
import numpy as np
from scipy import ndimage


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def atomic_json(path, value):
    path = Path(path)
    tmp = path.with_suffix(path.suffix + '.tmp')
    with tmp.open('w') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(tmp, path)


def heading_penalty(robot, x, y):
    """Equivalent travel distance lost turning; do not change collision checks."""
    if len(robot)<3:return 0.
    angle=math.atan2(y-robot[1],x-robot[0])-robot[2]
    return .18/.45*abs(math.atan2(math.sin(angle),math.cos(angle)))


class Grid:
    def __init__(self, data, width, height, resolution, origin=(0., 0., 0.)):
        if width <= 0 or height <= 0 or len(data) != width*height:
            raise ValueError('Malformed occupancy grid')
        if not math.isfinite(resolution) or resolution <= 0 or not all(map(math.isfinite, origin)):
            raise ValueError('Invalid map geometry')
        self.a = np.asarray(data, dtype=np.int16).reshape(height, width)
        self.res = resolution
        self.origin = origin
        self.h, self.w = height, width

    def cell(self, x, y):
        ox, oy, angle = self.origin
        dx, dy = x-ox, y-oy
        return (math.floor((math.cos(angle)*dx+math.sin(angle)*dy)/self.res),
                math.floor((-math.sin(angle)*dx+math.cos(angle)*dy)/self.res))

    def world(self, x, y):
        ox, oy, angle = self.origin
        xx, yy = (x+.5)*self.res, (y+.5)*self.res
        return (ox+math.cos(angle)*xx-math.sin(angle)*yy,
                oy+math.sin(angle)*xx+math.cos(angle)*yy)

    def routes(self, robot, clearance=.23, seed_radius=.35):
        free = (self.a >= 0) & (self.a <= 20)
        # Padding treats map edges as blocked, not infinite free space.
        # Unknown cells are not route centers, but do not invent obstacles around
        # every unsampled laser ray. Nav2 validates the footprint on its costmap.
        obstacle_free = (self.a < 0) | free
        distance = ndimage.distance_transform_edt(np.pad(obstacle_free, 1))[1:-1, 1:-1]*self.res
        safe = free & (distance >= clearance)
        x, y = self.cell(*robot[:2])
        costs = np.full(self.a.shape, np.inf)
        if not (0 <= x < self.w and 0 <= y < self.h):
            return safe, costs
        seed_distance = 0.
        if not safe[y, x]:
            # The robot is already standing here: refusing to seed destroys the
            # estimate without preventing any collision, and a Burger driving a
            # mapped corridor is routinely closer to a wall than `clearance`.
            # Bridge to the nearest safe cell instead, but never across an
            # occupied cell and never through less clearance than the robot
            # already has.  Every expanded cell still requires full clearance.
            floor = min(clearance, float(distance[y, x]))
            if self.a[y,x] >= 65 or floor <= 0.:
                return safe, costs
            # A known-but-tight cell may bridge up to seed_radius; an unmapped
            # patch under the robot stays limited to a small one.
            limit = seed_radius if self.a[y,x] >= 0 else .15
            candidates=np.argwhere(safe)
            if not len(candidates):return safe,costs
            delta=candidates-np.array([y,x])
            distances=np.linalg.norm(delta,axis=1)*self.res
            found=False
            for idx in np.argsort(distances):
                if distances[idx]>limit:break
                ny,nx=map(int,candidates[idx])
                steps=max(abs(nx-x),abs(ny-y))*4+1
                xx=np.rint(np.linspace(x,nx,steps)).astype(int)
                yy=np.rint(np.linspace(y,ny,steps)).astype(int)
                if np.all(obstacle_free[yy,xx] & (distance[yy,xx]>=floor)):
                    seed_distance=float(distances[idx]);x,y=nx,ny;found=True;break
            if not found:return safe,costs
        costs[y, x] = seed_distance
        queue = [(seed_distance, x, y)]
        while queue:
            d, xx, yy = heapq.heappop(queue)
            if d > costs[yy, xx]:
                continue
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1),(1,1),(1,-1),(-1,1),(-1,-1)):
                nx, ny = xx+dx, yy+dy
                if not (0 <= nx < self.w and 0 <= ny < self.h) or not safe[ny,nx]:
                    continue
                if dx and dy and (not safe[yy,nx] or not safe[ny,xx]):
                    continue
                nd = d + math.hypot(dx,dy)*self.res
                if nd < costs[ny,nx]:
                    costs[ny,nx] = nd
                    heapq.heappush(queue, (nd,nx,ny))
        return safe, costs

    def candidates(self, robot, costs, blacklist, now, min_size=5):
        free = (self.a >= 0) & (self.a <= 20)
        boundary = free & ndimage.binary_dilation(self.a < 0)
        labels, count = ndimage.label(boundary, structure=np.ones((3,3)))
        reachable = np.isfinite(costs)
        choices = []
        for idx in range(1, count+1):
            cluster = labels == idx
            size = int(cluster.sum())
            if size < min_size:
                continue
            near = ndimage.distance_transform_edt(~cluster)*self.res
            valid = reachable & (near <= .8) & (costs >= .45)
            if not valid.any():
                continue
            ys, xs = np.where(valid)
            # Stand back from unknown while preferring low travel cost.
            ranking = costs[ys,xs] + 3.*np.abs(near[ys,xs]-.30)
            order = np.argsort(ranking)
            for j in order:
                x,y = self.world(int(xs[j]),int(ys[j]))
                if any(until > now and math.hypot(x-bx,y-by) < .6 for bx,by,until in blacklist):
                    continue
                if any(math.hypot(x-c["x"], y-c["y"]) < .6 for c in choices):
                    continue
                d = float(costs[ys[j],xs[j]])
                # Final heading along travel; this is not a remedy for
                # controller/goal-checker tolerance mismatches.
                choices.append({'x':x,'y':y,'yaw':math.atan2(y-robot[1],x-robot[0]),'distance':d,
                                'gain':size*self.res,'score':size*self.res/(1+d+heading_penalty(robot,x,y))})
                if len(choices) >= 40:
                    return sorted(choices,key=lambda item:item["score"],reverse=True)
        return sorted(choices,key=lambda item:item['score'],reverse=True)

    def patrol_candidates(self, robot, costs, blacklist, now, max_radius=5.0):
        """Return safe, reachable patrol points when a frontier is unavailable.

        A frontier is a useful preference, not a prerequisite for motion: after
        SLAM has marked an open area as known, the frontier set can be empty even
        though there are still safe cells to visit.  This fallback keeps the
        mission exploring known free space while retaining the same route and
        blacklist checks.
        """
        safe = (self.a >= 0) & (self.a <= 20)
        reachable = np.isfinite(costs) & safe
        ys, xs = np.where(reachable)
        if not len(xs):
            return []
        rx, ry = robot[:2]
        choices = []
        for y, x in zip(ys[::4], xs[::4]):
            wx, wy = self.world(int(x), int(y))
            distance = math.hypot(wx-rx, wy-ry)
            if distance < .6 or distance > max_radius:
                continue
            if any(until > now and math.hypot(wx-bx, wy-by) < .6
                   for bx, by, until in blacklist):
                continue
            choices.append({'x': wx, 'y': wy, 'yaw': math.atan2(wy-ry, wx-rx),
                            'distance': float(costs[y, x]), 'gain': 0.,
                            # Prefer a short, reliable patrol step; later
                            # iterations will expand coverage without sending
                            # one long goal into an incompletely mapped area.
                            'score': -abs(distance-1.2)-heading_penalty(robot,wx,wy)})
        return sorted(choices, key=lambda item:item['score'], reverse=True)[:20]

    def distance_to(self, costs, point, tolerance=0.):
        """Route cost to `point`, accepting any route end within `tolerance`.

        One re-marked or momentarily unknown cell exactly under home must not
        read as "unreachable" while a valid route ends 5 cm away: Nav2 accepts
        the goal inside its own tolerance, so the return budget uses the same
        rule.  tolerance=0 keeps the exact-cell behaviour used elsewhere.
        """
        x,y = self.cell(*point[:2])
        radius = int(max(0.,tolerance)/self.res)
        x0,x1 = max(0,x-radius),min(self.w,x+radius+1)
        y0,y1 = max(0,y-radius),min(self.h,y+radius+1)
        if x0>=x1 or y0>=y1:
            return math.inf
        window = np.asarray(costs)[y0:y1,x0:x1]
        if radius:
            jj,ii = np.ogrid[y0-y:y1-y,x0-x:x1-x]
            window = np.where(np.hypot(ii*self.res,jj*self.res)<=tolerance,window,np.inf)
        return float(window.min())

    def route_report(self, costs, robot, home, tolerance=0.):
        """Why a home route is missing: robot seed, home cell, or connectivity."""
        rx,ry = self.cell(*robot[:2]);hx,hy = self.cell(*home[:2])
        inside = lambda x,y: 0<=x<self.w and 0<=y<self.h
        # Events are written with allow_nan=False; report infinities as null.
        finite = lambda value: value if math.isfinite(value) else None
        reachable = int(np.isfinite(costs).sum())
        return {'reachable_cells':reachable,
                'robot_cell':[rx,ry],'home_cell':[hx,hy],
                'robot_in_map':inside(rx,ry),'home_in_map':inside(hx,hy),
                'robot_occupancy':int(self.a[ry,rx]) if inside(rx,ry) else None,
                'home_occupancy':int(self.a[hy,hx]) if inside(hx,hy) else None,
                'home_cost_exact':finite(self.distance_to(costs,home)),
                'home_cost_tolerant':finite(self.distance_to(costs,home,tolerance)),
                # No reachable cell at all means the robot could not be seeded;
                # otherwise the map around home is what broke the route.
                'cause':'robot_seed' if not reachable else 'home_cell_or_connectivity'}


def reserve_seconds(distance, speed=.09, margin=25.):
    if speed <= 0 or not math.isfinite(speed) or not math.isfinite(distance) or distance < 0:
        return math.inf
    return distance/speed + margin


class TagStore:
    def __init__(self):
        self.samples = {}

    def add(self, tag_id, x, y, stamp):
        if not all(map(math.isfinite,(x,y,stamp))):
            return False
        points = self.samples.setdefault(int(tag_id),[])
        if points and stamp <= points[-1][2]:
            return False
        if len(points)>=3:
            mx,my = statistics.median(p[0] for p in points),statistics.median(p[1] for p in points)
            if math.hypot(x-mx,y-my) > .5:
                return False
        points.append((x,y,stamp))
        del points[:-100]
        return True

    def export(self):
        return [{'id':idx,'frame_id':'map','x':statistics.median(p[0] for p in points),
                 'y':statistics.median(p[1] for p in points),'observations':len(points),
                 'confirmed':len(points)>=3,'last_seen':points[-1][2]}
                for idx,points in sorted(self.samples.items()) if points]


def limited_command(linear, angular, enabled, permit_age, scan_age, command_age, nearest,
                    max_linear=.18, max_angular=.45):
    if (not enabled or permit_age>.7 or scan_age>.7 or command_age>.5
            or nearest<.20 or not all(map(math.isfinite,(linear,angular)))):
        return 0.,0.
    # No backwards blind recovery in the first supervised profile.
    return min(max(linear,0.),max_linear),min(max(angular,-max_angular),max_angular)
