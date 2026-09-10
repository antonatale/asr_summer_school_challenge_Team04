#!/usr/bin/env python3
"""Scenario offline della sola logica di ritorno, senza ROS e senza Gazebo.

Esegue la vera `Mission.tick()` su una mappa che evolve (robot che costeggia
una parete interna, home rimarcata ignota da un aggiornamento SLAM) e riporta
quando e perche' parte il ritorno.  Non sostituisce la prova in simulazione:
verifica solo che una perdita momentanea della rotta interna non anticipi il
ritorno e che la riserva di tempo resti attiva.

    python3 tools/scenario_return.py [--repo ALTRO_REPO]
"""
import argparse
import ast
import json
import math
import sys
import time
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import numpy as np

RES = .05
CELLS = 120


def load(root):
    for name in [m for m in list(sys.modules) if m.startswith('asr_summer_school')]:
        del sys.modules[name]
    sys.path.insert(0, str(root))
    from asr_summer_school.mission_core import Grid, TagStore, reserve_seconds
    tree = ast.parse((root/'asr_summer_school/mission_orchestrator.py').read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'Mission')
    cls.bases = []
    env = dict(time=time, math=math, json=json, Bool=NS, String=NS,
               reserve_seconds=reserve_seconds, GoalStatus=NS(STATUS_SUCCEEDED=4))
    exec(compile(ast.Module(body=[cls], type_ignores=[]), '<Mission>', 'exec'), env)
    sys.path.remove(str(root))
    return Grid, env['Mission'], TagStore


def arena():
    """Arena 6x6 m con bordo e parete interna, nello spirito di smoke_tags.world."""
    a = np.zeros((CELLS, CELLS), dtype=int)
    a[0, :] = a[-1, :] = a[:, 0] = a[:, -1] = 100
    a[40:80, 70] = 100
    return a


def run(root, flicker_home, label, duration=150., margin=25.):
    Grid, Mission, TagStore = load(root)
    a = arena()
    n = object.__new__(Mission)
    for k, v in dict(running=True, state='EXPLORE', enabled=True, duration=duration,
                     margin=margin, radius=5., battery=None, last_status=0.,
                     last_checkpoint=0., last_health=-100., last_plan=-100.,
                     returning=False, cancel_at=None, future=None, no_frontiers=0,
                     return_distance=math.inf, home_reached=False, failure=None,
                     pending_kind=None, goal=None, return_attempts=0, blacklist=[],
                     plan_queue=[], grid_cache=None, started=0.,
                     route_tolerance=.25, route_grace=6., detour=1.6,
                     route_ok=True, route_lost_at=None, health_grace=3., unhealthy_at=None,
                     map_at=0., scan_at=0., guard_at=0.).items():
        setattr(n, k, v)
    n.tags = TagStore()
    n.ready = Mock(return_value=True)
    n.permit = n.status = n.disarm = n.checkpoint = n.plan = Mock()
    clock = [0.]
    events = []
    n.event = lambda name, **data: events.append((clock[0], name, data))
    reference = Grid(a.ravel(), CELLS, CELLS, RES)
    home = reference.world(30, 60)+(0.,)
    wall_side = reference.world(67, 60)[0]      # 15 cm dalla parete interna

    def position(t):
        if t < 30: return (home[0]+t*.06, home[1], 0.)
        if t < 60: return (wall_side, home[1]+(t-30)*.02, 0.)
        return (wall_side, home[1]+.6, 0.)

    real_monotonic = time.monotonic
    time.monotonic = lambda: clock[0]
    t = 0.
    try:
        for step in range(int(2*(duration-10))):
            t = clock[0] = step*.5
            b = a.copy()
            if flicker_home and 70 <= t < 76:
                b[59:62, 29:32] = -1            # home torna ignota per 6 s
            n.grid = Grid(b.ravel(), CELLS, CELLS, RES)
            n.pose = Mock(return_value=position(t))
            n.home_pose = Mock(return_value=home)
            if step % 4 == 0:
                n.grid_cache = None
            n.tick()
            if not n.running or n.returning:
                break
    finally:
        time.monotonic = real_monotonic
    start = next((e for e in events if e[1] == 'return_started'), None)
    print('%-30s ritorno=%-26s t=%6.1fs  stato=%s' % (
        label, start[2]['reason'] if start else 'nessuno', start[0] if start else t, n.state))
    for stamp, name, data in events:
        if name in ('return_route_unavailable', 'return_route_lost', 'return_started'):
            print('    %6.1fs %s %s' % (stamp, name, {k: v for k, v in data.items()
                  if k in ('reason', 'cause', 'home_cost_tolerant', 'estimate')}))
    return start


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', default=str(Path(__file__).resolve().parents[1]/'asr_summer_school'))
    args = parser.parse_args()
    root = Path(args.repo)
    first = run(root, False, 'robot vicino alla parete')
    second = run(root, True, 'home ignota per 6 s')
    bad = [s for s in (first, second) if s and s[2]['reason'] == 'return_route_unavailable']
    print('ESITO:', 'ritorno anticipato dalla rotta non disponibile' if bad else 'nessun ritorno anticipato')
    sys.exit(1 if bad else 0)
