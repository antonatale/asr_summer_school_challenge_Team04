# Autonomous mission — `feature/autonomous-mission`

A first complete version of the challenge mission, developed and tested in
simulation. It does not touch `main` and it does not modify the tutors'
submodules; the fixes that were needed live in our own package instead.

Nothing here has been run autonomously on the physical robot yet. Treat it as a
base to read, try and take pieces from, not as a validated solution.

## What it does

The robot starts idle and waits for an explicit start call. Once started it maps
with SLAM, picks frontiers and explores on its own, reads AprilTags from the
camera and records each ID once in map coordinates, and goes back to the starting
point in time to be there before the deadline. Map, tag list and a JSON event log
are written to `~/mission_runs/<timestamp>/`.

Four nodes:

- `mission_orchestrator` — start service, mission clock, frontier choice, return
  budget, checkpoints, event log.
- `mission_core` — the ROS-free half, so it can be unit tested: grid geometry
  (rotated origins included), route search with obstacle clearance, frontier and
  patrol candidates, return-time budget, and a tag store that keeps one median
  position per ID and rejects outliers.
- `motion_guard` — a fail-closed gate on `/cmd_vel`. Nav2 is remapped to
  `/autonomy/cmd_vel` and nothing reaches the wheels unless the guard is armed,
  the mission heartbeat is alive, the laser scan is fresh and nothing is closer
  than 20 cm. Speed is capped at 0.18 m/s and 0.45 rad/s, and reversing is not
  allowed. It comes up disarmed on every launch.
- `tag_landmarks` — converts detections to range and bearing using the TF at the
  image timestamp rather than the latest one, waiting up to 0.5 s for a late
  transform instead of recording a wrong position. No tag position is ever read
  from the world file.

## Simulation

On nuc16, in one terminal:

    bash ~/asr_sim_ws/run_sim.sh launch

and in a second one:

    bash ~/asr_sim_ws/run_sim.sh status
    bash ~/asr_sim_ws/run_sim.sh arm
    bash ~/asr_sim_ws/run_sim.sh start
    bash ~/asr_sim_ws/run_sim.sh diagnose 30

`arm` only enables the virtual gate, there are no motors involved. Pass a world
path to `launch` to use a different one; `worlds/hard_maze_apriltag.world` is the
harder arena with obstacles and eleven tags. `run_sim.sh clean` clears a previous
launch: it terminates only the process group that launch recorded, and refuses if
anything else is still running, since on nuc16 we all share the `students`
account and a name match cannot tell our processes from a teammate's.

## Tests

    bash tools/test_offline.sh

Runs without ROS: 22 core tests, 30 regressions that execute the real orchestrator
methods with stubbed transport, 7 diagnostic tests, and a scripted return-home
scenario over an evolving map. On nuc16, `colcon test` adds the ROS ones,
including `guard_transport`, which drives the real `/cmd_vel` gate over ROS
transport and is pinned to the isolated domain 87 so it cannot reach the robot's
domain 8 or the simulation's 116.

## What has actually been checked

Two full simulation runs on nuc16 finished with the robot back at the start:
in the simple arena, 131.8 s of 150, all 16 Nav2 goals reached, tag 9 localised
about 2 cm from its true position; in the harder maze, 134.9 s, 12 goals, tag 4
about 2 cm out. Both prove the chain image → detection → TF → map end to end.
They say nothing about the real course, real tags or real timing, and both ran
with the old distance bound still in place, so the coverage figures are not
representative any more.

Two changes have not been exercised on ROS yet, only offline: the
`guard_transport` registration and the process-group cleanup. Worth a `colcon
test` and one throwaway launch at the start of the next lab session.

## Known limits

The tag size, 0.16 m, is inherited from the course configuration and has not been
measured on a real tag; measure the coded border before judging accuracy. The
return reserve uses a fixed 25 s margin and an assumed 0.09 m/s, which should be
retuned from measured return times. RPP is used in simulation only, the robot
launch still uses DWB, and the two have not been compared with numbers yet.

## Safety

`autonomy.launch.py sensors:=true` is the physical-robot path and must not be
launched on nuc16. Do not publish on `/cmd_vel` directly and do not bypass
`motion_guard`. Before any physical test: physical stop verified, area clear, one
person next to the robot. If the robot does not move and nothing is logged as an
error, the guard is almost certainly disarmed — that is by design, and
`/safety/status` says which condition is blocking.
