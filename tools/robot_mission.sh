#!/usr/bin/env bash
# Run on nuc08; no permanent shell or network configuration changes.
set -eo pipefail
source /opt/ros/humble/setup.bash
source /opt/ros/tb3_ws/install/setup.bash
source /home/students/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=8 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-02 CAMERA_MODEL=realsense
case "${1:-help}" in
  launch)
    exec timeout --signal=INT --kill-after=10s 420s ros2 launch asr_summer_school autonomy.launch.py sensors:=true "duration:=${2:-60.0}" "max_radius:=${3:-0.0}"
    ;;
  diagnose) exec python3 "$(dirname "${BASH_SOURCE[0]}")/diagnose_live.py" "${2:-15}" ;;
  status) exec timeout 5s ros2 topic echo /mission/status --once ;;
  snapshot) exec timeout 5s ros2 service call /mission/snapshot std_srvs/srv/Trigger '{}' ;;
  arm) exec timeout 5s ros2 service call /safety/enable std_srvs/srv/SetBool '{data: true}' ;;
  start) exec timeout 5s ros2 service call /mission/start std_srvs/srv/Trigger '{}' ;;
  stop)
    timeout 3s ros2 service call /safety/enable std_srvs/srv/SetBool '{data: false}' || true
    timeout 3s ros2 service call /mission/stop std_srvs/srv/Trigger '{}' || true
    ;;
  *) echo 'Usage: robot_mission.sh launch [seconds=60] [radius_m=0, 0 = no limit] | status | snapshot | arm | start | stop';;
esac
