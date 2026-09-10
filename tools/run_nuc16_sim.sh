#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
export GAZEBO_MODEL_DATABASE_URI=""
export GAZEBO_MODEL_PATH=/usr/share/gazebo-11/models:${GAZEBO_MODEL_PATH:-}
source /opt/ros/tb3_ws/install/setup.bash
source /home/students/asr_sim_ws/install/setup.bash
export ROS_DOMAIN_ID=116 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export TURTLEBOT3_MODEL=burger QT_QPA_PLATFORM=xcb
while IFS='=' read -r key value; do
 case "$key" in DISPLAY|XAUTHORITY|XDG_RUNTIME_DIR|DBUS_SESSION_BUS_ADDRESS) export "$key=$value";; esac
done < <(systemctl --user show-environment)
case "${1:-launch}" in
 launch) exec timeout --signal=INT --kill-after=15s 600s ros2 launch asr_summer_school simulation_autonomy.launch.py duration:=150.0;;
 diagnose) exec python3 /home/students/asr_sim_ws/src/challenge/tools/diagnose_live.py "${2:-10}";;
 status) exec timeout 5 ros2 topic echo /mission/status --once;;
 arm) exec timeout 5 ros2 service call /safety/enable std_srvs/srv/SetBool '{data: true}';;
 start) exec timeout 5 ros2 service call /mission/start std_srvs/srv/Trigger '{}';;
 stop) exec timeout 5 ros2 service call /mission/stop std_srvs/srv/Trigger '{}';;
esac
