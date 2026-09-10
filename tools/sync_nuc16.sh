#!/bin/bash
# Sincronizza il pacchetto su nuc16, ricompila solo asr_summer_school e lancia i test ROS.
# NON avvia mai il bringup hardware (autonomy.launch.py sensors:=true).
set -eo pipefail

HOST="${NUC16_HOST:-students@192.168.10.116}"
WS="/home/students/asr_sim_ws"
DEST="${WS}/src/challenge"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

echo "== rsync verso ${HOST}:${DEST} =="
rsync -az --delete \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '._*' --exclude '.DS_Store' \
  --exclude '.git' --exclude 'build' --exclude 'install' --exclude 'log' \
  "${HERE}/asr_summer_school/" "${HOST}:${DEST}/asr_summer_school/"
rsync -az \
  --exclude '__pycache__' --exclude '*.pyc' --exclude '._*' --exclude '.DS_Store' \
  "${HERE}/tools/" "${HOST}:${DEST}/tools/"

echo "== aggiorno run_sim.sh =="
rsync -az "${HERE}/tools/run_nuc16_sim.sh" "${HOST}:${WS}/run_sim.sh"
ssh "${HOST}" "chmod +x ${WS}/run_sim.sh"

echo "== build (solo asr_summer_school) =="
ssh "${HOST}" "bash -lc 'source /opt/ros/humble/setup.bash && cd ${WS} && \
  colcon build --symlink-install --parallel-workers 2 --packages-select asr_summer_school'"

echo "== test ROS/action =="
ssh "${HOST}" "bash -lc 'source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash && \
  export ROS_DOMAIN_ID=116 ROS_LOCALHOST_ONLY=1 && cd ${WS} && \
  colcon test --packages-select asr_summer_school && \
  colcon test-result --verbose --all'"
