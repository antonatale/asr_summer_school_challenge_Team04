#!/bin/bash
set -eo pipefail

# Processi che simulation_autonomy.launch.py puo' lasciare in giro.  Serve solo
# per RICONOSCERLI e riferirli all'operatore: la pulizia non uccide mai per nome,
# vedi cleanup_sim.
SIM_PROCS='gzserver|gzclient|spawn_entity.py|async_slam_toolbox_node|controller_server|planner_server|bt_navigator|behavior_server|smoother_server|velocity_smoother|waypoint_follower|lifecycle_manager|robot_state_publisher|apriltag_node|scan_to_scan_filter_chain|rviz2|motion_guard.py|mission_orchestrator.py|tag_landmarks.py|simulation_autonomy.launch'

# Quei nomi (robot_state_publisher, apriltag_node, motion_guard) li usa anche il
# bringup hardware: ucciderli sotto un robot vivo toglierebbe il guard mentre la
# base tiene ancora l'ultimo comando.  `autonomy.launch.py` e' ancorato per non
# combaciare con `simulation_autonomy.launch.py`.
HW_PROCS='(^|[/ ])autonomy\.launch\.py|bringup\.launch\.py|turtlebot3_ros|sensors:=true'

# Identita' del lancio: `launch` registra qui il proprio process group, e la
# pulizia termina SOLO quel gruppo.  Uccidere per nome su una macchina condivisa
# fa fuori il lancio di un compagno, che a `pkill -f` e' indistinguibile dal
# nostro: su nuc16 l'account `students` e' lo stesso per tutti.
PGID_FILE="${ASR_SIM_PGID_FILE:-${XDG_RUNTIME_DIR:-/tmp}/asr_sim_launch.pgid}"

own_pgid(){ ps -o pgid= -p "$$" 2>/dev/null | tr -d ' '; }

# Processi di simulazione di QUESTO utente, esclusi noi stessi.
foreign_sim(){ pgrep -u "$(id -u)" -f "$SIM_PROCS" 2>/dev/null | grep -v "^$$\$" || true; }

refuse_if_hardware(){
  if pgrep -f "$HW_PROCS" >/dev/null; then
    echo "RIFIUTO: e' attivo il percorso hardware, non la simulazione." >&2
    pgrep -af "$HW_PROCS" >&2
    echo "Fermare il bringup a mano prima di tornare alla simulazione." >&2
    return 1
  fi
  return 0
}

# Termina solo il process group registrato dal lancio precedente.
kill_recorded_group(){
  local pgid; pgid=$(cat "$PGID_FILE" 2>/dev/null || true)
  [ -n "$pgid" ] || return 0
  if ! kill -0 -- "-$pgid" 2>/dev/null; then rm -f "$PGID_FILE"; return 0; fi
  echo "chiudo il lancio precedente (process group $pgid)"
  kill -INT -- "-$pgid" 2>/dev/null || true
  for _ in $(seq 1 10); do kill -0 -- "-$pgid" 2>/dev/null || break; sleep 1; done
  kill -KILL -- "-$pgid" 2>/dev/null || true
  sleep 1
  rm -f "$PGID_FILE"
}

cleanup_sim(){
  refuse_if_hardware || return 1
  kill_recorded_group
  local leftovers; leftovers=$(foreign_sim)
  if [ -n "$leftovers" ]; then
    echo "ERRORE: restano processi di simulazione che questo script non ha avviato:" >&2
    ps -o pid=,pgid=,cmd= -p $(echo "$leftovers" | tr '\n' ' ') 2>/dev/null >&2 || true
    echo "Possono essere il lancio di un compagno sulla stessa macchina." >&2
    echo "Se sono sicuramente orfani tuoi: bash run_sim.sh clean --force-orphans" >&2
    return 1
  fi
  if ss -ltn 2>/dev/null | grep -q 11345; then
    echo "ERRORE: porta 11345 (gazebo master) ancora occupata" >&2
    return 1
  fi
  return 0
}

# Ultima spiaggia, da invocare a mano.  Uccide per nome, quindi su un account
# condiviso (su nuc16 sono tutti `students`) puo' colpire anche il lancio di un
# compagno: l'uid non li separa, solo il process group lo fa.  Per questo non e'
# mai automatica.  Serviva per gli orfani del 9/9, rimasti senza PGID_FILE.
force_orphans(){
  refuse_if_hardware || return 1
  echo "ATTENZIONE: chiusura per nome. Su account condiviso puo' colpire il" >&2
  echo "lancio di un compagno. Controlla l'elenco qui sotto." >&2
  local victims; victims=$(foreign_sim)
  [ -n "$victims" ] || { echo "nessun processo da chiudere"; return 0; }
  ps -o pid=,pgid=,cmd= -p $(echo "$victims" | tr '\n' ' ') 2>/dev/null >&2 || true
  pkill -INT -u "$(id -u)" -f "$SIM_PROCS" 2>/dev/null || true
  for _ in $(seq 1 10); do [ -n "$(foreign_sim)" ] || break; sleep 1; done
  pkill -KILL -u "$(id -u)" -f "$SIM_PROCS" 2>/dev/null || true
  sleep 1
  [ -z "$(foreign_sim)" ] || { echo "ERRORE: processi ancora vivi" >&2; return 1; }
  rm -f "$PGID_FILE"
  return 0
}

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
 clean)
  if [ "${2:-}" = "--force-orphans" ]; then force_orphans && echo "orfani chiusi"
  else cleanup_sim && echo "ambiente simulazione pulito"; fi;;
 launch)
  cleanup_sim
  own_pgid > "$PGID_FILE"          # identita' da terminare al prossimo lancio
  world_arg=""
  [ -n "${2:-}" ] && world_arg="world:=$2"
  exec timeout --signal=INT --kill-after=15s 600s ros2 launch asr_summer_school simulation_autonomy.launch.py duration:=150.0 $world_arg;;
 diagnose) exec python3 /home/students/asr_sim_ws/src/challenge/tools/diagnose_live.py "${2:-10}";;
 status) exec timeout 5 ros2 topic echo /mission/status --once;;
 arm) exec timeout 5 ros2 service call /safety/enable std_srvs/srv/SetBool '{data: true}';;
 start) exec timeout 5 ros2 service call /mission/start std_srvs/srv/Trigger '{}';;
 stop) exec timeout 5 ros2 service call /mission/stop std_srvs/srv/Trigger '{}';;
esac
