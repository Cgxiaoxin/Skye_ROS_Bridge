#!/usr/bin/env bash
# Usage: leader_arm_gate.sh <left|right> <off|on>
# Runs inside existing Marvin container via docker exec (no new container).
set -euo pipefail

SIDE="${1:?left|right}"
ACTION="${2:?off|on}"
CONTAINER="${MARVIN_CONTAINER_NAME:-skye_marvin_m6}"

case "$SIDE" in left|right) ;; *)
  echo "bad side" >&2
  exit 2
  ;;
esac
case "$ACTION" in off|on) ;; *)
  echo "bad action" >&2
  exit 2
  ;;
esac

docker exec -e "SIDE=${SIDE}" -e "ACTION=${ACTION}" "${CONTAINER}" bash -lc '
set -euo pipefail
source /marvin_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml
export ROS_LOCALHOST_ONLY=0
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/opt/openrobots/lib/python3.10/site-packages"

NODE="factr_teleop_${SIDE}"
CFG="/marvin_ws/configs/${ROBOT_PROFILE:-thor}/grav_comp_m6_${SIDE}.yaml"

if [[ "$ACTION" == "off" ]]; then
  pkill -f "$NODE" || true
  sleep 0.2
  python3 /scripts/disable_leader_dynamixel.py --side "$SIDE"
  exit 0
fi

if ros2 node list 2>/dev/null | grep -qx "/${NODE}"; then
  exit 0
fi

nohup ros2 run factr_teleop factr_teleop_robot_driver.py --ros-args \
  -r "__node:=${NODE}" \
  -p "config_file:=${CFG}" \
  -p debug_state_print:=true -p print_joint_states:=true \
  -p print_period:=0.5 -p cb_print_period:=1.0 \
  -r "/joint_control:=/gento/${SIDE}_joint_control" \
  -r "/joint_state:=/gento/${SIDE}_joint_states" \
  -r "/joint_move:=/${SIDE}_joint_move" \
  -r "/leader_arm/current_state:=/${SIDE}_leader_arm/current_state" \
  -r "/leader_arm/target_joint_state:=/${SIDE}_leader_arm/target_joint_state" \
  -r "/gripper/ctrl:=/${SIDE}_teleop_gripper/ctrl" \
  -r "/gripper/state:=/${SIDE}_gripper/state" \
  >"/tmp/${NODE}.log" 2>&1 &

deadline=$((SECONDS + 5))
until ros2 node list 2>/dev/null | grep -qx "/${NODE}"; do
  if (( SECONDS >= deadline )); then
    echo "failed to start /${NODE}; check /tmp/${NODE}.log in the Marvin container" >&2
    exit 1
  fi
  sleep 0.2
done
'
