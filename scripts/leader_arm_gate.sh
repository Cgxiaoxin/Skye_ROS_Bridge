#!/usr/bin/env bash
# Usage: leader_arm_gate.sh <left|right> <off|on>
# Prefer docker exec into the running Marvin container. For off, if the
# container is gone, fall back to a one-shot image (same as session teardown).
set -euo pipefail

SIDE="${1:?left|right}"
ACTION="${2:?off|on}"
CONTAINER="${MARVIN_CONTAINER_NAME:-skye_marvin_m6}"
IMAGE="${MARVIN_IMAGE:-marvin-m6-ros2:e5a9d8fd}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MARVIN_WS="${MARVIN_WS:-${REPO_ROOT}/marvin_ws}"

case "$SIDE" in left|right) ;; *)
  echo "bad side: ${SIDE}" >&2
  exit 2
  ;;
esac
case "$ACTION" in off|on) ;; *)
  echo "bad action: ${ACTION}" >&2
  exit 2
  ;;
esac

# Host-mounted marvin_ws install is not on the image's default sys.path.
_DXL_PYTHONPATH="/marvin_ws/install/local/lib/python3.10/dist-packages"

_disable_oneshot() {
  echo "Marvin 容器 ${CONTAINER} 未运行，改用一次性镜像去使能 ${SIDE}…" >&2
  docker run --rm --privileged --network host \
    -v /dev:/dev \
    -v "${MARVIN_WS}:/marvin_ws" \
    -v "${SCRIPT_DIR}:/scripts:ro" \
    -e "MARVIN_WS=/marvin_ws" \
    -e "PYTHONPATH=${_DXL_PYTHONPATH}" \
    -w /marvin_ws \
    "${IMAGE}" \
    python3 /scripts/disable_leader_dynamixel.py --side "${SIDE}"
}

if ! docker inspect "${CONTAINER}" >/dev/null 2>&1; then
  if [[ "$ACTION" == "off" ]]; then
    _disable_oneshot
    exit 0
  fi
  echo "Marvin 容器 ${CONTAINER} 未运行，无法重新使能小臂。请确认会话已启动且 marvin 步骤正常。" >&2
  exit 1
fi

# off 不需要 source ROS；显式 PYTHONPATH 即可找到 dynamixel_sdk。
if [[ "$ACTION" == "off" ]]; then
  if ! docker exec \
    -e "SIDE=${SIDE}" \
    -e "PYTHONPATH=${_DXL_PYTHONPATH}" \
    -e "MARVIN_WS=/marvin_ws" \
    "${CONTAINER}" bash -lc '
set -euo pipefail
NODE="factr_teleop_${SIDE}"
pkill -f "$NODE" || true
# 等串口从 factr 释放，避免 openPort 失败
sleep 1.0
python3 /scripts/disable_leader_dynamixel.py --side "$SIDE"
'; then
    echo "容器内去使能失败，改用一次性镜像…" >&2
    _disable_oneshot
  fi
  exit 0
fi

docker exec -e "SIDE=${SIDE}" -e "ACTION=on" "${CONTAINER}" bash -lc '
set -euo pipefail
source /marvin_ws/install/setup.bash
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml
export ROS_LOCALHOST_ONLY=0
export PYTHONPATH="${PYTHONPATH:+${PYTHONPATH}:}/opt/openrobots/lib/python3.10/site-packages"

NODE="factr_teleop_${SIDE}"
CFG="/marvin_ws/configs/${ROBOT_PROFILE:-thor}/grav_comp_m6_${SIDE}.yaml"

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
