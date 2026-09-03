#!/usr/bin/env bash
# Enter Marvin M6 FACTR docker with marvin_ws mounted at /marvin_ws.
#
# Usage:
#   ./scripts/run_marvin_m6_impedance.sh
#   # optional: IMAGE=harbor.../humble_add_impedance ./scripts/run_marvin_m6_impedance.sh
#
# Inside container (P4 / Skye bridge-less):
#   source /marvin_ws/install/setup.bash
#   export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   ros2 launch factr_teleop start_teleop_m6_dual_gento.launch.py use_keyboard:=true
#
# Host must already run skye_robot_driver on the same ROS_DOMAIN_ID.
# Do NOT start gento_robot_driver in parallel.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MARVIN_WS="${MARVIN_WS:-${REPO_ROOT}/marvin_ws}"

IMAGE="${1:-${IMAGE:-harbor.amigos-robot.com/tmp/marvin-m6-ros2:humble}}"

ROBOT_IP="${ROBOT_IP:-6.6.7.190}"
ROBOT_GRIPPER_PORT_LEFT="${ROBOT_GRIPPER_PORT_LEFT:-/dev/ttyUSB0}"
ROBOT_GRIPPER_PORT_RIGHT="${ROBOT_GRIPPER_PORT_RIGHT:-/dev/ttyUSB1}"
USE_LEFT_GRIPPER="${USE_LEFT_GRIPPER:-false}"
USE_RIGHT_GRIPPER="${USE_RIGHT_GRIPPER:-false}"

# Pass either a full /dev path or a /dev/serial/by-id basename.
# 优先读 bind_leader_arms.py 写入的绑定（换臂：python3 /scripts/bind_leader_arms.py）
BINDING_ENV="${MARVIN_WS}/.skye/leader_arms.env"
if [[ -f "${BINDING_ENV}" ]]; then
  # shellcheck disable=SC1090
  source "${BINDING_ENV}"
fi
ROBOT_LEADER_DYNAMIXEL_PORT_LEFT="${ROBOT_LEADER_DYNAMIXEL_PORT_LEFT:-usb-FTDI_USB__-__Serial_Converter_FTB8HNOT-if00-port0}"
ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT="${ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT:-usb-FTDI_USB__-__Serial_Converter_FTAO51EA-if00-port0}"

if [[ ! -f "${MARVIN_WS}/fastrtps_no_shm.xml" ]]; then
  echo "ERROR: missing ${MARVIN_WS}/fastrtps_no_shm.xml" >&2
  exit 1
fi

if [[ ! -f "${MARVIN_WS}/install/setup.bash" ]]; then
  echo "marvin_ws/install missing; trying bootstrap (GitLab Package Registry)..."
  bash "${SCRIPT_DIR}/bootstrap_marvin_install.sh"
fi

# Keep package launch/config in sync with tracked overlay (install/ is often gitignored).
bash "${SCRIPT_DIR}/sync_marvin_overlay.sh"

DOCKER_ARGS=(
  --rm
  -it
  --net=host
  --ipc=host
  --privileged
  --group-add dialout
  --cap-add SYS_NICE
  --ulimit rtprio=99
  --ulimit memlock=-1
  -e "ROBOT_IP=${ROBOT_IP}"
  -e "ROBOT_GRIPPER_PORT_LEFT=${ROBOT_GRIPPER_PORT_LEFT}"
  -e "ROBOT_GRIPPER_PORT_RIGHT=${ROBOT_GRIPPER_PORT_RIGHT}"
  -e "USE_LEFT_GRIPPER=${USE_LEFT_GRIPPER}"
  -e "USE_RIGHT_GRIPPER=${USE_RIGHT_GRIPPER}"
  -e "ROBOT_LEADER_DYNAMIXEL_PORT_LEFT=${ROBOT_LEADER_DYNAMIXEL_PORT_LEFT}"
  -e "ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT=${ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-21}"
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  -e "FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml"
  -v /dev:/dev
  -v "${MARVIN_WS}:/marvin_ws"
  -v "${SCRIPT_DIR}:/scripts:ro"
  -w /marvin_ws
)

if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY}"
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
fi

echo "Mount: ${MARVIN_WS} -> /marvin_ws"
echo "Image: ${IMAGE}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml"
exec docker run "${DOCKER_ARGS[@]}" "${IMAGE}"
