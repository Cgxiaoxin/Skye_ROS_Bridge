#!/usr/bin/env bash
# Enter Marvin M6 FACTR docker with marvin_ws mounted at /marvin_ws.
#
# Usage:
#   ./scripts/run_marvin_m6_impedance.sh thor
#   ./scripts/run_marvin_m6_impedance.sh orin
#   ./scripts/run_marvin_m6_impedance.sh              # fallback: env / machine file / thor
#   IMAGE=harbor.../img ./scripts/run_marvin_m6_impedance.sh orin
#
# First arg thor|orin selects machine profile; docker image stays via IMAGE=.
# Default (MARVIN_LAUNCH_CMD unset): opens an interactive shell in the container.
# Inside container (P4 / Skye bridge-less):
#   source /marvin_ws/install/setup.bash
#   export ROS_DOMAIN_ID=21 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   ros2 launch factr_teleop start_teleop_m6_dual_gento.launch.py use_keyboard:=true
#
# Non-interactive (operator UI / automation): set MARVIN_LAUNCH_CMD and the
# script runs it via `bash -lc` in the foreground instead of a shell, e.g.
#   MARVIN_LAUNCH_CMD='source /marvin_ws/install/setup.bash && \
#     ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py use_keyboard:=false' \
#     ./scripts/run_marvin_m6_impedance.sh
# A TTY is only allocated when stdin is a terminal, so it is safe under a
# supervisor that pipes stdout/stderr.
#
# Host must already run skye_robot_driver on the same ROS_DOMAIN_ID.
# Do NOT start gento_robot_driver in parallel.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MARVIN_WS="${MARVIN_WS:-${REPO_ROOT}/marvin_ws}"
# shellcheck source=lib/robot_profile.sh
source "${SCRIPT_DIR}/lib/robot_profile.sh"
apply_robot_profile_arg "$@"
set -- "${_ROBOT_PROFILE_REMAINING[@]}"
ROBOT_PROFILE="$(resolve_robot_profile "${MARVIN_WS}")"
validate_robot_profile "${ROBOT_PROFILE}" || exit 1
export ROBOT_PROFILE

IMAGE="${IMAGE:-harbor.amigos-robot.com/tmp/marvin-m6-ros2:humble}"
# Legacy: first leftover arg may still be a docker image tag/path.
if [[ -n "${1:-}" && "${1}" != -* ]]; then
  IMAGE="${1}"
  shift
fi

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

MARVIN_LAUNCH_CMD="${MARVIN_LAUNCH_CMD:-}"
# Fixed name so Operator UI / scripts can `docker rm -f` on session stop.
MARVIN_CONTAINER_NAME="${MARVIN_CONTAINER_NAME:-skye_marvin_m6}"

# Drop a previous orphan with the same name (Operator UI stop / crashed run).
if docker inspect "${MARVIN_CONTAINER_NAME}" >/dev/null 2>&1; then
  echo "Removing existing container ${MARVIN_CONTAINER_NAME} …"
  docker rm -f "${MARVIN_CONTAINER_NAME}" >/dev/null || true
fi

DOCKER_ARGS=(
  --rm
  --name "${MARVIN_CONTAINER_NAME}"
  -i
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
  -e "ROBOT_PROFILE=${ROBOT_PROFILE}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-21}"
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  -e "FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml"
  -v /dev:/dev
  -v "${MARVIN_WS}:/marvin_ws"
  -v "${SCRIPT_DIR}:/scripts:ro"
  -w /marvin_ws
)

# Only allocate a TTY when we actually have one; a supervisor-launched run pipes
# its stdio and `docker run -t` would fail there.
if [[ -t 0 ]]; then
  DOCKER_ARGS+=(-t)
fi

DOCKER_ARGS+=(-e "MARVIN_LAUNCH_CMD=${MARVIN_LAUNCH_CMD}")

if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY}"
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
fi

echo "Mount: ${MARVIN_WS} -> /marvin_ws"
echo "Profile: ${ROBOT_PROFILE}"
echo "Image: ${IMAGE}"
echo "FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml"

if [[ -n "${MARVIN_LAUNCH_CMD}" ]]; then
  echo "Launch: ${MARVIN_LAUNCH_CMD}"
  echo "Container: ${MARVIN_CONTAINER_NAME}"
  exec docker run "${DOCKER_ARGS[@]}" "${IMAGE}" bash -lc "${MARVIN_LAUNCH_CMD}"
fi

echo "Launch: interactive shell (set MARVIN_LAUNCH_CMD for non-interactive run)"
echo "Container: ${MARVIN_CONTAINER_NAME}"
exec docker run "${DOCKER_ARGS[@]}" "${IMAGE}"
