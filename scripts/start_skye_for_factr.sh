#!/usr/bin/env bash
# P4 host helper: start skye_robot_driver for FACTR teleop (bridge-less /gento/*).
# FACTR small arms still run in Docker via scripts/run_marvin_m6_impedance.sh.
#
# Usage:
#   Terminal A (host):  ./scripts/start_skye_for_factr.sh
#   Terminal B:         ./scripts/run_marvin_m6_impedance.sh
#     inside docker:    see docs/小臂大臂启动步骤.md

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS="${REPO_ROOT}/skye_ros2_ws"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE:-${REPO_ROOT}/marvin_ws/fastrtps_no_shm.xml}"

if pgrep -x gento_robot_driver >/dev/null 2>&1; then
  echo "ERROR: gento_robot_driver is running. Stop it first (only one SDK client)." >&2
  exit 1
fi
if pgrep -f 'lib/skye_robot_driver/skye_robot_driver' >/dev/null 2>&1; then
  echo "ERROR: skye_robot_driver already running." >&2
  exit 1
fi

cd "${WS}"
if [[ ! -f install/setup.bash ]]; then
  echo "== building skye_robot_driver =="
  bash ./scripts/build.sh
fi

set +u
# Prefer system python for ament
export PATH="/usr/bin:${PATH}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

echo "== skye_robot_driver ROS_DOMAIN_ID=${ROS_DOMAIN_ID} =="
echo "Expect FACTR remap: /gento/joint_states + /gento/{left,right}_joint_control"
echo "Mode default: imp_joint (2). Keyboard in docker: 1=sync 2=teleop 3=stop"
exec ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  connect_on_startup:="${CONNECT_ON_STARTUP:-true}"
