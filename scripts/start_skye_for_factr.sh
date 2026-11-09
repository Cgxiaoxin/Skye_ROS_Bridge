#!/usr/bin/env bash
# P4 host helper: start skye_robot_driver for FACTR teleop (bridge-less /gento/*).
# FACTR small arms still run in Docker via scripts/run_marvin_m6_impedance.sh.
#
# Usage:
#   ./scripts/start_skye_for_factr.sh thor
#   ./scripts/start_skye_for_factr.sh orin
#   ./scripts/start_skye_for_factr.sh          # fallback: env / machine file / thor
#
# Priority: CLI name > ROBOT_PROFILE env > marvin_ws/.skye/robot_profile
#           > marvin_ws/robot_profile > thor
# Extra ros2 launch args may follow the profile name.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck source=lib/robot_profile.sh
source "${SCRIPT_DIR}/lib/robot_profile.sh"

MARVIN_WS="${MARVIN_WS:-${REPO_ROOT}/marvin_ws}"
apply_robot_profile_arg "$@"
set -- "${_ROBOT_PROFILE_REMAINING[@]}"
ROBOT_PROFILE="$(resolve_robot_profile "${MARVIN_WS}")"
validate_robot_profile "${ROBOT_PROFILE}" || exit 1
export ROBOT_PROFILE
WS="${REPO_ROOT}/skye_ros2_ws"

# Profile default: thor=20, orin=21. Explicit ROS_DOMAIN_ID in the environment wins.
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  ROS_DOMAIN_ID="$(robot_ros_domain_id "${ROBOT_PROFILE}")"
fi
export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

# FastDDS 会对 FASTRTPS_DEFAULT_PROFILES_FILE 做 realpath；文件不存在会报
# "XMLPARSER Error: realpath failed"。外部若已 export 成无效路径，这里强制回退。
# FastDDS profile ships with the driver package (marvin_ws/fastrtps_no_shm.xml
# was removed in cb0f3e7); the helper also tolerates an invalid inherited path.
# shellcheck source=lib/ros_domain_env.sh
source "${SCRIPT_DIR}/lib/ros_domain_env.sh"

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
# Prefer system python for ament. Thor apex/jazzy first; humble kept as fallback.
export PATH="/usr/bin:${PATH}"
_ros_setup=""
for _candidate in \
  "${ROS_SETUP:-}" \
  /opt/kernelmind/apex/install/setup.bash \
  /opt/ros/jazzy/setup.bash \
  /opt/ros/humble/setup.bash
do
  if [[ -n "${_candidate}" && -f "${_candidate}" ]]; then
    _ros_setup="${_candidate}"
    break
  fi
done
if [[ -z "${_ros_setup}" ]]; then
  echo "ERROR: no ROS setup.bash found (tried apex/jazzy/humble; set ROS_SETUP=...)" >&2
  exit 1
fi
echo "== sourcing ${_ros_setup} =="
# shellcheck disable=SC1090
source "${_ros_setup}"
source install/setup.bash
set -u

echo "== skye_robot_driver profile=${ROBOT_PROFILE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} =="
echo "   controller_ip=$(robot_controller_ip "${ROBOT_PROFILE}")"
echo "   FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo "Expect FACTR remap: /gento/joint_states + /gento/{left,right}_joint_control"
echo "Mode default: imp_joint (2). Keyboard in docker: 1=sync 2=teleop 3=stop"
# enable_gripper defaults true in launch; override with ENABLE_GRIPPER=false if needed.
exec ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  connect_on_startup:="${CONNECT_ON_STARTUP:-true}" \
  enable_gripper:="${ENABLE_GRIPPER:-true}" \
  robot_profile:="${ROBOT_PROFILE}" \
  "$@"
