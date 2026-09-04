#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_PROFILE="${ROBOT_PROFILE:-thor}"
ROBOT_PROFILE="${ROBOT_PROFILE,,}"
case "${ROBOT_PROFILE}" in
  thor|orin) ;;
  *)
    echo "ERROR: ROBOT_PROFILE must be thor|orin (got: ${ROBOT_PROFILE})" >&2
    exit 1
    ;;
esac
WS="${REPO_ROOT}/skye_ros2_ws"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

FASTRTPS_XML="${REPO_ROOT}/marvin_ws/fastrtps_no_shm.xml"
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]]; then
  FASTRTPS_XML="${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi
[[ -f "${FASTRTPS_XML}" ]] \
  || { echo "ERROR: FastDDS xml missing: ${FASTRTPS_XML}" >&2; exit 1; }
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_XML}"

cd "${WS}"
if [[ ! -f install/setup.bash ]]; then
  echo "== building skye_follower_align =="
  bash ./scripts/build.sh skye_follower_align
fi

set +u
export PATH="/usr/bin:${PATH}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

echo "== follower_align profile=${ROBOT_PROFILE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} =="
exec ros2 launch skye_follower_align follower_align.launch.py \
  robot_profile:="${ROBOT_PROFILE}" enable_keyboard:=true
