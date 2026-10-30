# Shared ROS 2 / FastDDS environment for Skye host-side scripts.
# Source from other scripts:  source "${SCRIPT_DIR}/lib/ros_domain_env.sh"
#
# Domain by profile when ROS_DOMAIN_ID is unset: thor=20, orin=21.
# Requires robot_profile.sh already sourced (or ROBOT_PROFILE set).

: "${REPO_ROOT:?REPO_ROOT must be set before sourcing ros_domain_env.sh}"

if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  if declare -F robot_ros_domain_id >/dev/null 2>&1 && [[ -n "${ROBOT_PROFILE:-}" ]]; then
    ROS_DOMAIN_ID="$(robot_ros_domain_id "${ROBOT_PROFILE}")"
  else
    ROS_DOMAIN_ID=21
  fi
fi
export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

FASTRTPS_XML="${REPO_ROOT}/marvin_ws/fastrtps_no_shm.xml"
if [[ ! -f "${FASTRTPS_XML}" && -f "${REPO_ROOT}/marvin_ws1/fastrtps_no_shm.xml" ]]; then
  FASTRTPS_XML="${REPO_ROOT}/marvin_ws1/fastrtps_no_shm.xml"
fi
if [[ -n "${FASTRTPS_DEFAULT_PROFILES_FILE:-}" && -f "${FASTRTPS_DEFAULT_PROFILES_FILE}" ]]; then
  FASTRTPS_XML="${FASTRTPS_DEFAULT_PROFILES_FILE}"
fi
if [[ ! -f "${FASTRTPS_XML}" ]]; then
  echo "ERROR: FastDDS xml missing: ${FASTRTPS_XML}" >&2
  return 1 2>/dev/null || exit 1
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_XML}"

skye_source_ros() {
  local ws="${REPO_ROOT}/skye_ros2_ws"
  if [[ ! -f "${ws}/install/setup.bash" ]]; then
    echo "ERROR: ${ws}/install/setup.bash missing; run skye_ros2_ws/scripts/build.sh" >&2
    return 1
  fi
  set +u
  export PATH="/usr/bin:${PATH}"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
  # shellcheck disable=SC1091
  source "${ws}/install/setup.bash"
  set -u
}
