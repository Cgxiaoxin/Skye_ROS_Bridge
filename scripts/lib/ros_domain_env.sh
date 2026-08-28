# Shared ROS 2 / FastDDS environment for Skye host-side scripts.
# Source from other scripts:  source "${SCRIPT_DIR}/lib/ros_domain_env.sh"

: "${REPO_ROOT:?REPO_ROOT must be set before sourcing ros_domain_env.sh}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

FASTRTPS_XML="${REPO_ROOT}/marvin_ws/fastrtps_no_shm.xml"
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
