#!/usr/bin/env bash
# Public environment for skye_factr_teleop dual-arm trials.
# Usage in each new terminal:
#   source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh
#   source .../setup_factr_teleop_env.sh orin
#   source .../setup_factr_teleop_env.sh thor
#   ROBOT_PROFILE=orin source .../setup_factr_teleop_env.sh
#
# Sets ROS Humble + workspace overlay, FastRTPS, config path vars, and
# ROS_DOMAIN_ID by profile: orin=21, thor=20.
# Does not start hardware.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: source this script, do not execute it." >&2
  exit 1
fi

_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
_REPO="$(cd "$_WS/.." && pwd)"
cd "$_WS" || return 1

# shellcheck source=../../scripts/lib/robot_profile.sh
source "${_REPO}/scripts/lib/robot_profile.sh"

# Optional first arg: thor | orin
if [[ "${1:-}" == "thor" || "${1:-}" == "orin" ]]; then
  export ROBOT_PROFILE="${1}"
  shift
fi

_MARVIN_WS="${MARVIN_WS:-${_REPO}/marvin_ws}"
# Prefer explicit ROBOT_PROFILE; else persisted machine file; else orin (domain 21).
# Do not fall through resolve_robot_profile's hard default "thor" on Orin boxes.
if [[ -z "${ROBOT_PROFILE:-}" ]]; then
  if [[ -f "${_MARVIN_WS}/.skye/robot_profile" || -f "${_MARVIN_WS}/robot_profile" ]]; then
    ROBOT_PROFILE="$(resolve_robot_profile "${_MARVIN_WS}")"
  else
    ROBOT_PROFILE="orin"
  fi
fi
ROBOT_PROFILE="${ROBOT_PROFILE,,}"
validate_robot_profile "${ROBOT_PROFILE}" || return 1
export ROBOT_PROFILE

export PATH="/usr/bin:${PATH}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$_WS/install/setup.bash"
set -u

# Always apply profile domain (replaces the old hard-coded 73).
export ROS_DOMAIN_ID="$(robot_ros_domain_id "${ROBOT_PROFILE}")"
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

LEFT_CONFIG="$_WS/src/skye_factr_teleop/config/grav_comp_m6_left.yaml"
RIGHT_CONFIG="$_WS/src/skye_factr_teleop/config/grav_comp_m6_right.yaml"
DRIVER_CONFIG="$_WS/src/skye_robot_driver/config/skye_robot.yaml"
export LEFT_CONFIG RIGHT_CONFIG DRIVER_CONFIG

echo "skye_factr_teleop env ready: ROBOT_PROFILE=$ROBOT_PROFILE ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
echo "  LEFT_CONFIG=$LEFT_CONFIG"
echo "  RIGHT_CONFIG=$RIGHT_CONFIG"
echo "  DRIVER_CONFIG=$DRIVER_CONFIG"

unset _WS _REPO _MARVIN_WS
