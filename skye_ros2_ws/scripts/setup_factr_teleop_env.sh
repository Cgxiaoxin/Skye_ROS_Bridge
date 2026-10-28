#!/usr/bin/env bash
# Public environment for skye_factr_teleop dual-arm trials (phase 7.5).
# Usage in each new terminal:
#   source /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws/scripts/setup_factr_teleop_env.sh
#
# Sets ROS Humble + workspace overlay, domain 73, FastRTPS, and config path vars.
# Does not start hardware.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: source this script, do not execute it." >&2
  exit 1
fi

_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$_WS" || return 1

export PATH="/usr/bin:${PATH}"
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "$_WS/install/setup.bash"
set -u

export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

LEFT_CONFIG="$_WS/src/skye_factr_teleop/config/grav_comp_m6_left.yaml"
RIGHT_CONFIG="$_WS/src/skye_factr_teleop/config/grav_comp_m6_right.yaml"
DRIVER_CONFIG="$_WS/src/skye_robot_driver/config/skye_robot.yaml"
export LEFT_CONFIG RIGHT_CONFIG DRIVER_CONFIG

unset _WS

echo "skye_factr_teleop env ready: ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION"
echo "  LEFT_CONFIG=$LEFT_CONFIG"
echo "  RIGHT_CONFIG=$RIGHT_CONFIG"
echo "  DRIVER_CONFIG=$DRIVER_CONFIG"
