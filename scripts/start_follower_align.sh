#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROBOT_PROFILE="${ROBOT_PROFILE:-thor}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export FASTRTPS_DEFAULT_PROFILES_FILE="${REPO_ROOT}/marvin_ws/fastrtps_no_shm.xml"
source /opt/ros/humble/setup.bash
source "${REPO_ROOT}/skye_ros2_ws/install/setup.bash"
exec ros2 launch skye_follower_align follower_align.launch.py \
  robot_profile:="${ROBOT_PROFILE}" enable_keyboard:=true
