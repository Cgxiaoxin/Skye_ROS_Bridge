#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$WS"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
LOG_DIR="${ROS_LOG_DIR:-/tmp/skye_ros_log}"
export ROS_LOG_DIR="$LOG_DIR"
export ROS_HOME="${ROS_HOME:-/tmp/skye_ros_home}"
mkdir -p "$LOG_DIR" "$ROS_HOME"
LOG_FILE="$LOG_DIR/verify_hitl_abs_interfaces.out"

ros2 run skye_robot_driver skye_robot_driver --ros-args \
  -p connect_on_startup:=false \
  -r /left_joint_control_abs:=/gento/left_joint_control_abs \
  -r /right_joint_control_abs:=/gento/right_joint_control_abs \
  >"$LOG_FILE" 2>&1 &
NODE_PID=$!
trap 'kill "$NODE_PID" >/dev/null 2>&1 || true' EXIT

sleep 2
TOPICS="$(ros2 topic list)"
grep -qx '/gento/left_joint_control_abs' <<<"$TOPICS"
grep -qx '/gento/right_joint_control_abs' <<<"$TOPICS"
NODE_INFO="$(ros2 node info /skye_robot_driver)"
grep -q '/gento/left_joint_control_abs' <<<"$NODE_INFO"
grep -q '/gento/right_joint_control_abs' <<<"$NODE_INFO"

echo "PASS: absolute joint control topics are advertised"
