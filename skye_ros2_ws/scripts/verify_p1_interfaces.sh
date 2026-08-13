#!/usr/bin/env bash
set -euo pipefail
export ROS_LOG_DIR=/tmp/skye_ros_log
export ROS_HOME=/tmp/skye_ros_home
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
WS=/data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws
cd "$WS"
set +u
# Prefer system python for ros2cli
export PATH="/usr/bin:${PATH}"
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID=42

pkill -f 'lib/skye_robot_driver/skye_robot_driver' >/dev/null 2>&1 || true
sleep 1

ros2 run skye_robot_driver skye_robot_driver --ros-args \
  -p connect_on_startup:=false \
  -r /joint_states:=/gento/joint_states \
  -r /left_joint_control:=/gento/left_joint_control \
  -r /right_joint_control:=/gento/right_joint_control \
  -r /robot_state:=/gento/robot_state \
  -r /set_mode:=/gento/set_mode \
  -r /hold_current:=/gento/hold_current \
  -r /stop_motion:=/gento/stop_motion \
  -r /emergency_stop:=/gento/emergency_stop \
  >"$ROS_LOG_DIR/verify_node.out" 2>&1 &
NPID=$!
echo "node_pid=$NPID"
sleep 4

{
  echo "==== node log ===="
  cat "$ROS_LOG_DIR/verify_node.out"
  echo "==== nodes ===="
  ros2 node list || true
  echo "==== topics ===="
  ros2 topic list -t || true
  echo "==== services ===="
  ros2 service list || true
  echo "==== node info ===="
  ros2 node info /skye_robot_driver || true
  echo "==== topic info left ===="
  ros2 topic info -v /gento/left_joint_control || true
  echo "==== topic info states ===="
  ros2 topic info -v /gento/joint_states || true
} | tee "$ROS_LOG_DIR/verify_report.txt"

kill "$NPID" >/dev/null 2>&1 || true
pkill -f 'lib/skye_robot_driver/skye_robot_driver' >/dev/null 2>&1 || true
sleep 1
echo "VERIFY_SCRIPT_DONE"
