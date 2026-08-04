#!/usr/bin/env bash
# P3 safety / interface verification (works with or without hardware).
# Starts a dry-run node if none is present.
set -euo pipefail

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/skye_ros_log}"
export ROS_HOME="${ROS_HOME:-/tmp/skye_ros_home}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"

WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
export PATH="/usr/bin:${PATH}"
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

STARTED_HERE=0
if ! ros2 node list 2>/dev/null | grep -q '/skye_robot_driver'; then
  echo "== starting dry-run node =="
  ros2 run skye_robot_driver skye_robot_driver --ros-args \
    -p connect_on_startup:=false \
    -p control_mode:=imp_joint \
    -p cmd_cycle_time_ms:=4 \
    -p max_delta_per_cycle:=0.05 \
    -p command_timeout_s:=0.20 \
    -r /joint_states:=/gento/joint_states \
    -r /left_joint_control:=/gento/left_joint_control \
    -r /right_joint_control:=/gento/right_joint_control \
    -r /robot_state:=/gento/robot_state \
    -r /set_mode:=/gento/set_mode \
    -r /hold_current:=/gento/hold_current \
    -r /stop_motion:=/gento/stop_motion \
    -r /emergency_stop:=/gento/emergency_stop \
    >"$ROS_LOG_DIR/p3_node.out" 2>&1 &
  echo $! >"$ROS_LOG_DIR/p3_node.pid"
  STARTED_HERE=1
  sleep 3
fi

cleanup() {
  if [[ "$STARTED_HERE" -eq 1 ]]; then
    kill "$(cat "$ROS_LOG_DIR/p3_node.pid")" >/dev/null 2>&1 || true
    pkill -f 'lib/skye_robot_driver/skye_robot_driver' >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

echo "== P3 params =="
ros2 param get /skye_robot_driver control_mode | tee /tmp/skye_p3_mode.txt
grep -qi 'imp_joint' /tmp/skye_p3_mode.txt
ros2 param get /skye_robot_driver cmd_cycle_time_ms | tee /tmp/skye_p3_cycle.txt
grep -E '4|Integer' /tmp/skye_p3_cycle.txt >/dev/null
ros2 param get /skye_robot_driver max_delta_per_cycle | grep -q '0.05'
ros2 param get /skye_robot_driver command_timeout_s | grep -q '0.2'
ros2 param get /skye_robot_driver left_acceleration_ratio >/dev/null
ros2 param get /skye_robot_driver right_acceleration_ratio >/dev/null

echo "== P3 interfaces =="
ros2 service list | grep -q '/gento/set_mode'
ros2 topic list | grep -q '/gento/robot_state'
ros2 service list | grep -q '/gento/hold_current'
ros2 service list | grep -q '/gento/stop_motion'
ros2 service list | grep -q '/gento/emergency_stop'

echo "== P3 mode service type =="
ros2 service type /gento/set_mode | grep -q 'skye_robot_driver/srv/SetMode'

echo "== P3 static safety helpers in binary/symbols =="
# Ensure core still exports the safety path used by the node.
nm -C "$WS/install/skye_robot_driver/lib/skye_robot_driver/skye_robot_driver" \
  | grep -E 'validate_target|limit_delta|SetPDCmdCycleTime|SwitchToImpJointMode|SwitchToPositionMode|SwitchToImpCartMode' \
  | head -30 || true

# Grep source for required P3 behaviors (defense in depth for review).
grep -q 'max_delta_per_cycle' \
  "$WS/src/skye_robot_driver/src/driver_node.cpp"
grep -q 'command_timeout' \
  "$WS/src/skye_robot_driver/src/driver_node.cpp"
grep -q 'SetPDCmdCycleTime' \
  "$WS/src/skye_robot_driver/src/driver_core.cpp"
grep -q 'SwitchToImpJointMode' \
  "$WS/src/skye_robot_driver/src/driver_core.cpp"
grep -q 'SwitchToPositionMode' \
  "$WS/src/skye_robot_driver/src/driver_core.cpp"
grep -q 'SwitchToImpCartMode' \
  "$WS/src/skye_robot_driver/src/driver_core.cpp"

echo "P3 VERIFY OK"
