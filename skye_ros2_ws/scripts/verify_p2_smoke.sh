#!/usr/bin/env bash
# P2 smoke: require live driver on ROS_DOMAIN_ID (default 20).
# Checks ImpJoint mode (FX_STATE_IMP_JOINT=2) and joint_states rate.
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
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"

echo "== P2: node / topics =="
ros2 node list | grep -q '/skye_robot_driver' \
  || { echo "FAIL: skye_robot_driver not running (launch with connect_on_startup:=true)"; exit 1; }

ros2 topic list | grep -q '/gento/joint_states' || { echo "FAIL: missing joint_states"; exit 1; }
ros2 topic list | grep -q '/gento/robot_state' || { echo "FAIL: missing robot_state"; exit 1; }
ros2 service list | grep -q '/gento/set_mode' || { echo "FAIL: missing set_mode service"; exit 1; }

echo "== P2: robot_state (expect left/right FX_STATE_IMP_JOINT=2) =="
# State publishers use RELIABLE (FACTR sync); CLI must match.
STATE="$(ros2 topic echo --once /gento/robot_state --qos-reliability reliable 2>/dev/null || true)"
echo "$STATE"
echo "$STATE" | grep -q 'data:' || { echo "FAIL: no robot_state data (is hardware linked?)"; exit 1; }

LEFT="$(echo "$STATE" | awk '/data:/{getline; print $2; exit}')"
RIGHT="$(echo "$STATE" | awk '/data:/{getline; getline; print $2; exit}')"
echo "parsed left=$LEFT right=$RIGHT"
if [[ "$LEFT" != "2" || "$RIGHT" != "2" ]]; then
  echo "WARN: expected IMP_JOINT=2,2. Got ${LEFT},${RIGHT}."
  echo "      Try: ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode '{mode: 2}'"
  # Soft-fail only if completely unknown
  if [[ "$LEFT" == "200" || "$RIGHT" == "200" || -z "$LEFT" ]]; then
    echo "FAIL: robot not in a usable control state"
    exit 1
  fi
fi

echo "== P2: joint_states hz (sample ~1s) =="
timeout 2s ros2 topic hz /gento/joint_states --window 50 2>&1 | tee /tmp/skye_p2_hz.txt || true
grep -E 'average rate:' /tmp/skye_p2_hz.txt | head -1 || {
  echo "FAIL: no joint_states rate"
  exit 1
}

echo "== P2: feedback sample =="
ros2 topic echo --once /gento/joint_states --qos-reliability reliable | head -40

echo "P2 VERIFY OK (feedback + mode path). Motion smoke: manually nudge one joint ±0.03 rad."
echo "  See docs/teleop_sop.md"
