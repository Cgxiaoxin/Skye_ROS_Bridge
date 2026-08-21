#!/usr/bin/env bash
# P6.1 bench verify (no robot): arbiter + dummy chunk + takeover + control_mode.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$WS"

export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/skye_hitl_p61_verify_log}"
export ROS_HOME="${ROS_HOME:-/tmp/skye_hitl_p61_verify_home}"
mkdir -p "$ROS_LOG_DIR" "$ROS_HOME"
LOG_FILE="$ROS_LOG_DIR/verify_hitl_p61_interfaces.out"
ABS_FILE="$ROS_LOG_DIR/verify_hitl_p61_abs.txt"

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PATH="/usr/bin:/bin:${PATH}"
ros2 daemon stop >/dev/null 2>&1 || true
ros2 daemon start >/dev/null 2>&1 || true

cleanup() {
  kill "$ARB_PID" >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "== P6.1: start control_arbiter =="
ros2 run skye_hitl_dagger control_arbiter >"$LOG_FILE" 2>&1 &
ARB_PID=$!
sleep 2

ros2 node list | grep -q '/control_arbiter' \
  || { echo "FAIL: control_arbiter not running"; exit 1; }

TOPICS="$(ros2 topic list)"
for t in /skye/policy_action /skye/intervention_cmd /skye/control_mode \
         /gento/left_joint_control_abs /gento/right_joint_control_abs; do
  grep -qx "$t" <<<"$TOPICS" || { echo "FAIL: missing topic $t"; exit 1; }
done

echo "== P6.1: publish dummy policy chunk =="
python3 <<'PY'
import rclpy
from rclpy.node import Node
from skye_hitl_dagger.msg import PolicyActionChunk

rclpy.init()
node = Node("verify_hitl_p61_dummy_pub")
pub = node.create_publisher(PolicyActionChunk, "/skye/policy_action", 1)
msg = PolicyActionChunk()
msg.policy_version = "verify_p61"
msg.chunk_size = 16
msg.dt = 0.05
msg.left_joints = [0.1] * 112
msg.right_joints = [-0.1] * 112
msg.left_gripper = [0.0] * 16
msg.right_gripper = [0.0] * 16
for _ in range(10):
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
node.destroy_node()
rclpy.shutdown()
PY

sleep 0.5
echo "== P6.1: abs joint control sample =="
timeout 5s ros2 topic echo --once /gento/left_joint_control_abs \
  2>/dev/null | tee "$ABS_FILE"
grep -q 'position:' "$ABS_FILE" \
  || { echo "FAIL: no messages on /gento/left_joint_control_abs"; exit 1; }

echo "== P6.1: takeover -> HANDOVER_SYNC =="
python3 <<'PY'
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from skye_hitl_dagger.msg import ControlMode

rclpy.init()
node = Node("verify_hitl_p61_takeover_check")
seen = []

def on_mode(msg: ControlMode) -> None:
    seen.append(msg.mode)

node.create_subscription(ControlMode, "/skye/control_mode", on_mode, 10)
pub = node.create_publisher(String, "/skye/intervention_cmd", 10)
deadline = time.monotonic() + 3.0
while time.monotonic() < deadline and not seen:
    rclpy.spin_once(node, timeout_sec=0.1)
for _ in range(20):
    pub.publish(String(data="takeover"))
    rclpy.spin_once(node, timeout_sec=0.05)
    if any("HANDOVER" in m for m in seen):
        break
else:
    raise SystemExit("FAIL: control_mode missing HANDOVER after takeover")
node.destroy_node()
rclpy.shutdown()
PY

echo "PASS: HITL P6.1 interfaces (arbiter, chunk, takeover, abs publish)"
