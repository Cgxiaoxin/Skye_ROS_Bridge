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

echo "== P6.1: publish dummy policy chunk + wait abs =="
export ABS_FILE
python3 <<'PY'
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from skye_hitl_dagger.msg import PolicyActionChunk

ABS_FILE = os.environ["ABS_FILE"]

rclpy.init()
node = Node("verify_hitl_p61_dummy_pub")
pub = node.create_publisher(PolicyActionChunk, "/skye/policy_action", 1)
abs_msgs: list[JointState] = []


def on_abs(msg: JointState) -> None:
    if msg.position:
        abs_msgs.append(msg)


node.create_subscription(
    JointState, "/gento/left_joint_control_abs", on_abs, 10)
msg = PolicyActionChunk()
msg.policy_version = "verify_p61"
msg.chunk_size = 16
msg.dt = 0.05
msg.left_joints = [0.1] * 112
msg.right_joints = [-0.1] * 112
msg.left_gripper = [0.0] * 16
msg.right_gripper = [0.0] * 16

match_deadline = time.monotonic() + 2.0
while time.monotonic() < match_deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        break
else:
    time.sleep(2.0)

publish_end = time.monotonic() + 3.0
while time.monotonic() < publish_end and not abs_msgs:
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)

if not abs_msgs:
    raise SystemExit("FAIL: no messages on /gento/left_joint_control_abs")

rate_start_count = len(abs_msgs)
rate_deadline = time.monotonic() + 1.0
while time.monotonic() < rate_deadline:
    msg.header.stamp = node.get_clock().now().to_msg()
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
rate_count = len(abs_msgs) - rate_start_count
if rate_count < 5:
    raise SystemExit(
        f"FAIL: abs rate too low ({rate_count} msgs in 1s, need >=5)")

with open(ABS_FILE, "w", encoding="utf-8") as f:
    f.write("position:\n")
    for p in abs_msgs[0].position:
        f.write(f"- {p}\n")

node.destroy_node()
rclpy.shutdown()
PY

echo "== P6.1: abs joint control sample + hz =="
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
match_deadline = time.monotonic() + 2.0
while time.monotonic() < match_deadline:
    rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() > 0:
        break
else:
    time.sleep(2.0)
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
