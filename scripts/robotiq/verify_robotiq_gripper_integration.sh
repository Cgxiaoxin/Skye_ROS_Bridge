#!/usr/bin/env bash
# 双臂 Robotiq 夹爪集成测试 — 需 skye_robot_driver 已连接 6.6.7.190
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
WS="${REPO_ROOT}/skye_ros2_ws"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
unset ROS_LOCALHOST_ONLY || true
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
FASTRTPS_XML="${REPO_ROOT}/marvin_ws/fastrtps_no_shm.xml"
[[ -f "${FASTRTPS_XML}" ]] && export FASTRTPS_DEFAULT_PROFILES_FILE="${FASTRTPS_XML}"

set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/setup.bash"
set -u

fail() { echo "FAIL: $*" >&2; exit 1; }
pass() { echo "PASS: $*"; }

echo "== 1) 检查 skye_robot_driver =="
ros2 node list | grep -q '/skye_robot_driver' \
  || fail "skye_robot_driver 未运行。先: ./scripts/start_skye_for_factr.sh"

echo "== 2) 检查夹爪类型参数 =="
LT="$(ros2 param get /skye_robot_driver gripper_left_type 2>/dev/null | tail -1)"
RT="$(ros2 param get /skye_robot_driver gripper_right_type 2>/dev/null | tail -1)"
echo "  gripper_left_type=${LT}"
echo "  gripper_right_type=${RT}"
[[ "${LT}" == *robotiq* ]] || fail "gripper_left_type 不是 robotiq: ${LT}"
[[ "${RT}" == *robotiq* ]] || fail "gripper_right_type 不是 robotiq: ${RT}"
pass "双臂均为 robotiq"

echo "== 2b) 检查左右臂闭合开度 (mm) =="
LMIN="$(ros2 param get /skye_robot_driver gripper_left_robotiq_pos_min_mm 2>/dev/null | tail -1 | grep -oE '[0-9.]+' | tail -1)"
RMIN="$(ros2 param get /skye_robot_driver gripper_right_robotiq_pos_min_mm 2>/dev/null | tail -1 | grep -oE '[0-9.]+' | tail -1)"
echo "  gripper_left_robotiq_pos_min_mm=${LMIN}"
echo "  gripper_right_robotiq_pos_min_mm=${RMIN}"
python3 - <<PY
import sys
l, r = float("${LMIN:-0}"), float("${RMIN:-0}")
ok = abs(l - 13.0) < 0.01 and abs(r - 2.0) < 0.01
print(f"  期望: 左=13.0 mm 右=2.0 mm（2026-09-04 L/R 实体互换后）")
if not ok:
    print(f"FAIL: 闭合开度配置不符 (左={l}, 右={r})")
    sys.exit(1)
print("PASS: 闭合开度配置正确")
PY

echo "== 3) 检查 gripper topic =="
ros2 topic list | grep -q '/left_gripper/state' || fail "缺少 /left_gripper/state"
ros2 topic list | grep -q '/right_gripper/state' || fail "缺少 /right_gripper/state"
INFO_L="$(ros2 topic info /left_gripper/state -v 2>/dev/null | grep 'Node name' | head -1 || true)"
INFO_R="$(ros2 topic info /right_gripper/state -v 2>/dev/null | grep 'Node name' | head -1 || true)"
echo "  ${INFO_L}"
echo "  ${INFO_R}"
pass "gripper state topic 存在"

echo "== 4) 读初始反馈 (robotiq frame_id, 启动应张开≈1) =="
SL="$(ros2 topic echo --once /left_gripper/state 2>/dev/null || true)"
SR="$(ros2 topic echo --once /right_gripper/state 2>/dev/null || true)"
echo "${SL}" | head -8
echo "${SR}" | head -8
echo "${SL}" | grep -q 'robotiq_9' || fail "左爪 frame_id 不是 robotiq_9"
echo "${SR}" | grep -q 'robotiq_9' || fail "右爪 frame_id 不是 robotiq_9"
INIT_L="$(read_gripper_pos /left_gripper/state)"
INIT_R="$(read_gripper_pos /right_gripper/state)"
echo "  启动后左 position=${INIT_L} 右 position=${INIT_R}"
python3 - <<PY
l, r = float("${INIT_L:-0}"), float("${INIT_R:-0}")
if l < 0.85 or r < 0.85:
    print(f"WARN: 启动后夹爪应张开(FACTR语义≈1), 当前 左={l:.2f} 右={r:.2f}")
else:
    print(f"PASS: 启动后双臂张开 左={l:.2f} 右={r:.2f}")
PY
pass "双臂 Modbus 反馈有效 (robotiq_9)"

read_gripper_pos() {
  ros2 topic echo --once "$1" 2>/dev/null \
    | awk '/^position:/{getline; if ($1=="-") print $2; exit}'
}

pub_gripper() {
  local topic=$1 val=$2
  ros2 topic pub --once "${topic}" sensor_msgs/msg/JointState \
    "{name: ['gripper_joint'], position: [${val}]}" >/dev/null
}

echo "== 5) 左爪开合 (FACTR 语义: 1=开 0=闭, invert 后电机) =="
pub_gripper /left_teleop_gripper/ctrl 1.0
sleep 2.5
OPEN_L="$(read_gripper_pos /left_gripper/state)"
echo "  左开 position=${OPEN_L}"
pub_gripper /left_teleop_gripper/ctrl 0.0
sleep 2.5
CLOSE_L="$(read_gripper_pos /left_gripper/state)"
echo "  左闭 position=${CLOSE_L}"

echo "== 6) 右爪开合 =="
pub_gripper /right_teleop_gripper/ctrl 1.0
sleep 2.5
OPEN_R="$(read_gripper_pos /right_gripper/state)"
echo "  右开 position=${OPEN_R}"
pub_gripper /right_teleop_gripper/ctrl 0.0
sleep 2.5
CLOSE_R="$(read_gripper_pos /right_gripper/state)"
echo "  右闭 position=${CLOSE_R}"

# FACTR invert: state 回到扳机语义，开≈1 闭≈0
python3 - <<PY
open_l, close_l = float("${OPEN_L}"), float("${CLOSE_L}")
open_r, close_r = float("${OPEN_R}"), float("${CLOSE_R}")
ok = True
for name, o, c in [("左", open_l, close_l), ("右", open_r, close_r)]:
    if not (o > c + 0.2):
        print(f"FAIL: {name}爪开({o:.2f}) 应明显大于 闭({c:.2f})")
        ok = False
    else:
        print(f"PASS: {name}爪 开={o:.2f} 闭={c:.2f}")
raise SystemExit(0 if ok else 1)
PY

echo ""
echo "=== 集成测试全部通过 ==="
