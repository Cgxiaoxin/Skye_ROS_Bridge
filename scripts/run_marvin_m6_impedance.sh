#!/usr/bin/env bash
# Enter Marvin M6 FACTR docker with marvin_ws mounted at /marvin_ws.
#
# Usage:
#   ./scripts/run_marvin_m6_impedance.sh
#   # optional: IMAGE=harbor.../humble_add_impedance ./scripts/run_marvin_m6_impedance.sh
#
# Inside container (P4 / Skye bridge-less):
#   source /marvin_ws/install/setup.bash
#   export ROS_DOMAIN_ID=20 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
#   ros2 launch factr_teleop start_teleop_m6_dual_gento.launch.py use_keyboard:=true
#
# Host must already run skye_robot_driver on the same ROS_DOMAIN_ID.
# Do NOT start gento_robot_driver in parallel.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MARVIN_WS="${MARVIN_WS:-${REPO_ROOT}/marvin_ws}"

IMAGE="${1:-${IMAGE:-harbor.amigos-robot.com/tmp/marvin-m6-ros2:humble}}"

ROBOT_IP="${ROBOT_IP:-6.6.7.190}"
ROBOT_GRIPPER_PORT_LEFT="${ROBOT_GRIPPER_PORT_LEFT:-/dev/ttyUSB0}"
ROBOT_GRIPPER_PORT_RIGHT="${ROBOT_GRIPPER_PORT_RIGHT:-/dev/ttyUSB1}"
USE_LEFT_GRIPPER="${USE_LEFT_GRIPPER:-false}"
USE_RIGHT_GRIPPER="${USE_RIGHT_GRIPPER:-false}"

# Pass either a full /dev path or a /dev/serial/by-id basename.
# 优先读 bind_leader_arms.py 写入的绑定（换臂：python3 /scripts/bind_leader_arms.py）
BINDING_ENV="${MARVIN_WS}/.skye/leader_arms.env"
if [[ -f "${BINDING_ENV}" ]]; then
  # shellcheck disable=SC1090
  source "${BINDING_ENV}"
fi
ROBOT_LEADER_DYNAMIXEL_PORT_LEFT="${ROBOT_LEADER_DYNAMIXEL_PORT_LEFT:-usb-FTDI_USB__-__Serial_Converter_FTB8HNOT-if00-port0}"
ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT="${ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT:-usb-FTDI_USB__-__Serial_Converter_FTAO51EA-if00-port0}"

if [[ ! -f "${MARVIN_WS}/install/setup.bash" ]]; then
  echo "marvin_ws/install missing; trying bootstrap (GitLab Package Registry)..."
  bash "${SCRIPT_DIR}/bootstrap_marvin_install.sh"
fi

# Keep package launch in sync with tracked overlay (install/ is often gitignored).
OVERLAY_LAUNCH="${MARVIN_WS}/launch_overlay/start_teleop_m6_dual_gento.launch.py"
INSTALL_LAUNCH="${MARVIN_WS}/install/share/factr_teleop/launch/start_teleop_m6_dual_gento.launch.py"
if [[ -f "${OVERLAY_LAUNCH}" ]]; then
  mkdir -p "$(dirname "${INSTALL_LAUNCH}")"
  cp -f "${OVERLAY_LAUNCH}" "${INSTALL_LAUNCH}"
fi

# 右臂夹爪 Dynamixel 第 8 维必须与左臂同为 +1。-1 把松开扳机翻到负半轴 → 归一化 0。
# 左右臂 j4 min 对齐大臂 -1.0472（小臂原 -2.4 会把大臂顶到限位后猛追）。
FACTR_CFG="${MARVIN_WS}/install/share/factr_teleop/configs"
python3 - "${FACTR_CFG}" <<'PY'
import re, sys
from pathlib import Path

cfg = Path(sys.argv[1])

def patch_file(path: Path) -> None:
    if not path.is_file():
        return
    text = path.read_text()
    notes = []
    new, n = re.subn(
        r"(joint_signs:\s*\[[^\]]+),\s*-1(\s*\])",
        r"\1, 1\2",
        text,
        count=1,
    )
    if n:
        notes.append("gripper joint_signs[7] -1 -> +1")
        text = new

    def j4_min(m):
        parts = [p.strip() for p in m.group(1).split(",")]
        if len(parts) >= 4 and parts[3] in ("-2.4", "-2.40"):
            parts[3] = "-1.0"
            notes.append("j4 min -2.4 -> -1.0")
        return "arm_joint_limits_min: [" + ", ".join(parts) + "]"

    text, _ = re.subn(
        r"arm_joint_limits_min:\s*\[([^\]]+)\]",
        j4_min,
        text,
        count=1,
    )
    if notes:
        path.write_text(text)
        print(f"patched {path.name}: " + "; ".join(notes))

patch_file(cfg / "grav_comp_m6_left.yaml")
patch_file(cfg / "grav_comp_m6_right.yaml")
PY

DOCKER_ARGS=(
  --rm
  -it
  --net=host
  --ipc=host
  --privileged
  --group-add dialout
  --cap-add SYS_NICE
  --ulimit rtprio=99
  --ulimit memlock=-1
  -e "ROBOT_IP=${ROBOT_IP}"
  -e "ROBOT_GRIPPER_PORT_LEFT=${ROBOT_GRIPPER_PORT_LEFT}"
  -e "ROBOT_GRIPPER_PORT_RIGHT=${ROBOT_GRIPPER_PORT_RIGHT}"
  -e "USE_LEFT_GRIPPER=${USE_LEFT_GRIPPER}"
  -e "USE_RIGHT_GRIPPER=${USE_RIGHT_GRIPPER}"
  -e "ROBOT_LEADER_DYNAMIXEL_PORT_LEFT=${ROBOT_LEADER_DYNAMIXEL_PORT_LEFT}"
  -e "ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT=${ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT}"
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-20}"
  -e "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  -v /dev:/dev
  -v "${MARVIN_WS}:/marvin_ws"
  -v "${SCRIPT_DIR}:/scripts:ro"
  -w /marvin_ws
)

if [[ -n "${DISPLAY:-}" && -d /tmp/.X11-unix ]]; then
  DOCKER_ARGS+=(
    -e "DISPLAY=${DISPLAY}"
    -v /tmp/.X11-unix:/tmp/.X11-unix
  )
fi

echo "Mount: ${MARVIN_WS} -> /marvin_ws"
echo "Image: ${IMAGE}"
exec docker run "${DOCKER_ARGS[@]}" "${IMAGE}"
