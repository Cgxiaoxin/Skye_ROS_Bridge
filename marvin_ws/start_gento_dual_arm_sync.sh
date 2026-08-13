#!/usr/bin/env bash
# Start the two leader (small) arms and the Gento dual-arm driver for factr teleop.
#
# ROS contract (bridge-less):
#   /gento/joint_states            sensor_msgs/msg/JointState, 14 positions in rad
#   /gento/left_joint_control      factr left /joint_control remap target
#   /gento/right_joint_control     factr right /joint_control remap target
#   /mode/switch_{sync,teleop,stop} from keyboard_gripper (keys 1/2/3)
#   /gento/hold_current, /gento/stop_motion  std_srvs/srv/Trigger
#
# Formal teleop: press keyboard 1 (sync) then 2 (teleop). Do not rely on
# /enable_position_sync unless GENTO_AUTO_SYNC=1 (legacy demo).
#
# The small-arm controller already runs at 500 Hz.  This script starts the
# Gento state publisher at 500 Hz as well.  It never publishes to the Orin
# legacy /left_joint_state or /right_joint_state topics.
# skye_leader_bridge is NOT used for Gento teleop.

set -Eeuo pipefail

readonly SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
readonly ROS_DOMAIN_ID=20
readonly ROBOT_IP="6.6.7.190"
readonly LEADER_CONTAINER="gento_leader_teleop"
readonly GENTO_WS="${SCRIPT_DIR}/gento_ros2_ws"
readonly GENTO_PARAMS="${GENTO_WS}/install/gento_robot_driver/share/gento_robot_driver/config/gento_robot.yaml"
readonly RUNTIME_DIR="${SCRIPT_DIR}/.runtime/gento_dual_arm_sync"
readonly DRIVER_LOG="${RUNTIME_DIR}/gento_robot_driver.log"
readonly DRIVER_PID_FILE="${RUNTIME_DIR}/gento_robot_driver.pid"
# Match grav_comp_m6_{left,right}.yaml dynamixel_port basenames.
readonly LEADER_FTDI_BY_ID=(
  "usb-FTDI_USB__-__Serial_Converter_FTB8HNOT-if00-port0"
  "usb-FTDI_USB__-__Serial_Converter_FTAO51EA-if00-port0"
)

say() { printf '\n==> %s\n' "$*"; }
fail() { printf '\nERROR: %s\n' "$*" >&2; exit 1; }

# ROS/ament setup scripts reference optional unset vars (e.g. AMENT_TRACE_SETUP_FILES).
# Under `set -u` that aborts; temporarily relax nounset while sourcing.
source_ros() {
  set +u
  # shellcheck disable=SC1090,SC1091
  source "$1"
  set -u
}

docker_sudo() { sudo docker "$@"; }

# factr_teleop refuses to start unless FTDI latency_timer is 1 (low USB latency).
ensure_ftdi_latency_timer() {
  local by_id_name="$1"
  local link="/dev/serial/by-id/${by_id_name}"
  local tty timer current
  [[ -e "${link}" ]] || fail "FTDI port missing: ${link}. Plug in both small-arm USB adapters first."
  tty="$(basename "$(readlink -f "${link}")")"
  timer="/sys/bus/usb-serial/devices/${tty}/latency_timer"
  [[ -f "${timer}" ]] || fail "Cannot find latency_timer for ${tty} (${link})."
  current="$(<"${timer}")"
  if [[ "${current}" != "1" ]]; then
    say "Setting ${tty} latency_timer ${current} -> 1"
    echo 1 | sudo tee "${timer}" >/dev/null
  else
    printf '    %s latency_timer already 1\n' "${tty}"
  fi
}

cleanup_failed_driver() {
  if [[ -f "${DRIVER_PID_FILE}" ]]; then
    local pid
    pid="$(<"${DRIVER_PID_FILE}")"
    if kill -0 "${pid}" 2>/dev/null; then
      kill "${pid}" 2>/dev/null || true
    fi
  fi
}

require_file() {
  [[ -f "$1" ]] || fail "Required file is missing: $1"
}

for tool in docker ros2 timeout; do
  command -v "${tool}" >/dev/null 2>&1 || fail "Required command is unavailable: ${tool}"
done
require_file "${GENTO_PARAMS}"
require_file "${GENTO_WS}/install/setup.bash"

mkdir -p "${RUNTIME_DIR}"

if [[ -f "${DRIVER_PID_FILE}" ]]; then
  existing_pid="$(<"${DRIVER_PID_FILE}")"
  if kill -0 "${existing_pid}" 2>/dev/null; then
    fail "An earlier Gento driver started by this script is still running (PID ${existing_pid}). Stop it before starting another SDK connection."
  fi
  rm -f "${DRIVER_PID_FILE}"
fi

# Match the real driver binary only. Do not use `pgrep -f gento_robot_driver`:
# Cursor/agent shells often keep that substring in long colcon command lines and
# would falsely block startup.
if pgrep -x gento_robot_driver >/dev/null 2>&1; then
  pgrep -ax gento_robot_driver >&2 || true
  fail "Another gento_robot_driver process already exists. Only one Gento SDK client may connect at a time."
fi

if [[ "${1:-}" != "--yes" ]]; then
  cat <<'WARNING'

This will restart the two small-arm controller processes and then connect to
the Gento controller.  Formal teleop uses keyboard 1/2/3 (sync/teleop/stop).
Confirm the area is clear before continuing.
WARNING
  read -r -p "Press Enter to continue (Ctrl+C to cancel): " confirmation
  [[ -z "${confirmation}" ]] || fail "Cancelled; press Enter without entering text to continue."
fi

say "1/4 Restarting both small-arm nodes (ROS_DOMAIN_ID=${ROS_DOMAIN_ID})"
for ftdi in "${LEADER_FTDI_BY_ID[@]}"; do
  ensure_ftdi_latency_timer "${ftdi}"
done
docker_sudo rm -f "${LEADER_CONTAINER}" >/dev/null 2>&1 || true

# Both small-arm nodes receive the same complete 14-axis state vector.  The
# left/right follower_joint_offset values in their own YAML select axes 0..6
# and 7..13 respectively.  /joint_control remaps to /gento/* so the driver
# applies mapping/rate-limit without skye_leader_bridge.
leader_command='set -Eeuo pipefail
unset ROS_LOCALHOST_ONLY
export ROS_DOMAIN_ID=20
set +u
source /opt/ros/humble/setup.bash
source /marvin_ws/install/setup.bash
set -u

ros2 run factr_teleop factr_teleop_robot_driver.py --ros-args \
  -r __node:=factr_teleop_left \
  -p config_file:=/marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_left.yaml \
  -p debug_state_print:=true -p print_joint_states:=true \
  -p print_period:=0.5 -p cb_print_period:=1.0 \
  -r /joint_control:=/gento/left_joint_control \
  -r /joint_state:=/gento/joint_states \
  -r /joint_move:=/left_joint_move \
  -r /gripper/ctrl:=/left_teleop_gripper/ctrl \
  -r /gripper/state:=/left_gripper/state &
left_pid=$!

ros2 run factr_teleop factr_teleop_robot_driver.py --ros-args \
  -r __node:=factr_teleop_right \
  -p config_file:=/marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_right.yaml \
  -p debug_state_print:=true -p print_joint_states:=true \
  -p print_period:=0.5 -p cb_print_period:=1.0 \
  -r /joint_control:=/gento/right_joint_control \
  -r /joint_state:=/gento/joint_states \
  -r /joint_move:=/right_joint_move \
  -r /gripper/ctrl:=/right_teleop_gripper/ctrl \
  -r /gripper/state:=/right_gripper/state &
right_pid=$!

ros2 run factr_teleop keyboard_gripper.py --ros-args -r __node:=keyboard_gripper &
kb_pid=$!

trap "kill ${left_pid} ${right_pid} ${kb_pid} 2>/dev/null || true" EXIT INT TERM
wait -n "${left_pid}" "${right_pid}" "${kb_pid}"
exit 1'

docker_sudo run -d --name "${LEADER_CONTAINER}" \
  --net=host --ipc=host --privileged --group-add dialout \
  --cap-add SYS_NICE --ulimit rtprio=99 --ulimit memlock=-1 \
  -e "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}" \
  -v /dev:/dev -v "${SCRIPT_DIR}:/marvin_ws" -w /marvin_ws \
  harbor.amigos-robot.com/tmp/marvin-m6-ros2:humble \
  bash -lc "${leader_command}" >/dev/null

for _ in $(seq 1 20); do
  if docker_sudo exec "${LEADER_CONTAINER}" bash -lc '
      export ROS_DOMAIN_ID=20
      set +u
      source /opt/ros/humble/setup.bash
      source /marvin_ws/install/setup.bash
      set -u
      ros2 node list | grep -qx /factr_teleop_left && ros2 node list | grep -qx /factr_teleop_right
    ' >/dev/null 2>&1; then
    break
  fi
  sleep 0.5
done

if ! docker_sudo exec "${LEADER_CONTAINER}" bash -lc '
    export ROS_DOMAIN_ID=20
    set +u
    source /opt/ros/humble/setup.bash
    source /marvin_ws/install/setup.bash
    set -u
    ros2 node list | grep -qx /factr_teleop_left && ros2 node list | grep -qx /factr_teleop_right
  ' >/dev/null 2>&1; then
  docker_sudo logs --tail 160 "${LEADER_CONTAINER}" >&2 || true
  fail "Both small-arm nodes did not remain online. Do not start Gento motion; inspect the Dynamixel diagnostics above."
fi

say "Small-arm nodes online: /factr_teleop_left and /factr_teleop_right"

say "2/4 Starting Gento driver: ${ROBOT_IP}, state publisher 500 Hz"
export ROS_DOMAIN_ID
unset ROS_LOCALHOST_ONLY
source_ros /opt/ros/humble/setup.bash
source_ros "${GENTO_WS}/install/setup.bash"

: >"${DRIVER_LOG}"
nohup ros2 run gento_robot_driver gento_robot_driver --ros-args \
  --params-file "${GENTO_PARAMS}" \
  -p state_publish_hz:=500.0 \
  -r /left_joint_control:=/gento/left_joint_control \
  -r /right_joint_control:=/gento/right_joint_control \
  -r /joint_states:=/gento/joint_states \
  -r /hold_current:=/gento/hold_current \
  -r /stop_motion:=/gento/stop_motion \
  >>"${DRIVER_LOG}" 2>&1 &
driver_pid=$!
printf '%s\n' "${driver_pid}" >"${DRIVER_PID_FILE}"

connected=false
for _ in $(seq 1 20); do
  if grep -q "Connected to Gento controller ${ROBOT_IP}" "${DRIVER_LOG}"; then
    connected=true
    break
  fi
  if ! kill -0 "${driver_pid}" 2>/dev/null; then
    tail -n 160 "${DRIVER_LOG}" >&2 || true
    rm -f "${DRIVER_PID_FILE}"
    fail "Gento driver exited before connecting. It may be held by another SDK client."
  fi
  sleep 0.5
done

if [[ "${connected}" != true ]]; then
  tail -n 160 "${DRIVER_LOG}" >&2 || true
  cleanup_failed_driver
  rm -f "${DRIVER_PID_FILE}"
  fail "Gento connection did not complete within 10 seconds."
fi

say "Gento driver connected; waiting for the 14-axis radian state stream"
if ! timeout 8s ros2 topic echo --once /gento/joint_states >"${RUNTIME_DIR}/first_joint_state.yaml"; then
  cleanup_failed_driver
  rm -f "${DRIVER_PID_FILE}"
  fail "No /gento/joint_states message was received."
fi

say "3/4 ROS graph (Gento publisher and small-arm subscribers)"
ros2 topic info -v /gento/joint_states
printf '\nFirst Gento state message (positions are rad):\n'
sed -n '1,100p' "${RUNTIME_DIR}/first_joint_state.yaml"
printf '\nObserved state-publish frequency:\n'
timeout 4s ros2 topic hz --window 500 /gento/joint_states || true

say "4/4 Starting keyboard teleop control (1=sync, 2=teleop, 3=stop)"
# Formal teleop: operator presses keyboard 1 after arms are up.
# Legacy auto position-sync demo: GENTO_AUTO_SYNC=1
if [[ "${GENTO_AUTO_SYNC:-0}" == "1" ]]; then
  printf '%s\n' 'GENTO_AUTO_SYNC=1: publishing /enable_position_sync true (legacy demo)'
  ros2 topic pub --once /enable_position_sync std_msgs/msg/Bool "{data: true}"
else
  printf '%s\n' 'Publishing /enable_position_sync false; use keyboard 1 for sync, 2 for teleop, 3 to stop'
  ros2 topic pub --once /enable_position_sync std_msgs/msg/Bool "{data: false}"
fi
sleep 1

printf '\nSmall-arm container log after teleop setup:\n'
docker_sudo logs --tail 120 "${LEADER_CONTAINER}" || true
printf '\nGento driver log:\n'
tail -n 80 "${DRIVER_LOG}" || true

cat <<EOF

Started successfully.  Runtime files:
  Gento log:    ${DRIVER_LOG}
  Gento PID:    ${DRIVER_PID_FILE}
  First state:  ${RUNTIME_DIR}/first_joint_state.yaml

Keyboard (inside ${LEADER_CONTAINER} /keyboard_gripper):
  1  sync small arms to large-arm posture (factr TELEOP_SYNCING → SYNCED)
  2  teleop (factr TELEOP → /gento/{left,right}_joint_control)
  3  stop teleop output

Driver Trigger services:
  ros2 service call /gento/hold_current std_srvs/srv/Trigger "{}"
  ros2 service call /gento/stop_motion std_srvs/srv/Trigger "{}"

Legacy debug: auto-enable position sync with GENTO_AUTO_SYNC=1 on next start, or:
  export ROS_DOMAIN_ID=20
  set +u; source /opt/ros/humble/setup.bash; set -u
  ros2 topic pub --once /enable_position_sync std_msgs/msg/Bool "{data: true}"

To stop the Gento driver later:
  kill $(cat "${DRIVER_PID_FILE}")
EOF
