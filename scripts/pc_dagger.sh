#!/usr/bin/env bash
# PC-side HITL DAgger stack (driver + small-arm teleop + arbiter) in one process tree.
#
# Matches the local-PC workaround in docs/dagger_dev/流程梳理.md:
#   A) start_skye_for_factr.sh thor
#   B) ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py
#   C) ros2 launch skye_hitl_dagger hitl_dagger.launch.py
#
# Does NOT start Thor cameras / DAgger client / 5090 serve.
#
# Usage:
#   ./scripts/pc_dagger.sh                  # profile=thor (Domain 20)
#   ./scripts/pc_dagger.sh orin             # profile=orin (Domain 21 unless ROS_DOMAIN_ID set)
#   ./scripts/pc_dagger.sh --status
#   ./scripts/pc_dagger.sh --stop
#   ./scripts/pc_dagger.sh --no-teleop      # driver + arbiter only
#   ./scripts/pc_dagger.sh --arbiter-only   # reuse existing driver/teleop
#
# Env overrides: ROBOT_PROFILE, ROS_DOMAIN_ID, ROS_SETUP, CONNECT_ON_STARTUP,
# ENABLE_GRIPPER, ENABLE_RECORDER, GRIPPER_INVERT, PC_DAGGER_LOG_DIR

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS="${REPO_ROOT}/skye_ros2_ws"
# shellcheck source=lib/robot_profile.sh
source "${SCRIPT_DIR}/lib/robot_profile.sh"

START_DRIVER=1
START_TELEOP=1
START_ARBITER=1
DO_STOP=0
DO_STATUS=0
PROFILE_ARG=""

usage() {
  sed -n '2,22p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help) usage 0 ;;
    --stop) DO_STOP=1 ;;
    --status) DO_STATUS=1 ;;
    --no-teleop) START_TELEOP=0 ;;
    --arbiter-only|--no-driver)
      START_DRIVER=0
      START_TELEOP=0
      ;;
    --with-driver) START_DRIVER=1 ;;
    thor|orin)
      PROFILE_ARG="$1"
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage 1
      ;;
  esac
  shift
done

if [[ -n "${PROFILE_ARG}" ]]; then
  apply_robot_profile_arg "${PROFILE_ARG}"
fi
ROBOT_PROFILE="$(resolve_robot_profile "${REPO_ROOT}/marvin_ws")"
validate_robot_profile "${ROBOT_PROFILE}" || exit 1
export ROBOT_PROFILE

# shellcheck source=lib/ros_domain_env.sh
source "${SCRIPT_DIR}/lib/ros_domain_env.sh"

LOG_DIR="${PC_DAGGER_LOG_DIR:-/tmp/skye_pc_dagger}"
PID_DIR="${LOG_DIR}/pids"
mkdir -p "${LOG_DIR}" "${PID_DIR}"

DRIVER_PID_FILE="${PID_DIR}/driver.pid"
TELEOP_PID_FILE="${PID_DIR}/teleop.pid"
ARBITER_PID_FILE="${PID_DIR}/arbiter.pid"

count_procs() {
  local pattern="$1"
  pgrep -fc "${pattern}" 2>/dev/null || true
}

is_pid_alive() {
  local pid="$1"
  [[ -n "${pid}" ]] && kill -0 "${pid}" 2>/dev/null
}

read_pid_file() {
  local file="$1"
  if [[ -f "${file}" ]]; then
    tr -d '[:space:]' <"${file}" || true
  fi
}

print_status() {
  local name="$1" pattern="$2" pid_file="$3"
  local pid count
  pid="$(read_pid_file "${pid_file}")"
  count="$(count_procs "${pattern}")"
  if is_pid_alive "${pid}"; then
    echo "  ${name}: running pid=${pid} (tracked)"
  elif [[ "${count}" -gt 0 ]]; then
    echo "  ${name}: running (untracked matches=${count})"
  else
    echo "  ${name}: stopped"
  fi
}

stop_tracked() {
  local name="$1" pid_file="$2" pattern="$3"
  local pid
  pid="$(read_pid_file "${pid_file}")"
  if is_pid_alive "${pid}"; then
    echo "== stopping ${name} (pid ${pid}) =="
    kill "${pid}" 2>/dev/null || true
    for _ in 1 2 3 4 5; do
      is_pid_alive "${pid}" || break
      sleep 0.4
    done
    if is_pid_alive "${pid}"; then
      kill -9 "${pid}" 2>/dev/null || true
    fi
  fi
  rm -f "${pid_file}"
  if [[ "$(count_procs "${pattern}")" -gt 0 ]]; then
    echo "== pkill leftover ${name} =="
    pkill -f "${pattern}" 2>/dev/null || true
  fi
}

if [[ "${DO_STATUS}" -eq 1 ]]; then
  echo "== pc_dagger status profile=${ROBOT_PROFILE} DOMAIN=${ROS_DOMAIN_ID} =="
  print_status "driver" 'lib/skye_robot_driver/skye_robot_driver' "${DRIVER_PID_FILE}"
  print_status "teleop" 'start_teleop_m6_dual_hitl.launch|skye_factr_teleop' "${TELEOP_PID_FILE}"
  print_status "arbiter" 'hitl_dagger.launch|lib/skye_hitl_dagger/control_arbiter' "${ARBITER_PID_FILE}"
  echo "logs: ${LOG_DIR}"
  exit 0
fi

if [[ "${DO_STOP}" -eq 1 ]]; then
  stop_tracked "arbiter" "${ARBITER_PID_FILE}" \
    'hitl_dagger.launch.py|lib/skye_hitl_dagger/control_arbiter|skye_hitl_dagger control_arbiter|skye_hitl_dagger hitl_keyboard'
  stop_tracked "teleop" "${TELEOP_PID_FILE}" \
    'start_teleop_m6_dual_hitl.launch'
  stop_tracked "driver" "${DRIVER_PID_FILE}" \
    'lib/skye_robot_driver/skye_robot_driver'
  echo "== pc_dagger stopped =="
  exit 0
fi

if pgrep -x gento_robot_driver >/dev/null 2>&1; then
  echo "ERROR: gento_robot_driver is running. Stop it first (only one SDK client)." >&2
  exit 1
fi

if [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "== building skye_ros2_ws =="
  bash "${WS}/scripts/build.sh"
fi
skye_source_ros || exit 1

echo "== pc_dagger profile=${ROBOT_PROFILE} ROS_DOMAIN_ID=${ROS_DOMAIN_ID} =="
echo "   FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE:-<unset>}"
echo "   logs → ${LOG_DIR}"
echo "   stop → ./scripts/pc_dagger.sh --stop"
echo ""

PIDS_TO_WAIT=()

cleanup() {
  local code=$?
  trap - EXIT INT TERM
  echo ""
  echo "== pc_dagger shutting down =="
  stop_tracked "arbiter" "${ARBITER_PID_FILE}" \
    'hitl_dagger.launch.py|lib/skye_hitl_dagger/control_arbiter|skye_hitl_dagger control_arbiter|skye_hitl_dagger hitl_keyboard'
  if [[ "${START_TELEOP}" -eq 1 ]]; then
    stop_tracked "teleop" "${TELEOP_PID_FILE}" 'start_teleop_m6_dual_hitl.launch'
  fi
  if [[ "${START_DRIVER}" -eq 1 ]]; then
    stop_tracked "driver" "${DRIVER_PID_FILE}" 'lib/skye_robot_driver/skye_robot_driver'
  fi
  exit "${code}"
}
trap cleanup EXIT INT TERM

start_bg() {
  local name="$1" pid_file="$2" log_file="$3"
  shift 3
  echo "== starting ${name} =="
  # Keep children in their own process group so Ctrl+C on this script cleans via trap.
  set +e
  "$@" >"${log_file}" 2>&1 &
  local pid=$!
  set -e
  echo "${pid}" >"${pid_file}"
  PIDS_TO_WAIT+=("${pid}")
  sleep 2
  if ! is_pid_alive "${pid}"; then
    echo "ERROR: ${name} exited early; see ${log_file}" >&2
    tail -n 40 "${log_file}" >&2 || true
    exit 1
  fi
  echo "   ${name} pid=${pid} log=${log_file}"
}

if [[ "${START_DRIVER}" -eq 1 ]]; then
  if [[ "$(count_procs 'lib/skye_robot_driver/skye_robot_driver')" -gt 0 ]]; then
    echo "== skye_robot_driver already running; reusing =="
  else
    start_bg "driver" "${DRIVER_PID_FILE}" "${LOG_DIR}/driver.log" \
      ros2 launch skye_robot_driver skye_robot_driver.launch.py \
      connect_on_startup:="${CONNECT_ON_STARTUP:-true}" \
      enable_gripper:="${ENABLE_GRIPPER:-true}" \
      robot_profile:="${ROBOT_PROFILE}"
  fi
else
  if [[ "$(count_procs 'lib/skye_robot_driver/skye_robot_driver')" -eq 0 ]]; then
    echo "WARN: skye_robot_driver not detected; big arm will not move." >&2
  fi
fi

if [[ "${START_TELEOP}" -eq 1 ]]; then
  if [[ "$(count_procs 'start_teleop_m6_dual_hitl.launch')" -gt 0 ]]; then
    echo "== HITL teleop already running; reusing =="
  else
    start_bg "teleop" "${TELEOP_PID_FILE}" "${LOG_DIR}/teleop.log" \
      ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py \
      robot_profile:="${ROBOT_PROFILE}"
  fi
fi

if [[ "${START_ARBITER}" -eq 1 ]]; then
  if [[ "$(count_procs 'lib/skye_hitl_dagger/control_arbiter|hitl_dagger.launch')" -gt 0 ]]; then
    echo "ERROR: control_arbiter / hitl_dagger already running." >&2
    echo "  ./scripts/pc_dagger.sh --stop   # then retry" >&2
    exit 1
  fi
  start_bg "arbiter" "${ARBITER_PID_FILE}" "${LOG_DIR}/arbiter.log" \
    ros2 launch skye_hitl_dagger hitl_dagger.launch.py \
    robot_profile:="${ROBOT_PROFILE}" \
    gripper_invert_on_driver:="${GRIPPER_INVERT:-true}" \
    enable_recorder:="${ENABLE_RECORDER:-false}"
fi

echo ""
echo "== stack up. Useful checks (another terminal, same Domain) =="
echo "  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROBOT_PROFILE=${ROBOT_PROFILE}"
echo "  unset ROS_LOCALHOST_ONLY; export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  source ${WS}/install/setup.bash"
echo "  ros2 topic hz /gento/joint_states"
echo "  ros2 topic echo /teleop/state --once"
echo "  tail -f ${LOG_DIR}/{driver,teleop,arbiter}.log"
echo ""
echo "Thor still needs cameras + run_client_dagger.sh; 5090 needs serve."
echo "Ctrl+C here stops the processes this script started."
echo ""

# Block until any tracked child exits, then cleanup.
while true; do
  alive=0
  for pid in "${PIDS_TO_WAIT[@]:-}"; do
    if is_pid_alive "${pid}"; then
      alive=1
      break
    fi
  done
  if [[ "${alive}" -eq 0 ]]; then
    echo "ERROR: all tracked processes exited; check ${LOG_DIR}" >&2
    exit 1
  fi
  sleep 1
done
