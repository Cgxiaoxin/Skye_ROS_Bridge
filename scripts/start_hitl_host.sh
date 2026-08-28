#!/usr/bin/env bash
# Host-side HITL stack: skye_robot_driver (optional) + hitl_dagger launch.
# FACTR small arms still run in Docker — see printed command after start.
#
# Usage:
#   ./scripts/start_hitl_host.sh                    # driver + arbiter (default)
#   ./scripts/start_hitl_host.sh --arbiter-only     # arbiter only (driver already up)
#   ./scripts/start_hitl_host.sh --cleanup-stale    # kill zombie arbiter/recorder first
#   ./scripts/start_hitl_host.sh --no-driver        # same as --arbiter-only
#
# Keyboard: q=takeover, w=return (this terminal must stay foreground for hitl_keyboard).
#
# Big-arm-only bench: skip Docker; publish dummy policy:
#   ros2 run skye_hitl_dagger pub_dummy_policy_chunk

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS="${REPO_ROOT}/skye_ros2_ws"

START_DRIVER=1
CLEANUP_STALE=0
ENABLE_RECORDER="${ENABLE_RECORDER:-false}"
GRIPPER_INVERT="${GRIPPER_INVERT:-true}"

usage() {
  sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
  exit "${1:-0}"
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --arbiter-only|--no-driver) START_DRIVER=0 ;;
    --with-driver) START_DRIVER=1 ;;
    --cleanup-stale) CLEANUP_STALE=1 ;;
    --enable-recorder) ENABLE_RECORDER=true ;;
    --gripper-invert=*) GRIPPER_INVERT="${1#*=}" ;;
    -h|--help) usage 0 ;;
    *) echo "Unknown option: $1" >&2; usage 1 ;;
  esac
  shift
done

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/ros_domain_env.sh"

count_procs() {
  local pattern="$1"
  pgrep -fc "${pattern}" 2>/dev/null || true
}

cleanup_stale_hitl() {
  local arb rec
  arb="$(count_procs 'lib/skye_hitl_dagger/control_arbiter|skye_hitl_dagger control_arbiter')"
  rec="$(count_procs 'lib/skye_hitl_dagger/episode_recorder|skye_hitl_dagger episode_recorder')"
  if [[ "${arb}" -eq 0 && "${rec}" -eq 0 ]]; then
    return 0
  fi
  echo "== cleaning stale HITL processes (arbiter=${arb}, recorder=${rec}) =="
  pkill -f 'lib/skye_hitl_dagger/control_arbiter' 2>/dev/null || true
  pkill -f 'skye_hitl_dagger control_arbiter' 2>/dev/null || true
  pkill -f 'lib/skye_hitl_dagger/episode_recorder' 2>/dev/null || true
  pkill -f 'skye_hitl_dagger episode_recorder' 2>/dev/null || true
  pkill -f 'lib/skye_hitl_dagger/hitl_keyboard' 2>/dev/null || true
  pkill -f 'skye_hitl_dagger hitl_keyboard' 2>/dev/null || true
  sleep 1
}

if [[ "${CLEANUP_STALE}" -eq 1 ]]; then
  cleanup_stale_hitl
fi

if pgrep -x gento_robot_driver >/dev/null 2>&1; then
  echo "ERROR: gento_robot_driver is running. Stop it first (only one SDK client)." >&2
  exit 1
fi

if [[ "$(count_procs 'lib/skye_hitl_dagger/control_arbiter|skye_hitl_dagger control_arbiter')" -gt 0 ]]; then
  echo "ERROR: control_arbiter already running." >&2
  echo "  Re-run with --cleanup-stale to kill zombies, or stop the existing session." >&2
  exit 1
fi

DRIVER_STARTED=0
DRIVER_PID=""

stop_started_driver() {
  if [[ "${DRIVER_STARTED}" -eq 1 && -n "${DRIVER_PID}" ]]; then
    echo "== stopping skye_robot_driver (pid ${DRIVER_PID}) =="
    kill "${DRIVER_PID}" 2>/dev/null || true
    wait "${DRIVER_PID}" 2>/dev/null || true
  fi
}

on_exit() {
  local code=$?
  stop_started_driver
  exit "${code}"
}
trap on_exit EXIT INT TERM

if [[ "${START_DRIVER}" -eq 1 ]]; then
  if pgrep -f 'lib/skye_robot_driver/skye_robot_driver' >/dev/null 2>&1; then
    echo "== skye_robot_driver already running; reusing =="
  else
    if [[ ! -f "${WS}/install/setup.bash" ]]; then
      echo "== building skye_ros2_ws =="
      bash "${WS}/scripts/build.sh"
    fi
    skye_source_ros || exit 1
    echo "== starting skye_robot_driver in background =="
    ros2 launch skye_robot_driver skye_robot_driver.launch.py \
      connect_on_startup:="${CONNECT_ON_STARTUP:-true}" &
    DRIVER_PID=$!
    DRIVER_STARTED=1
    sleep 3
    if ! kill -0 "${DRIVER_PID}" 2>/dev/null; then
      echo "ERROR: skye_robot_driver exited early." >&2
      exit 1
    fi
  fi
else
  if ! pgrep -f 'lib/skye_robot_driver/skye_robot_driver' >/dev/null 2>&1; then
    echo "WARN: skye_robot_driver not detected; big arm will not move until it is started." >&2
  fi
fi

if [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "== building skye_hitl_dagger (skye_ros2_ws) =="
  bash "${WS}/scripts/build.sh"
fi
skye_source_ros || exit 1

echo ""
echo "== HITL host stack ROS_DOMAIN_ID=${ROS_DOMAIN_ID} =="
echo "   FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
echo ""
echo "Docker (small arms) — run in a separate terminal after entering the container:"
echo "  source /marvin_ws/install/setup.bash"
echo "  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ROS_LOCALHOST_ONLY=0"
echo "  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}"
echo "  export FASTRTPS_DEFAULT_PROFILES_FILE=/marvin_ws/fastrtps_no_shm.xml"
echo "  ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py use_keyboard:=false"
echo ""
echo "Verify graph:  ./scripts/verify_runtime_path.sh"
echo "Big-arm-only bench:  ros2 run skye_hitl_dagger pub_dummy_policy_chunk"
echo "  (policy uses /gento/*_joint_control relative mapping; needs /gento/joint_states)"
echo ""

# Foreground: hitl_keyboard needs this terminal.
trap - EXIT
stop_started_driver() { :; }

exec ros2 launch skye_hitl_dagger hitl_dagger.launch.py \
  gripper_invert_on_driver:="${GRIPPER_INVERT}" \
  enable_recorder:="${ENABLE_RECORDER}"
