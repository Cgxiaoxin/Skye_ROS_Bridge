#!/usr/bin/env bash
# Usage: leader_arm_gate.sh <left|right> <off|on>
# Host-side gate for skye_factr_teleop leader nodes (no Marvin docker exec).
set -euo pipefail

SIDE="${1:?left|right}"
ACTION="${2:?off|on}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROFILE="${ROBOT_PROFILE:-thor}"
NODE="skye_factr_teleop_${SIDE}"
WS_SETUP="${REPO_ROOT}/skye_ros2_ws/install/setup.bash"
TELEOP_ENV="${REPO_ROOT}/skye_ros2_ws/scripts/setup_factr_teleop_env.sh"

case "$SIDE" in left|right) ;; *)
  echo "bad side: ${SIDE}" >&2
  exit 2
  ;;
esac
case "$ACTION" in off|on) ;; *)
  echo "bad action: ${ACTION}" >&2
  exit 2
  ;;
esac

_source_ros() {
  if [[ ! -f "$WS_SETUP" ]]; then
    echo "missing ROS workspace setup: ${WS_SETUP}" >&2
    return 1
  fi
  set +u
  # shellcheck disable=SC1090
  source "$WS_SETUP"
  if [[ -f "$TELEOP_ENV" ]]; then
    # shellcheck disable=SC1091
    source "$TELEOP_ENV" --robot "$PROFILE"
  else
    export ROBOT_PROFILE="$PROFILE"
    if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
      if [[ "$PROFILE" == "thor" ]]; then
        export ROS_DOMAIN_ID=20
      else
        export ROS_DOMAIN_ID=21
      fi
    fi
  fi
  set -u
  return 0
}

if [[ "$ACTION" == "off" ]]; then
  pkill -f "$NODE" || true
  sleep 1
  exec python3 "${SCRIPT_DIR}/disable_leader_dynamixel.py" --side "$SIDE"
fi

if ! _source_ros; then
  exit 1
fi

if ros2 node list 2>/dev/null | grep -qx "/${NODE}"; then
  exit 0
fi

nohup ros2 launch skye_factr_teleop "start_teleop_m6_${SIDE}.launch.py" \
  "robot_profile:=${PROFILE}" sync_diagnostics:=true \
  >"/tmp/${NODE}.log" 2>&1 &

deadline=$((SECONDS + 15))
until ros2 node list 2>/dev/null | grep -qx "/${NODE}"; do
  if (( SECONDS >= deadline )); then
    echo "failed to start /${NODE}; check /tmp/${NODE}.log" >&2
    exit 1
  fi
  sleep 0.2
done
