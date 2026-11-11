#!/usr/bin/env bash
# Usage: leader_arm_gate.sh <left|right> <off|on>
# Soft gate for skye_factr_teleop leader nodes inside a dual launch:
# publish /mode/<side>/leader_gate so the live node mutes itself.
# Do not kill processes or relaunch outside the UI session process group.
set -euo pipefail

SIDE="${1:?left|right}"
ACTION="${2:?off|on}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PROFILE="${ROBOT_PROFILE:-thor}"
NODE="skye_factr_teleop_${SIDE}"
WS_SETUP="${REPO_ROOT}/skye_ros2_ws/install/setup.bash"
TELEOP_ENV="${REPO_ROOT}/skye_ros2_ws/scripts/setup_factr_teleop_env.sh"
GATE_TOPIC="/mode/${SIDE}/leader_gate"

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

if ! _source_ros; then
  exit 1
fi

if ! ros2 node list 2>/dev/null | grep -qx "/${NODE}"; then
  if [[ "$ACTION" == "off" ]]; then
    # Already absent — treat as gated off for UI bookkeeping.
    exit 0
  fi
  echo "missing /${NODE}; cannot enable soft gate (start dual teleop session first)" >&2
  exit 1
fi

# VOLATILE event QoS: repeat so the live node is likely to see at least one.
for _ in 1 2 3 4 5; do
  ros2 topic pub --once "${GATE_TOPIC}" std_msgs/msg/String "{data: ${ACTION}}" >/dev/null
done
