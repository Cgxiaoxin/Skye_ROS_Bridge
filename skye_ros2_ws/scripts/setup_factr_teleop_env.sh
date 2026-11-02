#!/usr/bin/env bash
# Source in operator/recorder terminals. USB calibration is selected separately
# from the saved follower profile. Accepts --robot, --device, --side and --domain.

if [[ "${BASH_SOURCE[0]}" == "${0}" ]]; then
  echo "ERROR: source this script, do not execute it." >&2
  exit 1
fi

_SKYE_TELEOP_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
if ! _SKYE_TELEOP_ENV="$(/usr/bin/python3 "$_SKYE_TELEOP_ROOT/scripts/teleop.py" env "$@")"; then
  unset _SKYE_TELEOP_ROOT _SKYE_TELEOP_ENV
  return 1
fi
# Preserve the caller's nounset setting while loading ROS setup scripts.
_SKYE_TELEOP_NOUNSET="${-//[^u]/}"
set +u
_skye_factr_apply_env() { eval "$_SKYE_TELEOP_ENV"; }
if _skye_factr_apply_env; then
  _SKYE_TELEOP_RESULT=0
else
  _SKYE_TELEOP_RESULT=1
fi
unset -f _skye_factr_apply_env
[[ -z "$_SKYE_TELEOP_NOUNSET" ]] || set -u
unset _SKYE_TELEOP_ROOT _SKYE_TELEOP_ENV _SKYE_TELEOP_NOUNSET
if [[ "$_SKYE_TELEOP_RESULT" == 1 ]]; then
  unset _SKYE_TELEOP_RESULT
  return 1
fi
unset _SKYE_TELEOP_RESULT
# Keep the documented command, with synchronization feedback in this terminal.
# Other ros2 commands retain their standard behavior.
ros2() {
  if [[ $# -eq 6 && "$1" == topic && "$2" == pub && "$3" == --once &&
        "$4" == /mode/switch_sync && "$5" == std_msgs/msg/String ]]; then
    command /usr/bin/python3 -m skye_factr_teleop.sync_monitor "$@"
  else
    command ros2 "$@"
  fi
}
echo "skye_factr_teleop env ready: robot=$ROBOT_PROFILE domain=$ROS_DOMAIN_ID"
