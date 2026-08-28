#!/usr/bin/env bash
# Inspect the live ROS graph and report pure-teleop vs HITL-DAgger wiring.
#
# Nodes live on every machine sharing ROS_DOMAIN_ID (host + Docker). This script
# does not distinguish host vs container; it classifies the *control path*.
#
# Usage:
#   export ROS_DOMAIN_ID=21
#   ./scripts/verify_runtime_path.sh
#
# Big-arm-only bench (no small arms): enough for driver + arbiter + policy chunk.
# Full HITL human takeover (q/w): needs FACTR in Docker on the HITL remap launch.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/ros_domain_env.sh"
skye_source_ros || exit 1

say() { printf '%s\n' "$*"; }
warn() { printf 'WARN: %s\n' "$*" >&2; }
fail() { printf 'FAIL: %s\n' "$*" >&2; exit 1; }

topic_publishers() {
  local topic="$1"
  ros2 topic info -v "${topic}" 2>/dev/null \
    | awk '/Publisher count:/{p=1; next} /Subscription count:/{p=0} p && /Node name:/{print $3}'
}

topic_subscribers() {
  local topic="$1"
  ros2 topic info -v "${topic}" 2>/dev/null \
    | awk '/Subscription count:/{s=1; next} s && /Node name:/{print $3} /Publisher count:/{exit}'
}

node_running() {
  local pattern="$1"
  ros2 node list 2>/dev/null | grep -qE "${pattern}"
}

has_publishers() {
  local topic="$1"
  [[ -n "$(topic_publishers "${topic}" 2>/dev/null || true)" ]]
}

say "== Skye runtime path check =="
say "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
say "FASTRTPS_DEFAULT_PROFILES_FILE=${FASTRTPS_DEFAULT_PROFILES_FILE}"
say ""

if ! ros2 node list >/dev/null 2>&1; then
  fail "ros2 node list failed (daemon down or ROS not sourced?)"
fi

NODES="$(ros2 node list 2>/dev/null || true)"
say "== Node roles (大臂 vs 小臂) =="
say "大臂 (host, Gento SDK):  /skye_robot_driver"
say "HITL 仲裁 (host):        /control_arbiter, /hitl_keyboard, /episode_recorder"
say "小臂主臂 (Docker/FACTR): /factr_teleop_left, /factr_teleop_right"
say ""
say "Visible nodes (filtered):"
if [[ -z "${NODES}" ]]; then
  say "  (none)"
else
  printf '%s\n' "${NODES}" | grep -E 'skye_robot_driver|control_arbiter|hitl_keyboard|episode_recorder|factr_teleop' \
    | sed 's/^/  /' || say "  (no skye/factr/hitl nodes)"
fi
say ""

HAS_DRIVER=0
HAS_ARBITER=0
HAS_FACTR=0
node_running '/skye_robot_driver' && HAS_DRIVER=1
node_running '/control_arbiter' && HAS_ARBITER=1
node_running '/factr_teleop_' && HAS_FACTR=1

say "== Topic publishers =="
for topic in \
  /gento/joint_states \
  /gento/left_joint_control \
  /gento/left_joint_control_abs \
  /skye/teleop_action_left \
  /skye/policy_action; do
  pubs="$(topic_publishers "${topic}" 2>/dev/null | tr '\n' ' ' | sed 's/ $//')"
  if [[ -z "${pubs}" ]]; then
    say "  ${topic}: (no publisher)"
  else
    say "  ${topic}: ${pubs}"
  fi
done
say ""

# --- classify path ---
PATH_LABEL="UNKNOWN"
PATH_DETAIL="No skye_robot_driver or HITL/FACTR nodes detected."
EXIT_CODE=2

if [[ "${HAS_DRIVER}" -eq 0 && "${HAS_ARBITER}" -eq 0 && "${HAS_FACTR}" -eq 0 ]]; then
  PATH_LABEL="IDLE"
  PATH_DETAIL="Nothing running on this ROS domain (or wrong ROS_DOMAIN_ID)."
elif [[ "${HAS_DRIVER}" -eq 1 && "${HAS_ARBITER}" -eq 0 && "${HAS_FACTR}" -eq 0 ]]; then
  PATH_LABEL="BIG_ARM_ONLY"
  PATH_DETAIL="Only skye_robot_driver — OK for driver/SDK checks and HITL policy bench (arbiter + dummy chunk)."
  EXIT_CODE=0
elif [[ "${HAS_DRIVER}" -eq 1 && "${HAS_ARBITER}" -eq 1 && "${HAS_FACTR}" -eq 0 ]]; then
  PATH_LABEL="HITL_HOST_ONLY"
  PATH_DETAIL="Driver + control_arbiter without FACTR — OK for policy rollout / abs joint bench; human q/w needs small arms."
  EXIT_CODE=0
elif [[ "${HAS_DRIVER}" -eq 1 && "${HAS_ARBITER}" -eq 0 && "${HAS_FACTR}" -eq 1 ]]; then
  if has_publishers /gento/left_joint_control \
     && grep -q factr <<<"$(topic_publishers /gento/left_joint_control)"; then
    PATH_LABEL="TELEOP_PURE"
    PATH_DETAIL="FACTR → /gento/* directly (HITL off). Expected for daily teleop."
    EXIT_CODE=0
  else
    PATH_LABEL="TELEOP_PARTIAL"
    PATH_DETAIL="FACTR running but /gento/left_joint_control publisher unclear — check remap launch."
    EXIT_CODE=2
  fi
elif [[ "${HAS_DRIVER}" -eq 1 && "${HAS_ARBITER}" -eq 1 && "${HAS_FACTR}" -eq 1 ]]; then
  factr_on_gento=0
  factr_on_skye=0
  grep -q factr <<<"$(topic_publishers /gento/left_joint_control 2>/dev/null || true)" && factr_on_gento=1
  grep -q factr <<<"$(topic_publishers /skye/teleop_action_left 2>/dev/null || true)" && factr_on_skye=1
  if [[ "${factr_on_gento}" -eq 1 && "${factr_on_skye}" -eq 0 ]]; then
    PATH_LABEL="CONFLICT"
    PATH_DETAIL="arbiter is up but FACTR still publishes /gento/* — use start_teleop_m6_dual_gento_hitl.launch.py in Docker."
    EXIT_CODE=1
  elif [[ "${factr_on_skye}" -eq 1 ]]; then
    PATH_LABEL="HITL_FULL"
    PATH_DETAIL="Full HITL-DAgger path (driver + arbiter + FACTR on /skye/teleop_*)."
    EXIT_CODE=0
  else
    PATH_LABEL="HITL_PARTIAL"
    PATH_DETAIL="arbiter + FACTR nodes present; check /skye/teleop_action_left publishers."
    EXIT_CODE=2
  fi
elif [[ "${HAS_ARBITER}" -eq 1 && "${HAS_DRIVER}" -eq 0 ]]; then
  PATH_LABEL="HITL_NO_DRIVER"
  PATH_DETAIL="control_arbiter without skye_robot_driver — start ./scripts/start_skye_for_factr.sh first."
  EXIT_CODE=1
elif [[ "${HAS_FACTR}" -eq 1 && "${HAS_DRIVER}" -eq 0 ]]; then
  PATH_LABEL="FACTR_NO_DRIVER"
  PATH_DETAIL="FACTR without skye_robot_driver — joint_states/commands will not reach the big arm."
  EXIT_CODE=1
else
  PATH_LABEL="MIXED"
  PATH_DETAIL="Unusual node combination — inspect publishers above."
  EXIT_CODE=2
fi

say "== Verdict =="
say "PATH=${PATH_LABEL}"
say "${PATH_DETAIL}"
say ""

say "== What you need connected =="
say "| Goal                         | 大臂 | 小臂 |"
say "|------------------------------|------|------|"
say "| verify_runtime_path (graph)  |  -   |  -   |  ROS only"
say "| Driver / joint_states        |  yes |  no  |"
say "| HITL policy (dummy/VLA chunk)|  yes |  no  |  + control_arbiter"
say "| HITL human takeover (q/w)    |  yes | yes  |  + FACTR hitl launch"
say "| Daily teleop (HITL off)      |  yes | yes  |  gento launch, no arbiter"
say ""

if [[ "${PATH_LABEL}" == "CONFLICT" || "${PATH_LABEL}" == "HITL_NO_DRIVER" ]]; then
  say "Fix hints:"
  if [[ "${PATH_LABEL}" == "CONFLICT" ]]; then
    say "  Docker: ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py"
    say "  Host:   ./scripts/start_hitl_host.sh"
  fi
  if [[ "${PATH_LABEL}" == "HITL_NO_DRIVER" ]]; then
    say "  Host:   ./scripts/start_skye_for_factr.sh"
  fi
fi

exit "${EXIT_CODE}"
