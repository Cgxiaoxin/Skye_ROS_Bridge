#!/usr/bin/env bash
# DAgger client entry on Thor (pre-P0 client + current pi05_history serve).
# Vendor restored to yesterday baseline; only inference env aligned to 5090.
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VENDOR="${ROOT}/vendor/remote_inference_async"
SKYE_WS="${SKYE_WS:-/data/Skye_ROS_Bridge/skye_ros2_ws}"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
unset ROS_LOCALHOST_ONLY

# 5090 wzh serve_policy.sh pi05_history
export INFERENCE_HOST="${INFERENCE_HOST:-192.168.201.253}"
export INFERENCE_PORT="${INFERENCE_PORT:-5569}"
# CRITICAL: do not stream to Gento when HITL arbiter owns the arm.
export DAGGER_MODE=1
export DISABLE_CONTROLLER_STREAM=1
export DAGGER_POLICY_VERSION="${DAGGER_POLICY_VERSION:-pi05_history}"
unset CONTROLLER_IP

# pi05 history8 checkpoint (was 30 for UniJEPA)
export ACTION_HISTORY_HORIZON="${ACTION_HISTORY_HORIZON:-8}"
export MODEL_ACTION_HZ="${MODEL_ACTION_HZ:-30}"
export ACTION_HZ="${ACTION_HZ:-30}"
export POLICY_OBSERVATION_HZ="${POLICY_OBSERVATION_HZ:-30}"
export POLICY_JPEG_QUALITY="${POLICY_JPEG_QUALITY:-70}"
export RTC_MAX_PREFIX="${RTC_MAX_PREFIX:-0}"
export SKIP_CAMERAS="${SKIP_CAMERAS:-1}"   # cameras via ensure_cameras.sh

if [[ ! -d "${VENDOR}" ]]; then
  echo "[ERROR] Missing ${VENDOR}" >&2
  echo "[ERROR] Copy Thor remote_inference_async/ into client_thor/vendor/ — see NEED_FROM_YOU.md" >&2
  exit 2
fi

if [[ -f "${SKYE_WS}/install/setup.bash" ]]; then
  set +u
  # shellcheck disable=SC1091
  source "${SKYE_WS}/install/setup.bash"
  set -u
else
  echo "[WARN] ${SKYE_WS}/install/setup.bash not found; PolicyActionChunk import may fail" >&2
fi

echo "[INFO] DOMAIN=${ROS_DOMAIN_ID} INFERENCE=${INFERENCE_HOST}:${INFERENCE_PORT} policy=${DAGGER_POLICY_VERSION}"
echo "[INFO] history=${ACTION_HISTORY_HORIZON} hz=${ACTION_HZ} RTC_MAX_PREFIX=${RTC_MAX_PREFIX} (pre-P0 client)"
echo "[INFO] Publishes PolicyActionChunk to /skye/policy_action (no CONTROLLER write)."

if [[ -x "${VENDOR}/run_robot_dagger.sh" ]]; then
  exec bash "${VENDOR}/run_robot_dagger.sh"
fi

echo "[ERROR] Missing ${VENDOR}/run_robot_dagger.sh" >&2
exit 2
