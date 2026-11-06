#!/usr/bin/env bash
# Original Thor wrapper (from ~/zph/run_robot_unijepa.sh) — REFERENCE ONLY.
# This still execs legacy run_robot.sh which writes CONTROLLER_IP.
# Do NOT use alongside skye_robot_driver / HITL arbiter.
set -euo pipefail

RUNTIME_DIR="${RUNTIME_DIR:-/home/user/Desktop/wzh/runtime-left}"

export INFERENCE_PORT="${INFERENCE_PORT:-5568}"
export CONTROLLER_IP="${CONTROLLER_IP:-6.6.7.191}"
export ACTION_HISTORY_HORIZON=30
export ACTION_STRIDE=1
export MODEL_ACTION_HZ=30
export ACTION_HZ=30
export POLICY_OBSERVATION_HZ=30
export MAX_PENDING_POLICY_REQUESTS=2
export RTC_MAX_PREFIX=0
export GRIPPER_CONTROL_MODE=direct
export ARM_COMMAND_STYLE=stream
export ACTION_AGGREGATE=weighted_average
export ACTION_EMA_WEIGHT=0.75
export RESIZE_ROS_IMAGES=0
export POLICY_JPEG_QUALITY="${POLICY_JPEG_QUALITY:-70}"
export SKIP_CAMERAS="${SKIP_CAMERAS:-0}"

echo "[WARN] Legacy path writes controller at ${CONTROLLER_IP}; conflicts with HITL DAgger." >&2
exec bash "${RUNTIME_DIR}/remote_inference_async/run_robot.sh"
