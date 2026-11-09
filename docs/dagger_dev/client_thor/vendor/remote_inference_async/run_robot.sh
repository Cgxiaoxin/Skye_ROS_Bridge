#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/.." && pwd)"

readonly inference_server_ip="192.168.201.253"
inference_server_port="${INFERENCE_PORT:-5556}"
server_endpoint="tcp://${inference_server_ip}:${inference_server_port}"
controller_ip="${CONTROLLER_IP:-6.6.7.191}"
ros_setup="${ROS_SETUP:-/opt/kernelmind/apex/install/setup.bash}"
camera_ros_setup="${CAMERA_ROS_SETUP:-/home/user/tianji_capture_camera_ws/install/setup.bash}"
head_topic="${HEAD_COLOR_TOPIC:-/head_d435/camera/color/image_raw}"
left_wrist_topic="${WRIST_LEFT_TOPIC:-/left_wrist_d405/camera/color/image_raw}"
right_wrist_topic="${WRIST_RIGHT_TOPIC:-/right_wrist_d405/camera/color/image_raw}"
ros_camera_reliability="${ROS_CAMERA_RELIABILITY:-reliable}"
ros_camera_qos_depth="${ROS_CAMERA_QOS_DEPTH:-10}"
python_bin="${PYTHON_BIN:-python3}"
gripper_control_mode="${GRIPPER_CONTROL_MODE:-direct}"
action_stride="${ACTION_STRIDE:-1}"
model_action_hz="${MODEL_ACTION_HZ:-30}"
action_hz="${ACTION_HZ:-30}"
arm_command_style="${ARM_COMMAND_STYLE:-stream}"
observation_hz="${POLICY_OBSERVATION_HZ:-30}"
max_pending_policy_requests="${MAX_PENDING_POLICY_REQUESTS:-2}"
action_history_horizon="${ACTION_HISTORY_HORIZON:-30}"
rtc_action_horizon="${RTC_ACTION_HORIZON:-50}"
rtc_max_prefix="${RTC_MAX_PREFIX:-0}"
impedance_velocity_percent="${IMPEDANCE_VELOCITY_PERCENT:-100}"
impedance_acceleration_percent="${IMPEDANCE_ACCELERATION_PERCENT:-100}"
initial_position_velocity_percent="${INITIAL_POSITION_VELOCITY_PERCENT:-40}"
initial_position_acceleration_percent="${INITIAL_POSITION_ACCELERATION_PERCENT:-40}"
initial_move_step_deg="${INITIAL_MOVE_STEP_DEG:-0.5}"
initial_move_max_lead_deg="${INITIAL_MOVE_MAX_LEAD_DEG:-5.0}"
start_paused="${START_PAUSED:-0}"
runtime_control_socket="${RUNTIME_CONTROL_SOCKET:-/tmp/gento_robot_runtime.sock}"

case "${start_paused}" in
    0|1)
        ;;
    *)
        echo "[ERROR] START_PAUSED must be 0 or 1, got: ${start_paused}" >&2
        exit 2
        ;;
esac

case "${arm_command_style}" in
    ptp|stream)
        ;;
    *)
        echo "[ERROR] ARM_COMMAND_STYLE must be ptp or stream, got: ${arm_command_style}" >&2
        exit 2
        ;;
esac

if ! [[ "${action_stride}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] ACTION_STRIDE must be a positive integer, got: ${action_stride}" >&2
    exit 2
fi
if ! [[ "${action_history_horizon}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] ACTION_HISTORY_HORIZON must be a positive integer, got: ${action_history_horizon}" >&2
    exit 2
fi
if ! [[ "${rtc_action_horizon}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] RTC_ACTION_HORIZON must be a positive integer, got: ${rtc_action_horizon}" >&2
    exit 2
fi
if ! [[ "${rtc_max_prefix}" =~ ^[0-9]+$ ]] || (( rtc_max_prefix > rtc_action_horizon )); then
    echo "[ERROR] RTC_MAX_PREFIX must be an integer in [0, RTC_ACTION_HORIZON], got: ${rtc_max_prefix}" >&2
    exit 2
fi
if (( rtc_max_prefix > 0 && action_stride != 1 )); then
    echo "[ERROR] RTC requires ACTION_STRIDE=1" >&2
    exit 2
fi
# The clean runtime has one production path: bimanual direct-guard execution.
action_aggregate="${ACTION_AGGREGATE:-weighted_average}"
action_ema_weight="${ACTION_EMA_WEIGHT:-0.75}"
chunk_size_threshold="${CHUNK_SIZE_THRESHOLD:-1.0}"

case "${action_aggregate}" in
    weighted_average|latest|average|conservative)
        ;;
    *)
        echo "[ERROR] ACTION_AGGREGATE must be weighted_average, latest, average, or conservative, got: ${action_aggregate}" >&2
        exit 2
        ;;
esac

case "${gripper_control_mode}" in
    direct)
        gripper_contact_torque_nm="${GRIPPER_CONTACT_TORQUE_NM:-0.35}"
        ;;
    postprocess)
        gripper_contact_torque_nm="${GRIPPER_CONTACT_TORQUE_NM:-0.45}"
        ;;
    *)
        echo "[ERROR] GRIPPER_CONTROL_MODE must be direct or postprocess, got: ${gripper_control_mode}" >&2
        exit 2
        ;;
esac

set +u
source "${ros_setup}"
if [[ ! -f "${camera_ros_setup}" ]]; then
    echo "[ERROR] RealSense ROS workspace setup does not exist: ${camera_ros_setup}" >&2
    exit 2
fi
source "${camera_ros_setup}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
unset ROS_LOCALHOST_ONLY
# The camera workspace's old wired-only profile is pinned to 6.6.7.100.  All
# three publishers and this subscriber now run on Left itself, so use FastDDS's
# built-in local transports and ignore any inherited path to that stale file.
unset FASTRTPS_DEFAULT_PROFILES_FILE

cd "${repo_dir}"

if [[ "${SKIP_CAMERAS:-0}" == "1" ]]; then
    echo "[INFO] SKIP_CAMERAS=1; cameras are owned by the control agent"
else
    "${script_dir}/tools/ensure_cameras.sh"
fi

# The Gento controller accepts only one SDK client. The platform's
# marvin_robot_node normally owns local UDP 3720-3724 while it is linked.
# Refuse early so a known conflict does not produce a misleading link error.
controller_port_bindings="$(
    ss -H -uan 2>/dev/null |
        awk '$4 ~ /:(3720|3721|3722|3723|3724)$/'
)"
if [[ -n "${controller_port_bindings}" ]]; then
    echo "[ERROR] Gento controller UDP ports 3720-3724 are already in use:" >&2
    echo "${controller_port_bindings}" >&2
    echo "[ERROR] Another robot client (often marvin_robot_node) is still linked." >&2
    echo "[ERROR] Stop the platform robot module/controller client before direct inference." >&2
    echo "[ERROR] Find its PID with:" >&2
    echo "        sudo ss -uapn | grep -E ':(3720|3721|3722|3723|3724)\\b'" >&2
    echo "[ERROR] Stop that client/service, wait 2 seconds, then retry." >&2
    exit 1
fi

if pgrep -f '/dm_gripper_py/dm_motor_test([[:space:]]|$)' >/dev/null; then
    echo "[WARN] dm_gripper_py is running but is not used by this direct-terminal client."
    echo "       Do not run a separate vcan/terminal bridge at the same time."
fi

client_args=(
    --server "${server_endpoint}"
    --controller-ip "${controller_ip}"
    --camera-source ros2
    --ros-camera "head_color=${head_topic}"
    --ros-camera "wrist_left=${left_wrist_topic}"
    --ros-camera "wrist_right=${right_wrist_topic}"
    --ros-camera-reliability "${ros_camera_reliability}"
    --ros-camera-qos-depth "${ros_camera_qos_depth}"
    --runtime-control-socket "${runtime_control_socket}"
    --jpeg-quality "${POLICY_JPEG_QUALITY:-70}"
    --action-hz "${action_hz}"
    --arm-command-style "${arm_command_style}"
    --model-action-hz "${model_action_hz}"
    --observation-hz "${observation_hz}"
    --max-pending-policy-requests "${max_pending_policy_requests}"
    --action-stride "${action_stride}"
    --action-history-horizon "${action_history_horizon}"
    --rtc-action-horizon "${rtc_action_horizon}"
    --rtc-max-prefix "${rtc_max_prefix}"
    # Install the requested joint-impedance ratios during the transition.  Once
    # JointImp is active, prepare_arm() repeats them through the same separate
    # runtime velocity/acceleration calls as FXPlatform's Confirm Speed button.
    --velocity "${impedance_velocity_percent}"
    --acceleration "${impedance_acceleration_percent}"
    # Enter JointImp while holding measured feedback, then move both arms on
    # one locally generated smooth joint trajectory. Controller ratios and the
    # per-cycle/feedback-lead bounds are independent of inference playback.
    --initial-position-velocity "${initial_position_velocity_percent}"
    --initial-position-acceleration "${initial_position_acceleration_percent}"
    # GR00T lazily initializes part of its VLM processor on the first real
    # observation.  Give only that first action a longer window; once one
    # action chunk arrives, the normal 5-second runtime watchdog applies.
    --startup-timeout-ms "${POLICY_STARTUP_TIMEOUT_MS:-60000}"
    --timeout-ms "${POLICY_TIMEOUT_MS:-5000}"
    --arm-settle-timeout "${ARM_SETTLE_TIMEOUT:-10}"
    --initial-move-hz "${INITIAL_MOVE_HZ:-20}"
    --initial-move-step-deg "${initial_move_step_deg}"
    --initial-move-max-lead-deg "${initial_move_max_lead_deg}"
    # JointImp intentionally permits a small steady-state following error.
    # Three degrees is sufficient for the policy start pose without waiting
    # forever for rigid position-mode accuracy.
    --initial-move-tolerance-deg "${INITIAL_MOVE_TOLERANCE_DEG:-3.0}"
    --initial-move-timeout "${INITIAL_MOVE_TIMEOUT:-60}"
    --initial-gripper-position "${INITIAL_GRIPPER_OPEN_POSITION:-0.0}"
    # The old 0.02 normalized following window is only 0.032 rad with the
    # default DM range.  At kp=3 that can be too weak to overcome static
    # friction, causing the command to remain one tiny step ahead of feedback
    # forever.  Keep a bounded feedback-following move, but allow 0.05
    # normalized (0.08 rad) during the initial open trajectory.
    --initial-gripper-step "${INITIAL_GRIPPER_STEP:-0.05}"
    # The DM4310 can remain in a small near-open static-friction band while
    # enabled and returning healthy feedback.  Accept <=5% as initialized;
    # do not raise MIT gains merely to force the final few milliradians.
    --initial-gripper-tolerance "${INITIAL_GRIPPER_TOLERANCE:-0.05}"
    --initial-gripper-timeout "${INITIAL_GRIPPER_TIMEOUT:-20}"
    # In the default direct_guard mode every model arm target inside the
    # 20-degree policy-jump envelope passes unchanged. A larger model delta is
    # proportionally projected as one bimanual vector, so relative joint motion
    # is preserved and inference continues. Sustained target-versus-feedback
    # tracking error never clears the action queue or triggers a fresh policy
    # request.
    --max-arm-step-deg "${MAX_ARM_STEP_DEG:-8.0}"
    --hard-arm-jump-deg "${HARD_ARM_JUMP_DEG:-20.0}"
    --arm-max-velocity-deg-s "${ARM_MAX_VELOCITY_DEG_S:-40.0}"
    --wrist-max-velocity-deg-s "${WRIST_MAX_VELOCITY_DEG_S:-30.0}"
    --arm-max-acceleration-deg-s2 "${ARM_MAX_ACCELERATION_DEG_S2:-600.0}"
    --wrist-max-acceleration-deg-s2 "${WRIST_MAX_ACCELERATION_DEG_S2:-450.0}"
    --tracking-error-timeout "${TRACKING_ERROR_TIMEOUT:-2.0}"
    --tracking-error-reset-timeout "${TRACKING_ERROR_RESET_TIMEOUT:-1.0}"
    --no-tracking-error-replan
    # Allow a binary open/close policy target to reach the DM position loop in
    # one control cycle.  The old feedback-relative 0.25 window kept the MIT
    # target close to a lagging motor, which made closing unnecessarily slow
    # under load.  Position limits and the motor's 100 Hz MIT loop still apply.
    --max-gripper-step "${MAX_GRIPPER_STEP:-0.9}"
    # PTP publishes each accepted policy waypoint directly; stream retains the
    # bounded slew from the preceding published target.
    --interpolation-step-deg "${INTERPOLATION_STEP_DEG:-3.0}"
    --camera-read-timeout "${CAMERA_READ_TIMEOUT:-1.0}"
    --camera-max-age "${CAMERA_MAX_AGE:-0.5}"
    --camera-sync-tolerance "${CAMERA_SYNC_TOLERANCE:-0.15}"
    --chunk-size-threshold "${chunk_size_threshold}"
    --aggregate "${action_aggregate}"
    --action-ema-weight "${action_ema_weight}"
    # Empty-gripper endpoints are robot-specific. Left's open endpoint remains
    # 0.025 rad; Right's open hard stop was remeasured on Left on 2026-09-03 at
    # about 0.0963 rad, where the old 0.020-rad target stalled at -0.23 Nm.
    --gripper-right-min "${GRIPPER_RIGHT_MIN:-0.097}"
    --gripper-right-max "${GRIPPER_RIGHT_MAX:-1.212}"
    --gripper-left-min "${GRIPPER_LEFT_MIN:-0.025}"
    --gripper-left-max "${GRIPPER_LEFT_MAX:-1.188}"
    --gripper-feedback-timeout "${GRIPPER_FEEDBACK_TIMEOUT:-15}"
    --gripper-command-hz "${GRIPPER_COMMAND_HZ:-100}"
    --gripper-kp "${GRIPPER_KP:-3.0}"
    --gripper-kd "${GRIPPER_KD:-0.12}"
    --gripper-init-timeout "${GRIPPER_INIT_TIMEOUT:-3}"
    --gripper-trace-interval "${GRIPPER_TRACE_INTERVAL:-2}"
    --gripper-control-mode "${gripper_control_mode}"
    # Direct mode matches the original dm_gripper_py teleoperation semantics:
    # target_rad = policy_command * 1.6, so 0.9 requests 1.44 rad.
    --gripper-teleop-range-rad "${GRIPPER_TELEOP_RANGE_RAD:-1.6}"
    # Amplify partial policy close requests in the hardware-control mapping.
    # The normalized request remains bounded; physical overtravel is configured
    # separately below and remains guarded by sustained contact detection.
    --gripper-close-scale "${GRIPPER_CLOSE_SCALE:-1.25}"
    # Add a small force-limited physical target margin after the measured
    # empty-gripper endpoint. Feedback normalization keeps its calibration.
    --gripper-close-overtravel-rad "${GRIPPER_CLOSE_OVERTRAVEL_RAD:-0.18}"
    # Hold the strongest close request through small policy oscillations. Only
    # an explicit open request releases it, so placement remains responsive.
    --gripper-hold-enter-command "${GRIPPER_HOLD_ENTER_COMMAND:-0.60}"
    --gripper-hold-release-command "${GRIPPER_HOLD_RELEASE_COMMAND:-0.35}"
    --gripper-open-hold-seconds "${GRIPPER_OPEN_HOLD_SECONDS:-1.5}"
    --gripper-contact-release-command "${GRIPPER_CONTACT_RELEASE_COMMAND:-0.55}"
    --gripper-close-rate-rad-s "${GRIPPER_CLOSE_RATE_RAD_S:-0.40}"
    --gripper-contact-torque-nm "${gripper_contact_torque_nm}"
    --gripper-contact-velocity-rad-s "${GRIPPER_CONTACT_VELOCITY_RAD_S:-0.12}"
    --gripper-contact-cycles "${GRIPPER_CONTACT_CYCLES:-5}"
    --gripper-regrip-torque-nm "${GRIPPER_REGRIP_TORQUE_NM:-0.20}"
    --gripper-regrip-cycles "${GRIPPER_REGRIP_CYCLES:-10}"
)

if [[ "${start_paused}" == "1" ]]; then
    client_args+=(--start-paused)
else
    client_args+=(--no-start-paused)
fi

# Local orchestration patch (ws dual-robot control agent): skip the JointImp
# initial-pose move when the previous motion phase already placed the arms.
if [[ "${SKIP_INITIAL_POSE:-0}" == "1" ]]; then
    client_args+=(--skip-initial-pose)
fi

# Preserve the three RealSense sources' native 640x360 geometry by default.
# Explicit resizing remains available when the deployed checkpoint requires it.
case "${RESIZE_ROS_IMAGES:-0}" in
    0)
        client_args+=(--no-resize-ros-images)
        ;;
    1)
        client_args+=(
            --resize-ros-images
            --width "${POLICY_IMAGE_WIDTH:-640}"
            --height "${POLICY_IMAGE_HEIGHT:-480}"
        )
        ;;
    *)
        echo "[ERROR] RESIZE_ROS_IMAGES must be 0 or 1" >&2
        exit 2
        ;;
esac

echo "[INFO] Bimanual direct-guard execution: aggregate=${action_aggregate}, chunk_threshold=${chunk_size_threshold}, action_stride=${action_stride}"
echo "[INFO] Arm command style: ${arm_command_style} (JointImp; ptp=direct waypoint, stream=limited slew)"
echo "[INFO] Full-chunk timing: model=${model_action_hz}Hz, execution=${action_hz}Hz, observation_refresh<=${observation_hz}Hz"
echo "[INFO] Policy pipeline: one active + one latest server-pending observation, client outstanding cap=${max_pending_policy_requests}"
echo "[INFO] History: action=${action_history_horizon}, state=16, timestep_rate=${action_hz}Hz"
echo "[INFO] RTC committed prefix: max=${rtc_max_prefix}, action_horizon=${rtc_action_horizon}"
echo "[INFO] Action overlap aggregation: ${action_aggregate}, new-action EMA weight=${action_ema_weight}"
echo "[INFO] Initial synchronized quintic move in JointImp: velocity=${initial_position_velocity_percent}%, acceleration=${initial_position_acceleration_percent}%, frequency=${INITIAL_MOVE_HZ:-20}Hz, max_step=${initial_move_step_deg}deg, max_feedback_lead=${initial_move_max_lead_deg}deg"
echo "[INFO] Joint impedance transition: velocity=${impedance_velocity_percent}%, acceleration=${impedance_acceleration_percent}%"
echo "[INFO] Sustained joint tracking-error hold-and-replan: disabled for all joints"
echo "[INFO] Arm units: Gento SDK/safety=degree, policy transport/checkpoint=radian (protocol v5)"
echo "[INFO] Camera source: persistent three-stream RealSense launcher, message_type=raw, reliability=${ros_camera_reliability}, qos_depth=${ros_camera_qos_depth}"
echo "[INFO] Camera lifecycle: reuse across inference restarts; stop with ${script_dir}/tools/stop_cameras.sh"
echo "[INFO] Camera topics: head=${head_topic}, left=${left_wrist_topic}, right=${right_wrist_topic}"
echo "[INFO] Runtime control: socket=${runtime_control_socket}, start_paused=${start_paused}"
echo "[INFO] Gripper control mode: ${gripper_control_mode}"
if [[ "${gripper_control_mode}" == "postprocess" ]]; then
    echo "[INFO] Gripper close scale: ${GRIPPER_CLOSE_SCALE:-1.25} (effective command remains capped at 0.9)"
    echo "[INFO] Gripper close overtravel: ${GRIPPER_CLOSE_OVERTRAVEL_RAD:-0.18} rad (force-limited)"
    echo "[INFO] Gripper command hold: enter>=${GRIPPER_HOLD_ENTER_COMMAND:-0.60}, release<=${GRIPPER_HOLD_RELEASE_COMMAND:-0.35}"
    echo "[INFO] Gripper forced-open hold: ${GRIPPER_OPEN_HOLD_SECONDS:-1.5} seconds"
    echo "[INFO] Gripper confirmed-contact release threshold: ${GRIPPER_CONTACT_RELEASE_COMMAND:-0.55}"
else
    echo "[INFO] Gripper direct teleop mapping: command x ${GRIPPER_TELEOP_RANGE_RAD:-1.6} rad (0.9 -> 1.44 rad); motor target updates immediately without a close-rate ramp"
fi
echo "[INFO] Gripper contact torque: ${gripper_contact_torque_nm} Nm"
if [[ "${RESIZE_ROS_IMAGES:-0}" == "1" ]]; then
    echo "[WARN] ROS images will be resized to ${POLICY_IMAGE_WIDTH:-640}x${POLICY_IMAGE_HEIGHT:-480} before transport"
else
    echo "[INFO] ROS images keep their native bridge resolutions and aspect ratios"
fi

# Record the exact head and right-wrist JPEGs that are sent to the policy.
# Saving happens on the trajectory writer thread and never blocks control.
if [[ "${RECORD_CAMERA_VIEWS:-0}" == "1" ]]; then
    client_args+=(--record-camera-frames)
else
    client_args+=(--no-record-camera-frames)
fi

if [[ -n "${TASK:-}" ]]; then
    client_args+=(--task "${TASK}")
fi
if [[ "${DRY_RUN:-0}" == "1" ]]; then
    client_args+=(--dry-run)
    if [[ -n "${DRY_RUN_GRIPPER_TARGET:-}" ]]; then
        client_args+=(--dry-run-gripper-target "${DRY_RUN_GRIPPER_TARGET}")
    fi
fi

exec "${python_bin}" -m remote_inference_async.async_robot_client "${client_args[@]}"
