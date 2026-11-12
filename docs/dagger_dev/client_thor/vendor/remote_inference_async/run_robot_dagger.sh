#!/usr/bin/env bash
# HITL DAgger client: ZMQ obs/action to 5090, publish /skye/policy_action.
# Does NOT link Gento SDK / CONTROLLER_IP (arbiter owns the arm).
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/.." && pwd)"

inference_server_ip="${INFERENCE_HOST:-192.168.201.253}"
inference_server_port="${INFERENCE_PORT:-5568}"
server_endpoint="tcp://${inference_server_ip}:${inference_server_port}"
joint_states_topic="${JOINT_STATES_TOPIC:-/gento/joint_states}"
dagger_policy_version="${DAGGER_POLICY_VERSION:-unijepa}"
controller_ip="${CONTROLLER_IP:-0.0.0.0}"

ros_setup="${ROS_SETUP:-/opt/kernelmind/apex/install/setup.bash}"
camera_ros_setup="${CAMERA_ROS_SETUP:-/home/user/tianji_capture_camera_ws/install/setup.bash}"
skye_ws_setup="${SKYE_WS_SETUP:-${SKYE_WS:-/data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws}/install/setup.bash}"
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
observation_hz="${POLICY_OBSERVATION_HZ:-30}"
max_pending_policy_requests="${MAX_PENDING_POLICY_REQUESTS:-2}"
action_history_horizon="${ACTION_HISTORY_HORIZON:-30}"
rtc_action_horizon="${RTC_ACTION_HORIZON:-50}"
rtc_max_prefix="${RTC_MAX_PREFIX:-0}"
runtime_control_socket="${RUNTIME_CONTROL_SOCKET:-/tmp/gento_robot_runtime_dagger.sock}"
action_aggregate="${ACTION_AGGREGATE:-weighted_average}"
action_ema_weight="${ACTION_EMA_WEIGHT:-0.75}"
chunk_size_threshold="${CHUNK_SIZE_THRESHOLD:-1.0}"

case "${action_aggregate}" in
    weighted_average|latest|average|conservative) ;;
    *)
        echo "[ERROR] ACTION_AGGREGATE invalid: ${action_aggregate}" >&2
        exit 2
        ;;
esac
if ! [[ "${action_stride}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] ACTION_STRIDE must be a positive integer" >&2
    exit 2
fi

set +u
if [[ -f "${ros_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${ros_setup}"
fi
if [[ -f "${skye_ws_setup}" ]]; then
    # shellcheck disable=SC1090
    source "${skye_ws_setup}"
else
    echo "[WARN] Skye workspace setup missing: ${skye_ws_setup}" >&2
    echo "[WARN] PolicyActionChunk import may fail" >&2
fi
if [[ "${SKIP_CAMERAS:-0}" != "1" ]]; then
    if [[ ! -f "${camera_ros_setup}" ]]; then
        echo "[ERROR] Camera ROS setup missing: ${camera_ros_setup}" >&2
        exit 2
    fi
    # shellcheck disable=SC1090
    source "${camera_ros_setup}"
fi
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-20}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
unset ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE

cd "${repo_dir}"

if [[ "${SKIP_CAMERAS:-0}" == "1" ]]; then
    echo "[INFO] SKIP_CAMERAS=1; cameras owned elsewhere"
else
    "${script_dir}/ensure_cameras.sh"
fi

# Intentionally skip UDP 3720-3724 occupancy check: skye_robot_driver owns
# the controller under HITL DAgger; this process must not link Gento SDK.

client_args=(
    --server "${server_endpoint}"
    --controller-ip "${controller_ip}"
    --dagger-mode
    --joint-states-topic "${joint_states_topic}"
    --dagger-policy-version "${dagger_policy_version}"
    --camera-source ros2
    --ros-camera "head_color=${head_topic}"
    --ros-camera "wrist_left=${left_wrist_topic}"
    --ros-camera "wrist_right=${right_wrist_topic}"
    --ros-camera-reliability "${ros_camera_reliability}"
    --ros-camera-qos-depth "${ros_camera_qos_depth}"
    --runtime-control-socket "${runtime_control_socket}"
    --jpeg-quality "${POLICY_JPEG_QUALITY:-70}"
    --action-hz "${action_hz}"
    --arm-command-style stream
    --model-action-hz "${model_action_hz}"
    --observation-hz "${observation_hz}"
    --max-pending-policy-requests "${max_pending_policy_requests}"
    --action-stride "${action_stride}"
    --action-history-horizon "${action_history_horizon}"
    --rtc-action-horizon "${rtc_action_horizon}"
    --rtc-max-prefix "${rtc_max_prefix}"
    --velocity "${IMPEDANCE_VELOCITY_PERCENT:-100}"
    --acceleration "${IMPEDANCE_ACCELERATION_PERCENT:-100}"
    --initial-position-velocity "${INITIAL_POSITION_VELOCITY_PERCENT:-40}"
    --initial-position-acceleration "${INITIAL_POSITION_ACCELERATION_PERCENT:-40}"
    --startup-timeout-ms "${POLICY_STARTUP_TIMEOUT_MS:-60000}"
    --timeout-ms "${POLICY_TIMEOUT_MS:-5000}"
    --skip-initial-pose
    --no-start-paused
    --chunk-size-threshold "${chunk_size_threshold}"
    --aggregate "${action_aggregate}"
    --action-ema-weight "${action_ema_weight}"
    --gripper-control-mode "${gripper_control_mode}"
    --no-tracking-error-replan
)

case "${RESIZE_ROS_IMAGES:-0}" in
    0) client_args+=(--no-resize-ros-images) ;;
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

if [[ -n "${TASK:-}" ]]; then
    client_args+=(--task "${TASK}")
fi

echo "[INFO] DAGGER mode: DOMAIN=${ROS_DOMAIN_ID} server=${server_endpoint}"
echo "[INFO] joint_states=${joint_states_topic} → /skye/policy_action (no Gento SDK)"
echo "[INFO] model=${model_action_hz}Hz execution=${action_hz}Hz obs<=${observation_hz}Hz"

exec "${python_bin}" -m remote_inference_async.async_robot_client "${client_args[@]}"
