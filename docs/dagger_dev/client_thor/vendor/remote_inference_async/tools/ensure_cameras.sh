#!/usr/bin/env bash
set -euo pipefail

ros_setup="${ROS_SETUP:-/opt/kernelmind/apex/install/setup.bash}"
camera_ros_setup="${CAMERA_ROS_SETUP:-/home/user/tianji_capture_camera_ws/install/setup.bash}"
camera_package="${CAMERA_PACKAGE:-tianji_gmsl_color_bridge}"
camera_wait_seconds="${CAMERA_WAIT_SECONDS:-90}"
camera_log_dir="${CAMERA_LOG_DIR:-${HOME}/.ros/remote_inference_cameras}"
head_topic="${HEAD_COLOR_TOPIC:-/head_d435/camera/color/image_raw}"
left_wrist_topic="${WRIST_LEFT_TOPIC:-/left_wrist_d405/camera/color/image_raw}"
right_wrist_topic="${WRIST_RIGHT_TOPIC:-/right_wrist_d405/camera/color/image_raw}"

if ! [[ "${camera_wait_seconds}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] CAMERA_WAIT_SECONDS must be a positive integer, got: ${camera_wait_seconds}" >&2
    exit 2
fi
if [[ ! -f "${ros_setup}" ]]; then
    echo "[ERROR] ROS setup does not exist: ${ros_setup}" >&2
    exit 2
fi
if [[ ! -f "${camera_ros_setup}" ]]; then
    echo "[ERROR] RealSense ROS workspace setup does not exist: ${camera_ros_setup}" >&2
    exit 2
fi

set +u
source "${ros_setup}"
source "${camera_ros_setup}"
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-21}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export RMW_FASTRTPS_PUBLICATION_MODE="${RMW_FASTRTPS_PUBLICATION_MODE:-ASYNCHRONOUS}"
unset ROS_LOCALHOST_ONLY
unset FASTRTPS_DEFAULT_PROFILES_FILE

mkdir -p "${camera_log_dir}"
# v2 replaces the original lock whose descriptor was inherited by persistent
# ros2 launch processes. Camera children explicitly close this descriptor below.
exec 9>"${camera_log_dir}/ensure-v2.lock"
if ! flock -w 15 9; then
    echo "[ERROR] Timed out waiting for another camera startup check" >&2
    exit 1
fi

camera_log_tail() {
    local log_file="$1"
    if [[ -f "${log_file}" ]]; then
        tail -n 40 "${log_file}" >&2 || true
    fi
}

camera_topic_ready() {
    local topic="$1"
    timeout 5 ros2 topic echo \
        --no-daemon \
        --spin-time 2.0 \
        --once \
        --qos-reliability reliable \
        "${topic}" sensor_msgs/msg/Image >/dev/null 2>&1
}

all_camera_topics_ready() {
    camera_topic_ready "${head_topic}" &&
        camera_topic_ready "${left_wrist_topic}" &&
        camera_topic_ready "${right_wrist_topic}"
}

wait_for_camera_topic() {
    local topic="$1"
    local deadline=$((SECONDS + camera_wait_seconds))
    while (( SECONDS < deadline )); do
        if camera_topic_ready "${topic}"; then
            echo "[INFO] RealSense topic ready: ${topic}"
            return
        fi
        sleep 1
    done
    echo "[ERROR] RealSense camera topic timed out: ${topic}" >&2
    camera_log_tail "${camera_log_dir}/wrist.log"
    camera_log_tail "${camera_log_dir}/head.log"
    exit 1
}

managed_launch_alive() {
    local label="$1"
    local pid_file="${camera_log_dir}/${label}.pid"
    local pid
    [[ -r "${pid_file}" ]] || return 1
    read -r pid <"${pid_file}"
    [[ "${pid}" =~ ^[1-9][0-9]*$ ]] || return 1
    kill -0 "${pid}" 2>/dev/null
}

if all_camera_topics_ready; then
    echo "[INFO] Reusing the three existing RealSense camera streams"
    exit 0
fi

managed_wrist=0
managed_head=0
managed_launch_alive wrist && managed_wrist=1
managed_launch_alive head && managed_head=1
if (( managed_wrist == 1 || managed_head == 1 )); then
    if (( managed_wrist != managed_head )); then
        echo "[ERROR] Only one managed camera launcher is alive; refusing a partial duplicate start" >&2
        echo "[ERROR] Run ${BASH_SOURCE[0]%/*}/stop_cameras.sh, then retry" >&2
        exit 1
    fi
    echo "[INFO] Managed RealSense launchers are already running; waiting for fresh frames"
    wait_for_camera_topic "${head_topic}"
    wait_for_camera_topic "${left_wrist_topic}"
    wait_for_camera_topic "${right_wrist_topic}"
    echo "[INFO] All three existing RealSense camera streams are ready"
    exit 0
fi

rm -f "${camera_log_dir}/wrist.pid" "${camera_log_dir}/head.pid"
existing_camera_nodes="$(
    ros2 node list --no-daemon 2>/dev/null |
        grep -E '^/(head_d435|left_wrist_d405|right_wrist_d405)/camera$' || true
)"
if [[ -n "${existing_camera_nodes}" ]]; then
    echo "[INFO] Existing RealSense nodes found; waiting for all three streams"
    echo "${existing_camera_nodes}"
    wait_for_camera_topic "${head_topic}"
    wait_for_camera_topic "${left_wrist_topic}"
    wait_for_camera_topic "${right_wrist_topic}"
    echo "[INFO] All three externally managed RealSense camera streams are ready"
    exit 0
fi

[[ -e /dev/video-rs-color-0 ]] || {
    echo "[ERROR] Missing left wrist RealSense device: /dev/video-rs-color-0" >&2
    echo "[ERROR] Check: systemctl status tianji-gmsl.service" >&2
    exit 1
}
[[ -e /dev/video-rs-color-1 ]] || {
    echo "[ERROR] Missing right wrist RealSense device: /dev/video-rs-color-1" >&2
    echo "[ERROR] Check: systemctl status tianji-gmsl.service" >&2
    exit 1
}

started_pids=()
started_pid_files=()
startup_complete=0

cleanup_failed_startup() {
    local status=$?
    local pid
    local pid_file
    if (( startup_complete == 1 )); then
        return
    fi
    for pid in "${started_pids[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
            kill -TERM "${pid}" 2>/dev/null || true
        fi
    done
    for pid_file in "${started_pid_files[@]}"; do
        rm -f "${pid_file}"
    done
    exit "${status}"
}

trap cleanup_failed_startup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_camera_launch() {
    local label="$1"
    local launch_file="$2"
    local log_file="${camera_log_dir}/${label}.log"
    local pid_file="${camera_log_dir}/${label}.pid"
    local pid

    echo "[INFO] Starting persistent RealSense ${label}: ${launch_file}"
    nohup ros2 launch "${camera_package}" "${launch_file}" \
        9>&- </dev/null >"${log_file}" 2>&1 &
    pid=$!
    printf '%s\n' "${pid}" >"${pid_file}"
    started_pids+=("${pid}")
    started_pid_files+=("${pid_file}")
    sleep 1
    if ! kill -0 "${pid}" 2>/dev/null; then
        wait "${pid}" 2>/dev/null || true
        echo "[ERROR] RealSense ${label} launch exited during startup" >&2
        camera_log_tail "${log_file}"
        exit 1
    fi
}

start_camera_launch wrist dual_d401_gmsl_color.launch.py
start_camera_launch head head_d435_usb_color.launch.py
wait_for_camera_topic "${head_topic}"
wait_for_camera_topic "${left_wrist_topic}"
wait_for_camera_topic "${right_wrist_topic}"
startup_complete=1
trap - EXIT INT TERM
echo "[INFO] All three RealSense camera streams are ready and will remain running"
