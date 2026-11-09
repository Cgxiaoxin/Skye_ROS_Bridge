#!/usr/bin/env bash
set -euo pipefail

camera_package="${CAMERA_PACKAGE:-tianji_gmsl_color_bridge}"
camera_log_dir="${CAMERA_LOG_DIR:-${HOME}/.ros/remote_inference_cameras}"
stop_timeout_seconds="${CAMERA_STOP_TIMEOUT_SECONDS:-10}"

if ! [[ "${stop_timeout_seconds}" =~ ^[1-9][0-9]*$ ]]; then
    echo "[ERROR] CAMERA_STOP_TIMEOUT_SECONDS must be a positive integer, got: ${stop_timeout_seconds}" >&2
    exit 2
fi

stop_managed_launch() {
    local label="$1"
    local launch_file="$2"
    local pid_file="${camera_log_dir}/${label}.pid"
    local pid
    local command_line
    local deadline

    if [[ ! -r "${pid_file}" ]]; then
        echo "[INFO] No managed ${label} camera PID file"
        return
    fi
    read -r pid <"${pid_file}"
    if ! [[ "${pid}" =~ ^[1-9][0-9]*$ ]]; then
        echo "[ERROR] Invalid PID in ${pid_file}; leaving it untouched" >&2
        return 1
    fi
    if ! kill -0 "${pid}" 2>/dev/null; then
        rm -f "${pid_file}"
        echo "[INFO] Removed stale ${label} camera PID file"
        return
    fi

    command_line="$(ps -p "${pid}" -o args= 2>/dev/null || true)"
    if [[ "${command_line}" != *"${camera_package}"* || "${command_line}" != *"${launch_file}"* ]]; then
        echo "[ERROR] PID ${pid} does not match the managed ${label} camera launcher" >&2
        echo "[ERROR] Command: ${command_line}" >&2
        echo "[ERROR] Refusing to stop it; leaving ${pid_file} untouched" >&2
        return 1
    fi

    echo "[INFO] Stopping managed RealSense ${label} launcher (PID ${pid})"
    kill -TERM "${pid}"
    deadline=$((SECONDS + stop_timeout_seconds))
    while kill -0 "${pid}" 2>/dev/null && (( SECONDS < deadline )); do
        sleep 0.2
    done
    if kill -0 "${pid}" 2>/dev/null; then
        echo "[ERROR] ${label} camera launcher did not stop within ${stop_timeout_seconds}s" >&2
        echo "[ERROR] PID file retained: ${pid_file}" >&2
        return 1
    fi
    rm -f "${pid_file}"
    echo "[INFO] Managed RealSense ${label} launcher stopped"
}

stop_status=0
stop_managed_launch head head_d435_usb_color.launch.py || stop_status=1
stop_managed_launch wrist dual_d401_gmsl_color.launch.py || stop_status=1
exit "${stop_status}"
