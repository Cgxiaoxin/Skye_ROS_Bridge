#!/usr/bin/env bash
# Host-side Skye operator UI (FastAPI + Svelte console).
#
# Usage:
#   ./scripts/start_operator_ui.sh
#
# Browser (default): http://127.0.0.1:8765  (port from skye_operator_ui/config/default.yaml)
# Suggested: google-chrome --app=http://127.0.0.1:8765

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/ros_domain_env.sh"
skye_source_ros || exit 1

echo "== operator_ui ROS_DOMAIN_ID=${ROS_DOMAIN_ID} =="
echo "   Open http://127.0.0.1:8765 (or port in config/default.yaml)"
echo ""

exec ros2 run skye_operator_ui operator_ui --ros-args -p repo_root:="${REPO_ROOT}"
