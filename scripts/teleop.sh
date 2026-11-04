#!/usr/bin/env bash
# Repository entry point; no manual ROS environment setup required.
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec /usr/bin/python3 "${SCRIPT_DIR}/teleop.py" "$@"
