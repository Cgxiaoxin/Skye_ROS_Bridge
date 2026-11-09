#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# vendor/ (parent of remote_inference_async/) so -m remote_inference_async.* works
repo_dir="$(cd "${script_dir}/../.." && pwd)"
python_bin="${PYTHON_BIN:-python3}"

cd "${repo_dir}"
exec "${python_bin}" -m remote_inference_async.runtime_ctl \
    --socket "${RUNTIME_CONTROL_SOCKET:-/tmp/gento_robot_runtime.sock}" \
    "$@"
