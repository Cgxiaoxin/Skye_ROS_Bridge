#!/usr/bin/env bash
set -e

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_dir="$(cd "${script_dir}/../.." && pwd)"
python_bin="${PYTHON_BIN:-python3}"
export PYTHONIOENCODING="${PYTHONIOENCODING:-utf-8}"

cd "${repo_dir}"
exec "${python_bin}" -m remote_inference_async.trajectory_keys "$@"
