#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
pkg_dir="$(cd "${script_dir}/.." && pwd)"

export START_PAUSED=1
exec "${pkg_dir}/run_robot.sh"
