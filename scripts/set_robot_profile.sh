#!/usr/bin/env bash
# Persist machine profile so bare launches/scripts default correctly.
#
# Usage (on that machine, once):
#   ./scripts/set_robot_profile.sh orin
#   ./scripts/set_robot_profile.sh thor
#
# Resolution order everywhere:
#   ROBOT_PROFILE env > marvin_ws/.skye/robot_profile > marvin_ws/robot_profile > thor

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib/robot_profile.sh
source "${SCRIPT_DIR}/lib/robot_profile.sh"

PROFILE="${1:-}"
if [[ -z "${PROFILE}" ]]; then
  echo "Usage: $0 thor|orin" >&2
  exit 1
fi
validate_robot_profile "${PROFILE}" || exit 1
PROFILE="${PROFILE,,}"

REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
MARVIN_WS="${MARVIN_WS:-${REPO_ROOT}/marvin_ws}"
PROFILE_FILE="$(robot_profile_write_path "${MARVIN_WS}")"

mkdir -p "$(dirname "${PROFILE_FILE}")"
echo "${PROFILE}" > "${PROFILE_FILE}"
echo "Wrote ${PROFILE_FILE} -> ${PROFILE}"
echo "Next starts need no robot_profile:= / ROBOT_PROFILE= if this file is present."
