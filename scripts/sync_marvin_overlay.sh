#!/usr/bin/env bash
# Sync tracked marvin_ws overlay (launch + configs) into install/share.
#
# Run on HOST (repo root) before docker, or INSIDE docker (/marvin_ws mounted):
#   ./scripts/sync_marvin_overlay.sh
#   /scripts/sync_marvin_overlay.sh
#
# After sync you can use either:
#   ros2 launch factr_teleop start_teleop_m6_dual_gento.launch.py
#   ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py

set -euo pipefail

resolve_marvin_ws() {
  if [[ -n "${MARVIN_WS:-}" && -d "${MARVIN_WS}/launch_overlay" ]]; then
    printf '%s\n' "${MARVIN_WS}"
    return 0
  fi
  if [[ -d /marvin_ws/launch_overlay ]]; then
    printf '%s\n' "/marvin_ws"
    return 0
  fi
  local script_dir repo_root
  script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  repo_root="$(cd "${script_dir}/.." && pwd)"
  if [[ -d "${repo_root}/marvin_ws/launch_overlay" ]]; then
    printf '%s\n' "${repo_root}/marvin_ws"
    return 0
  fi
  return 1
}

MARVIN_WS="$(resolve_marvin_ws)" || {
  echo "ERROR: cannot find marvin_ws/launch_overlay (set MARVIN_WS or run from repo)" >&2
  exit 1
}

INSTALL_LAUNCH="${MARVIN_WS}/install/share/factr_teleop/launch"
INSTALL_CFG="${MARVIN_WS}/install/share/factr_teleop/configs"
OVERLAY_LAUNCH="${MARVIN_WS}/launch_overlay"
OVERLAY_CFG="${MARVIN_WS}/configs"

if [[ ! -d "${OVERLAY_LAUNCH}" ]]; then
  echo "ERROR: missing ${OVERLAY_LAUNCH}" >&2
  exit 1
fi

mkdir -p "${INSTALL_LAUNCH}"

shopt -s nullglob
launch_files=("${OVERLAY_LAUNCH}"/*.launch.py)
if ((${#launch_files[@]} == 0)); then
  echo "ERROR: no *.launch.py under ${OVERLAY_LAUNCH}" >&2
  exit 1
fi

for src in "${launch_files[@]}"; do
  cp -f "${src}" "${INSTALL_LAUNCH}/"
  echo "  launch: $(basename "${src}")"
done

PROFILE="${ROBOT_PROFILE:-${MARVIN_PROFILE:-thor}}"
case "${PROFILE}" in
  thor|orin) ;;
  *)
    echo "ERROR: ROBOT_PROFILE must be thor|orin (got: ${PROFILE})" >&2
    exit 1
    ;;
esac

PROFILE_CFG="${MARVIN_WS}/configs/${PROFILE}"
if [[ ! -d "${PROFILE_CFG}" ]]; then
  echo "ERROR: missing profile configs: ${PROFILE_CFG}" >&2
  exit 1
fi

mkdir -p "${INSTALL_CFG}"
shopt -s nullglob
cfg_files=("${PROFILE_CFG}"/grav_comp_m6_*.yaml)
if ((${#cfg_files[@]} == 0)); then
  echo "ERROR: no grav_comp_m6_*.yaml under ${PROFILE_CFG}" >&2
  exit 1
fi
for src in "${cfg_files[@]}"; do
  cp -f "${src}" "${INSTALL_CFG}/"
  echo "  config[${PROFILE}]: $(basename "${src}")"
done
echo "OK: profile=${PROFILE} overlay synced"

echo "OK: overlay -> ${INSTALL_LAUNCH}"
echo "    grav_comp launch prefers configs/<profile>/ (see _grav_comp_config)."
