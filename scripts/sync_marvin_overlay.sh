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

if [[ -d "${OVERLAY_CFG}" ]]; then
  mkdir -p "${INSTALL_CFG}"
  cfg_files=("${OVERLAY_CFG}"/*.yaml)
  for src in "${cfg_files[@]}"; do
    cp -f "${src}" "${INSTALL_CFG}/"
    echo "  config: $(basename "${src}")"
  done
fi

echo "OK: overlay -> ${INSTALL_LAUNCH}"
echo "    grav_comp launch prefers ${OVERLAY_CFG} when present (see _grav_comp_config)."
