#!/usr/bin/env bash
# Host UDP socket tuning for ROS 2 Fast-DDS under teleop load
# (profile: skye_robot_driver/config/fastrtps_teleop.xml).
#
# Without this, Linux defaults (~208 KB) overflow under teleop load /
# participant churn → UdpRcvbufErrors + "sequence size exceeds remaining buffer".
# See docs/Q_log.md.
#
# Usage:
#   sudo ./scripts/setup_host_tuning.sh          # write sysctl.d + apply now
#   ./scripts/setup_host_tuning.sh --check       # print current values (no root)
#   sudo ./scripts/setup_host_tuning.sh --dry-run # show what would be written
# Kernel UDP buffers:
#   rmem_max / wmem_max     = 16 MB  (cap; FASTRTPS xml may request up to this)
#   rmem_default           =  8 MB  (sockets without an explicit SO_RCVBUF)
#   wmem_default           =  1 MB
# fastrtps_teleop.xml asks for 4 MB send/recv; 8 MB default covers participants
# that forget the profile (e.g. older hitl launches).

set -euo pipefail

CONF_PATH="/etc/sysctl.d/60-ros2-dds-udp.conf"
CONF_BODY='# ROS 2 DDS over UDP tuning (skye teleop / Fast-DDS)
# Docs: docs/Q_log.md
# rmem_default=8MB, rmem_max=16MB (fastrtps_teleop.xml uses 4MB sockets)
net.core.rmem_max=16777216
net.core.wmem_max=16777216
net.core.rmem_default=8388608
net.core.wmem_default=1048576
'

EXPECTED_RMEM_DEFAULT=8388608
EXPECTED_RMEM_MAX=16777216

MODE="apply"
for arg in "$@"; do
  case "$arg" in
    --check) MODE="check" ;;
    --dry-run) MODE="dry-run" ;;
    -h|--help)
      sed -n '2,16p' "$0"
      exit 0
      ;;
    *)
      echo "Unknown arg: $arg (use --check | --dry-run)" >&2
      exit 2
      ;;
  esac
done

print_values() {
  echo "Current UDP buffer sysctl:"
  sysctl net.core.rmem_max net.core.wmem_max net.core.rmem_default net.core.wmem_default 2>/dev/null || true
  if [[ -f "${CONF_PATH}" ]]; then
    echo "Persistent file: ${CONF_PATH} (present)"
    if grep -q "net.core.rmem_default=${EXPECTED_RMEM_DEFAULT}" "${CONF_PATH}" \
        && grep -q "net.core.rmem_max=${EXPECTED_RMEM_MAX}" "${CONF_PATH}"; then
      echo "Persistent file content: OK (8MB default / 16MB max)"
    else
      echo "Persistent file content: STALE/WRONG (expected rmem_default=${EXPECTED_RMEM_DEFAULT}, rmem_max=${EXPECTED_RMEM_MAX})" >&2
      echo "  Re-run: sudo $0" >&2
    fi
  else
    echo "Persistent file: ${CONF_PATH} (missing)"
  fi
  cur_default="$(sysctl -n net.core.rmem_default 2>/dev/null || echo 0)"
  if [[ "${cur_default}" != "${EXPECTED_RMEM_DEFAULT}" ]]; then
    echo "LIVE rmem_default=${cur_default} != expected ${EXPECTED_RMEM_DEFAULT} (8MB) — apply with: sudo $0" >&2
  fi
}

if [[ "${MODE}" == "check" ]]; then
  print_values
  exit 0
fi

if [[ "${MODE}" == "dry-run" ]]; then
  echo "Would write ${CONF_PATH}:"
  printf '%s' "${CONF_BODY}"
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "ERROR: apply mode needs root. Re-run:" >&2
  echo "  sudo $0" >&2
  echo "Or inspect only:  $0 --check" >&2
  exit 1
fi

printf '%s' "${CONF_BODY}" > "${CONF_PATH}"
chmod 644 "${CONF_PATH}"
sysctl --system >/dev/null 2>&1 || sysctl -p "${CONF_PATH}"

echo "Wrote and applied ${CONF_PATH}"
print_values
