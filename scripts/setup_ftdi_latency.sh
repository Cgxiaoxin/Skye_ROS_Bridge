#!/usr/bin/env bash
# Force FTDI USB-serial latency_timer = 1 via udev.
#
# Why: Dynamixel leader-arm drivers refuse to start when the FTDI converter
# reports latency_timer != 1 (default 16). The attribute lives in sysfs, so
# chmod/chown does not stick and the value resets on every replug / reboot.
# A udev rule is the only way to make it permanent.
#
# Usage:
#   sudo ./scripts/setup_ftdi_latency.sh    # install rule + apply now
#   ./scripts/setup_ftdi_latency.sh --check # print current values (no root)

set -euo pipefail

RULE_PATH="/etc/udev/rules.d/99-ftdi-latency.rules"
RULE_BODY='# FTDI USB-serial adapters: force 1ms latency timer (Dynamixel leader arms)
ACTION=="add|change", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
'

MODE="apply"
for arg in "$@"; do
  case "$arg" in
    --check) MODE="check" ;;
    -h|--help) sed -n '2,13p' "$0"; exit 0 ;;
    *) echo "Unknown arg: $arg (use --check)" >&2; exit 2 ;;
  esac
done

print_values() {
  echo "Current latency_timer:"
  for dev in /sys/bus/usb-serial/devices/*; do
    [[ -e "${dev}/latency_timer" ]] || continue
    printf '  %s = %s\n' "$(basename "${dev}")" "$(cat "${dev}/latency_timer")"
  done
  if [[ -f "${RULE_PATH}" ]]; then
    echo "Persistent rule: ${RULE_PATH} (present)"
  else
    echo "Persistent rule: ${RULE_PATH} (missing)"
  fi
}

if [[ "${MODE}" == "check" ]]; then
  print_values
  exit 0
fi

if [[ "$(id -u)" -ne 0 ]]; then
  echo "ERROR: apply mode needs root. Re-run: sudo $0" >&2
  exit 1
fi

printf '%s' "${RULE_BODY}" > "${RULE_PATH}"
chmod 644 "${RULE_PATH}"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb-serial --action=add
sleep 1

echo "Wrote ${RULE_PATH}"
print_values
