#!/usr/bin/env bash
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

set -euo pipefail

source /tmp/skye_factr_limit_trial.env

case "$SIDE" in
  left) expected_usb=usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0 ;;
  right) expected_usb=usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0 ;;
  *) printf '无效侧别: %s\n' "$SIDE" >&2; exit 1 ;;
esac
if [[ "$USB_BY_ID" != "$expected_usb" ]]; then
  printf '所选侧 USB by-id 与设备 A 已验收值不一致: %s\n' "$USB_BY_ID" >&2
  exit 1
fi

usb_path="/dev/serial/by-id/$USB_BY_ID"
if [[ ! -L "$usb_path" ]]; then
  printf 'USB by-id 不存在: %s\n' "$usb_path" >&2
  exit 1
fi

tty_path="$(readlink -f -- "$usb_path")"
if [[ ! -c "$tty_path" ]]; then
  printf 'USB by-id 未指向字符设备: %s -> %s\n' "$usb_path" "$tty_path" >&2
  exit 1
fi

tty_name="$(basename -- "$tty_path")"
latency_path="/sys/bus/usb-serial/devices/$tty_name/latency_timer"
if [[ ! -f "$latency_path" ]]; then
  printf '找不到 FTDI latency timer: %s\n' "$latency_path" >&2
  exit 1
fi

printf '1\n' | sudo tee "$latency_path" >/dev/null
latency_value="$(cat "$latency_path")"
if [[ "$latency_value" != 1 ]]; then
  printf 'latency timer 回读不是 1: %s\n' "$latency_value" >&2
  exit 1
fi

printf '侧别=%s ID=%s 边界=%s\nUSB=%s -> %s\nlatency_timer=%s\n' \
  "$SIDE" "$JOINT_ID" "$BOUNDARY" "$usb_path" "$tty_path" "$latency_value"
