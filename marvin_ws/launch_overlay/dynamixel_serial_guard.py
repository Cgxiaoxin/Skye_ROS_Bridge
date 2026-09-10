"""Catch USB-serial disconnects so FACTR's 500 Hz loop does not kill the node.

Patches dynamixel_sdk PortHandler read/write/clear. Does not reopen the port:
after a real USB drop, relaunch teleop. Logs at most once per second per port.
"""

from __future__ import annotations

import sys
import time
from typing import Any

_LOG_PERIOD_S = 1.0
_last_log: dict[tuple[str, str], float] = {}


def _port_name(handler: Any) -> str:
    name = getattr(handler, "port_name", None)
    if name:
        return str(name)
    ser = getattr(handler, "ser", None)
    return str(getattr(ser, "port", "?"))


def _log(op: str, handler: Any, exc: BaseException) -> None:
    key = (_port_name(handler), op)
    now = time.monotonic()
    if now - _last_log.get(key, 0.0) < _LOG_PERIOD_S:
        return
    _last_log[key] = now
    print(f"[serial_guard] {op} {key[0]} {type(exc).__name__}: {exc}", flush=True)


def _serial_errors() -> tuple[type[BaseException], ...]:
    errors: list[type[BaseException]] = [OSError]
    try:
        from serial.serialutil import SerialException

        errors.insert(0, SerialException)
    except Exception:
        pass
    return tuple(errors)


def install() -> None:
    from dynamixel_sdk import port_handler

    if getattr(port_handler.PortHandler.readPort, "_skye_serial_guard", False):
        return

    orig_read = port_handler.PortHandler.readPort
    orig_write = port_handler.PortHandler.writePort
    orig_clear = port_handler.PortHandler.clearPort
    err_types = _serial_errors()

    def readPort(self, length):  # noqa: N802 — SDK name
        try:
            return orig_read(self, length)
        except err_types as exc:
            _log("read", self, exc)
            return b"" if sys.version_info > (3, 0) else []

    def writePort(self, packet):  # noqa: N802 — SDK name
        try:
            return orig_write(self, packet)
        except err_types as exc:
            _log("write", self, exc)
            return 0

    def clearPort(self):  # noqa: N802 — SDK name
        try:
            return orig_clear(self)
        except err_types as exc:
            _log("clear", self, exc)
            return None

    readPort._skye_serial_guard = True
    port_handler.PortHandler.readPort = readPort
    port_handler.PortHandler.writePort = writePort
    port_handler.PortHandler.clearPort = clearPort
    print("[serial_guard] installed on dynamixel_sdk.PortHandler", flush=True)
