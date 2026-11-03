#!/usr/bin/env python3
"""Disable torque on both FACTR leader Dynamixel chains (Torque Enable addr 64 = 0).

Loads ports from marvin_ws/.skye/leader_arms.env when present, else from the
environment (ROBOT_LEADER_DYNAMIXEL_PORT_LEFT / _RIGHT).

Hold the leaders before running — arms will drop when torque is off.

Usage (host, outside Docker):
  python3 scripts/disable_leader_dynamixel.py [--side left|right|both]
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

import yaml

_SIDE_ENV_KEYS: dict[str, str] = {
    "left": "ROBOT_LEADER_DYNAMIXEL_PORT_LEFT",
    "right": "ROBOT_LEADER_DYNAMIXEL_PORT_RIGHT",
}


def sides_for_arg(side: str) -> tuple[str, ...]:
    key = side.strip().lower()
    if key == "left":
        return ("left",)
    if key == "right":
        return ("right",)
    if key == "both":
        return ("left", "right")
    raise ValueError(f"invalid side: {side}")


def _repo_root() -> Path:
    return Path(__file__).resolve().parent.parent


def _load_binding_env(marvin_ws: Path) -> None:
    path = marvin_ws / ".skye" / "leader_arms.env"
    if not path.is_file():
        return
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if line.startswith("export "):
            line = line[len("export ") :]
        if "=" not in line:
            continue
        key, _, val = line.partition("=")
        key = key.strip()
        val = val.strip().strip("'").strip('"')
        if key and key not in os.environ:
            os.environ[key] = val


def _ports_from_host_yaml(repo: Path, profile: str) -> dict[str, str]:
    pkg = repo / "skye_ros2_ws/src/skye_factr_teleop/config"
    base = pkg / "orin" if profile == "orin" else pkg
    out: dict[str, str] = {}
    for side in ("left", "right"):
        doc = yaml.safe_load((base / f"grav_comp_m6_{side}.yaml").read_text(encoding="utf-8"))
        out[side] = str(doc["dynamixel"]["dynamixel_port"])
    return out


def _fill_ports_from_profile_if_missing(repo: Path) -> None:
    profile = os.environ.get("ROBOT_PROFILE", "").strip().lower()
    if profile not in ("thor", "orin"):
        return
    ports = _ports_from_host_yaml(repo, profile)
    for side, port in ports.items():
        key = _SIDE_ENV_KEYS[side]
        if not os.environ.get(key):
            os.environ[key] = port


def _port_path(name: str) -> str:
    if name.startswith("/"):
        return name
    return f"/dev/serial/by-id/{name}"


def _disable_side(label: str, env_key: str) -> None:
    from dynamixel_sdk import PacketHandler, PortHandler  # type: ignore

    name = os.environ.get(env_key)
    if not name:
        print(f"{label}: skip (missing {env_key})", file=sys.stderr)
        return

    port = _port_path(name)
    if not Path(port).exists() and not Path(name).exists():
        # Prefer by-id path; if bare name was a full path that exists, use it.
        print(f"{label}: skip (port not found: {port})", file=sys.stderr)
        return

    pk = PortHandler(port)
    if not pk.openPort():
        raise RuntimeError(f"{label}: openPort failed for {port}")
    if not pk.setBaudRate(4_000_000):
        pk.closePort()
        raise RuntimeError(f"{label}: setBaudRate failed for {port}")

    ph = PacketHandler(2.0)
    try:
        for motor_id in range(1, 9):
            ph.write1ByteTxRx(pk, motor_id, 64, 0)
            print(f"{label} disabled {motor_id}")
    finally:
        pk.closePort()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--side",
        default="both",
        choices=("left", "right", "both"),
        help="which leader arm chain to disable (default: both)",
    )
    args = parser.parse_args()

    marvin_ws = Path(os.environ.get("MARVIN_WS") or (_repo_root() / "marvin_ws"))
    _load_binding_env(marvin_ws)
    _fill_ports_from_profile_if_missing(_repo_root())

    try:
        import dynamixel_sdk  # noqa: F401
    except ImportError:
        print(
            "ERROR: dynamixel_sdk not importable in this Python "
            f"({sys.executable}). Install dynamixel_sdk on the host or run via "
            "leader_arm_gate.sh with MARVIN_WS PYTHONPATH configured.",
            file=sys.stderr,
        )
        return 2

    errors = 0
    for label in sides_for_arg(args.side):
        key = _SIDE_ENV_KEYS[label]
        try:
            _disable_side(label, key)
        except Exception as exc:  # noqa: BLE001 - report both sides
            print(f"{label}: ERROR {exc}", file=sys.stderr)
            errors += 1

    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
