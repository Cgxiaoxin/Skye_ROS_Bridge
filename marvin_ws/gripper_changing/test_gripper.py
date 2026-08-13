#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time

from changingtek_gripper_adapter import ChangingTekGripperAdapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test ChangingTek gripper control.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port.")
    parser.add_argument("--slave-id", type=int, default=1, help="Modbus slave id.")
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate.")
    parser.add_argument("--timeout", type=float, default=0.3, help="Serial timeout seconds.")
    parser.add_argument("--speed", type=int, default=30, help="Move speed percent, 0-100.")
    parser.add_argument("--force", type=int, default=80, help="Move force percent, 0-100.")
    return parser.parse_args()


def print_status(gripper: ChangingTekGripperAdapter) -> None:
    print(f"position={gripper.position:.2f}  current={gripper.ForceValue:.2f}")


def main() -> None:
    args = parse_args()

    gripper = ChangingTekGripperAdapter(
        port=args.port,
        slave_id=args.slave_id,
        baudrate=args.baudrate,
        timeout=args.timeout,
        init_action=None,
    )

    print("ChangingTek gripper test")
    print("Commands: o=open(90), c=close(0), m=middle(45), p=print, q=quit")
    print("You can also enter a number from 0 to 90. 0=closed, 90=open.")
    print_status(gripper)

    while True:
        cmd = input("> ").strip().lower()
        if not cmd:
            continue
        if cmd in {"q", "quit", "exit"}:
            break
        if cmd in {"p", "pos", "position"}:
            print_status(gripper)
            continue

        if cmd in {"o", "open"}:
            target = 90.0
        elif cmd in {"c", "close"}:
            target = 0.0
        elif cmd in {"m", "mid", "middle"}:
            target = 45.0
        else:
            try:
                target = float(cmd)
            except ValueError:
                print("Unknown command. Use o/c/m/p/q or a number from 0 to 90.")
                continue

        target = max(0.0, min(90.0, target))
        print(f"move to {target:.1f}")
        gripper.move(pos=target, speed=args.speed, force=args.force)
        time.sleep(0.5)
        print_status(gripper)


if __name__ == "__main__":
    main()
