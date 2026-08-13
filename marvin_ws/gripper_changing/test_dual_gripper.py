#!/usr/bin/env python3
from __future__ import annotations

import argparse
import time
import threading
from typing import Dict

from changingtek_gripper_adapter import ChangingTekGripperAdapter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Test two ChangingTek grippers control.")

    parser.add_argument(
        "--ports",
        nargs="+",
        default=["/dev/ttyUSB0", "/dev/ttyUSB1"],
        help="Serial ports for grippers. Example: --ports /dev/ttyUSB0 /dev/ttyUSB1",
    )

    parser.add_argument(
        "--slave-ids",
        nargs="+",
        type=int,
        default=[1, 1],
        help="Modbus slave ids for each gripper. Example: --slave-ids 1 1",
    )

    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baudrate.")
    parser.add_argument("--timeout", type=float, default=0.3, help="Serial timeout seconds.")
    parser.add_argument("--speed", type=int, default=30, help="Move speed percent, 0-100.")
    parser.add_argument("--force", type=int, default=50, help="Move force percent, 0-100.")
    parser.add_argument("--sleep", type=float, default=0.5, help="Sleep after move.")

    return parser.parse_args()


def print_status(name: str, gripper: ChangingTekGripperAdapter) -> None:
    try:
        print(
            f"[{name}] position={gripper.position:.2f}  current={gripper.ForceValue:.2f}"
        )
    except Exception as e:
        print(f"[{name}] failed to read status: {e}")


def move_one(
    name: str,
    gripper: ChangingTekGripperAdapter,
    target: float,
    speed: int,
    force: int,
) -> None:
    try:
        gripper.move(pos=target, speed=speed, force=force)
        print(f"[{name}] move command sent: target={target:.1f}")
    except Exception as e:
        print(f"[{name}] move failed: {e}")


def move_all(
    grippers: Dict[str, ChangingTekGripperAdapter],
    target: float,
    speed: int,
    force: int,
) -> None:
    threads = []

    for name, gripper in grippers.items():
        t = threading.Thread(
            target=move_one,
            args=(name, gripper, target, speed, force),
            daemon=True,
        )
        t.start()
        threads.append(t)

    for t in threads:
        t.join()


def main() -> None:
    args = parse_args()

    if len(args.ports) != len(args.slave_ids):
        raise ValueError(
            f"--ports and --slave-ids must have the same length, "
            f"got {len(args.ports)} ports and {len(args.slave_ids)} slave ids"
        )

    grippers: Dict[str, ChangingTekGripperAdapter] = {}

    for idx, port in enumerate(args.ports):
        slave_id = args.slave_ids[idx]
        name = f"gripper{idx}:{port}"

        print(f"Connecting {name}, slave_id={slave_id}")

        grippers[name] = ChangingTekGripperAdapter(
            port=port,
            slave_id=slave_id,
            baudrate=args.baudrate,
            timeout=args.timeout,
            init_action=None,
        )

    print()
    print("ChangingTek dual gripper test")
    print("Commands:")
    print("  o / open      -> open, target=90")
    print("  c / close     -> close, target=0")
    print("  m / middle    -> middle, target=45")
    print("  p / pos       -> print positions")
    print("  q / quit      -> quit")
    print("  number        -> target from 0 to 90")
    print()
    print("Meaning: 0=closed, 90=open")
    print()

    for name, gripper in grippers.items():
        print_status(name, gripper)

    while True:
        cmd = input("> ").strip().lower()

        if not cmd:
            continue

        if cmd in {"q", "quit", "exit"}:
            break

        if cmd in {"p", "pos", "position"}:
            for name, gripper in grippers.items():
                print_status(name, gripper)
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

        print(f"Move all grippers to {target:.1f}")
        move_all(
            grippers=grippers,
            target=target,
            speed=args.speed,
            force=args.force,
        )

        time.sleep(args.sleep)

        for name, gripper in grippers.items():
            print_status(name, gripper)


if __name__ == "__main__":
    main()