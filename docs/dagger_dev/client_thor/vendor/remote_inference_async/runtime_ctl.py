#!/usr/bin/env python3
"""Command-line controller for a persistent Tianji robot runtime."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

from remote_inference_async.runtime_control import (
    DEFAULT_RUNTIME_CONTROL_SOCKET,
    RuntimeControlError,
    send_runtime_command,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="控制常驻天机机器人推理进程")
    parser.add_argument(
        "--socket",
        type=Path,
        default=DEFAULT_RUNTIME_CONTROL_SOCKET,
        help="prepare_robot.sh 创建的本地控制 socket",
    )
    subparsers = parser.add_subparsers(dest="command", required=True)
    subparsers.add_parser("status", help="查看当前运行状态")
    start = subparsers.add_parser("start", help="从 HOLD 开始策略推理")
    start.add_argument("--task", default=None, help="可选的新任务描述")
    switch = subparsers.add_parser("switch", help="清除旧动作并切换策略任务")
    switch.add_argument("--task", required=True, help="新的任务描述")
    subparsers.add_parser("pause", help="停止推理并保持当前反馈位置")
    subparsers.add_parser("shutdown", help="关闭常驻进程并执行原有安全退出")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    command = {
        "start": "start_policy",
        "switch": "switch_policy",
    }.get(args.command, args.command)
    try:
        response = send_runtime_command(
            command,
            control_socket=args.socket,
            task=getattr(args, "task", None),
        )
        if args.command not in {"status", "shutdown"}:
            previous_generation = int(response.get("generation", -1))
            expected_state = "hold" if args.command == "pause" else "policy"
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                status = send_runtime_command(
                    "status",
                    control_socket=args.socket,
                )
                if (
                    int(status.get("generation", -1)) > previous_generation
                    and status.get("state") == expected_state
                ):
                    response = {"ok": True, "applied": command, **status}
                    break
                time.sleep(0.05)
    except RuntimeControlError as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 1
    print(json.dumps(response, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
