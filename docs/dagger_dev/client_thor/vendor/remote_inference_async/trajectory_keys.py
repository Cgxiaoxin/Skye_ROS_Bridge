#!/usr/bin/env python3
"""Keyboard controller for the Tianji right-arm trajectory recorder."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
import termios
import tty

from remote_inference_async.trajectory_recorder import (
    DEFAULT_CONTROL_SOCKET,
    RecorderControlError,
    send_control_command,
)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="按键切分天机机器人右臂实时推理轨迹"
    )
    parser.add_argument(
        "--socket",
        type=Path,
        default=DEFAULT_CONTROL_SOCKET,
        help="推理客户端创建的控制 socket",
    )
    parser.add_argument("--label", default="", help="本次分段的可选标签")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--start", action="store_true", help="立即开始一个分段")
    mode.add_argument("--stop", action="store_true", help="立即结束当前分段")
    mode.add_argument("--status", action="store_true", help="只显示当前状态")
    return parser


def _request(command: str, args: argparse.Namespace) -> dict:
    return send_control_command(
        command,
        control_socket=args.socket,
        label=args.label if command == "start" else "",
    )


def _print_status(response: dict) -> None:
    writer_error = response.get("writer_error")
    if writer_error:
        print("\n[记录器写盘故障] 当前片段已停止采样", flush=True)
        print(f"  原因：{writer_error}", flush=True)
        active = response.get("active_segment", {})
        if active:
            print(f"  未完成文件：{active.get('trajectory_path', '未知')}", flush=True)
        return
    if response.get("recording"):
        active = response.get("active_segment", {})
        print("\n[正在记录]", flush=True)
        print(f"  文件：{active.get('trajectory_path', '未知')}", flush=True)
        print(
            "  已入队/已写入/丢弃："
            f"{active.get('events_queued', 0)}/"
            f"{active.get('events_written', 0)}/"
            f"{active.get('events_dropped', 0)}",
            flush=True,
        )
    else:
        print("\n[等待开始] 当前没有正在记录的分段", flush=True)
        print(f"  保存目录：{response.get('output_dir', '未知')}", flush=True)


def _print_stop(response: dict) -> None:
    segment = response.get("segment", {})
    print("\n[分段已保存]", flush=True)
    print(f"  轨迹：{segment.get('trajectory_path', '未知')}", flush=True)
    print(f"  摘要：{segment.get('summary_path', '未知')}", flush=True)
    print(f"  时长：{float(segment.get('duration_seconds', 0.0)):.3f} 秒", flush=True)
    print(
        "  写入/丢弃："
        f"{segment.get('events_written', 0)}/"
        f"{segment.get('events_dropped', 0)}",
        flush=True,
    )


def _single_command(args: argparse.Namespace) -> int | None:
    command = None
    if args.start:
        command = "start"
    elif args.stop:
        command = "stop"
    elif args.status:
        command = "status"
    if command is None:
        return None
    try:
        response = _request(command, args)
    except RecorderControlError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1
    if command == "stop":
        _print_stop(response)
    else:
        _print_status(response)
    return 0


def _interactive(args: argparse.Namespace) -> int:
    if not sys.stdin.isatty():
        print("错误：交互按键模式需要在终端中运行", file=sys.stderr)
        return 2
    try:
        _print_status(_request("status", args))
    except RecorderControlError as exc:
        print(f"错误：{exc}", file=sys.stderr)
        return 1

    print("\n按键：S=开始，E=结束，R=刷新状态，Q=退出按键工具", flush=True)
    print(
        "说明：Q 只退出本工具，不会停止机器人；正在记录的分段会继续。",
        flush=True,
    )
    descriptor = sys.stdin.fileno()
    previous = termios.tcgetattr(descriptor)
    try:
        tty.setcbreak(descriptor)
        while True:
            key = sys.stdin.read(1).lower()
            if not key:
                print("\n终端输入已关闭。", flush=True)
                return 0
            if key == "q":
                print("\n已退出按键工具。", flush=True)
                return 0
            if key not in {"s", "e", "r"}:
                continue
            command = {"s": "start", "e": "stop", "r": "status"}[key]
            try:
                response = _request(command, args)
            except RecorderControlError as exc:
                print(f"\n错误：{exc}", flush=True)
                continue
            if command == "stop":
                _print_stop(response)
            else:
                _print_status(response)
    except KeyboardInterrupt:
        print(
            "\n已退出按键工具；机器人和正在记录的片段不受影响。",
            flush=True,
        )
        return 130
    finally:
        termios.tcsetattr(descriptor, termios.TCSADRAIN, previous)


def main() -> int:
    args = build_parser().parse_args()
    result = _single_command(args)
    return _interactive(args) if result is None else result


if __name__ == "__main__":
    raise SystemExit(main())
