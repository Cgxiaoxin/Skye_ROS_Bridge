#!/usr/bin/env python3
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Finalize one friction run after independent Torque Disable readback."""

import argparse
import json
from pathlib import Path

from skye_factr_teleop.friction_test_recorder import finalize_friction_record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--readback", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    if not args.record.is_file():
        parser.error(f"记录器 JSON 不存在：{args.record}；请检查终端 A 的启动与结束输出")
    if not args.readback.is_file():
        parser.error(f"Torque Disable 回读文件不存在：{args.readback}")
    record = json.loads(args.record.read_text(encoding="utf-8"))
    readback = json.loads(args.readback.read_text(encoding="utf-8"))
    baseline = (json.loads(args.baseline.read_text(encoding="utf-8"))
                if args.baseline else None)
    if record.get("enabled") and not record.get("paired") and baseline is None:
        raise ValueError("静摩擦开启轮必须提供已完成的关闭对照轮 --baseline")
    answer = input("测试节点正常退出，且回读前未断电或重启总线？[yes/no]: ").strip().lower()
    if answer not in ("yes", "no"):
        raise ValueError("请输入 yes 或 no；记录尚未修改")
    result = finalize_friction_record(
        record, readback, process_exited_normally=answer == "yes",
        readback_path=args.readback.resolve(), baseline=baseline,
        baseline_path=args.baseline.resolve() if args.baseline else None,
    )
    args.record.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(f"最终判定: {result['assessment']}，记录: {args.record}")


if __name__ == "__main__":
    main()
