#!/usr/bin/env python3
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Finish one limit-test record after independent Torque Disable readback."""

import argparse
import json
from pathlib import Path

from skye_factr_teleop.limit_test_recorder import finalize_limit_record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--record", type=Path, required=True)
    parser.add_argument("--readback", type=Path, required=True)
    args = parser.parse_args()
    record = json.loads(args.record.read_text(encoding="utf-8"))
    readback = json.loads(args.readback.read_text(encoding="utf-8"))
    answer = input("测试节点是否正常退出且回读前未断电或重启总线？[yes/no]: ").strip().lower()
    if answer not in ("yes", "no"):
        raise ValueError("请输入 yes 或 no；记录尚未修改")
    result = finalize_limit_record(
        record, readback, process_exited_normally=(answer == "yes"),
        readback_path=args.readback.resolve(),
    )
    args.record.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n",
                           encoding="utf-8")
    print(f"最终判定: {result['assessment']}，记录: {args.record}")


if __name__ == "__main__":
    main()
