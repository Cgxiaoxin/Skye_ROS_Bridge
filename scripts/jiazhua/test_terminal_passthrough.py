#!/usr/bin/env python3
"""手工 CANFD 终端直通测试 — 不依赖 controller_log_capture.

用法:
  python3 test/robot/test_terminal_passthrough.py
  python3 test/robot/test_terminal_passthrough.py --arm 1            # 右臂
  python3 test/robot/test_terminal_passthrough.py --arm 0 --listen   # 左臂监听
"""

import sys
import os
import argparse

_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.join(_current_dir, "..", "..")
if _parent_dir not in sys.path:
    sys.path.insert(0, _parent_dir)

from PYTHON_SDK.GentoRobot import GentoRobot

ROBOT_IP = "6.6.7.190"


def normalize_hex(value: str) -> str:
    value = value.strip()
    if not value:
        return ""
    try:
        return bytes.fromhex(value).hex(" ").upper()
    except ValueError as exc:
        raise ValueError(f"需要十六进制字节对, 如 01 A0 FF: {exc}") from exc


def main():
    p = argparse.ArgumentParser(description="手工 CANFD 终端直通测试")
    p.add_argument("--ip", default=ROBOT_IP)
    p.add_argument("--arm", type=int, default=0,
                   help="终端类型: 0=左臂 1=右臂 (默认 0)")
    p.add_argument("--listen", action="store_true",
                   help="持续监听模式")
    p.add_argument("--timeout", type=int, default=1000,
                   help="terminal_get 超时 ms")
    args = p.parse_args()

    robot = GentoRobot()
    try:
        addr = args.ip.split(".")
        ret = robot.link(*[int(b) for b in addr])
        if ret < 0:
            raise RuntimeError(f"link failed: {ret}")
        print(f"已连接 {args.ip}")

        if args.listen:
            import time
            print(f"监听 ARM{args.arm} (Ctrl+C 退出)...")
            while True:
                ret = robot.terminal_get(args.arm, timeout=args.timeout)
                if ret and ret[1]:
                    can_id = int.from_bytes(ret[1][0:4], "little")
                    print(f"ARM{args.arm}: CAN ID=0x{can_id:02X}  "
                          f"data={ret[1][4:].hex()}")
                time.sleep(0.05)
        else:
            robot.terminal_clear(args.arm)
            print(f"ARM{args.arm} 缓存已清空")

            raw = input("Hex payload (如 01000000FFFFFFFFFFFFFFFC), 回车跳过: ")
            payload = normalize_hex(raw)
            if payload:
                print(f"Payload: {payload}")
                ans = input("输入 SEND 发送: ").strip()
                if ans == "SEND":
                    ret = robot.terminal_set(args.arm, 1, payload)
                    print(f"terminal_set → {ret}")
                else:
                    print("已取消")

            print(f"读 ARM{args.arm} 一次...")
            ret = robot.terminal_get(args.arm, timeout=args.timeout)
            print(f"terminal_get → {ret}")

    except KeyboardInterrupt:
        pass
    finally:
        robot.unlink()
        robot.cleanup()


if __name__ == "__main__":
    main()
