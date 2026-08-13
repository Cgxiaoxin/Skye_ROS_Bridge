#!/usr/bin/env python3
"""双终端监听 — 同时监听 ARM0 + ARM1 CAN 数据.

用法:
  python3 test/robot/test_terminal_listen.py
"""

import sys, os, time
_current_dir = os.path.dirname(os.path.abspath(__file__))
_parent_dir = os.path.join(_current_dir, "..", "..")
sys.path.insert(0, _parent_dir)

from PYTHON_SDK.GentoRobot import GentoRobot

robot = GentoRobot()
robot.link(6, 6, 7, 190)
print("监听 ARM0 + ARM1 (Ctrl+C 退出)\n")
t0 = time.time()
count = [0, 0]
try:
    while True:
        for arm in (0, 1):
            ret = robot.terminal_get(arm, timeout=50)
            if ret and ret[1]:
                cid = int.from_bytes(ret[1][0:4], "little")
                elapsed = time.time() - t0
                label = "L" if arm == 0 else "R"
                print(f"[{elapsed:6.1f}s] ARM{arm}({label}) CAN ID=0x{cid:02X}  data={ret[1][4:].hex()}")
                count[arm] += 1
        time.sleep(0.01)
except KeyboardInterrupt:
    elapsed = time.time() - t0
    print(f"\n{elapsed:.0f}s: ARM0={count[0]}帧  ARM1={count[1]}帧")
finally:
    robot.unlink()
