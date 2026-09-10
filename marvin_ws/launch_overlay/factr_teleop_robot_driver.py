#!/usr/bin/env python3
import os
import sys

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)

try:
    from dynamixel_serial_guard import install as install_serial_guard

    install_serial_guard()
except Exception as exc:
    print(f"[serial_guard] failed to install: {exc}", flush=True)

from factr_teleop_robot_driver_bin import main


if __name__ == "__main__":
    main()
