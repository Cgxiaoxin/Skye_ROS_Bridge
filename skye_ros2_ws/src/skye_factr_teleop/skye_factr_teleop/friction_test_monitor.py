# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Show the selected joint's absolute commanded current on one terminal line."""

import argparse
import math
import sys
import time


class AbsoluteCommandCurrent:
    def __init__(self, joint_id):
        if not 1 <= joint_id <= 7:
            raise ValueError("joint ID must be 1..7")
        self.joint_id = joint_id
        self.value_ma = None
        self.received_at = None

    def update(self, values, now):
        if len(values) != 8 or not all(math.isfinite(value) for value in values):
            return
        self.value_ma = abs(values[self.joint_id - 1])
        self.received_at = now

    def line(self, now):
        if self.received_at is None:
            value = "等待命令电流消息"
        elif now - self.received_at > 1.0:
            value = "-- mA（超过 1 s 无新消息）"
        else:
            value = f"{self.value_ma:.2f} mA"
        return f"J{self.joint_id} |命令电流| = {value}"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint-id", type=int, choices=range(1, 8), required=True)
    args = parser.parse_args()

    from rclpy import init, shutdown, spin
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Float64MultiArray

    qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                     reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    display = AbsoluteCommandCurrent(args.joint_id)

    class MonitorNode(Node):
        def __init__(self):
            super().__init__("phase7_2_friction_abs_current")
            self.create_subscription(
                Float64MultiArray,
                f"/leader_arm/{args.side}/friction_test_current_ma",
                lambda message: display.update(message.data, time.monotonic()),
                qos,
            )
            self.create_timer(0.05, self.draw)

        def draw(self):
            # CR + erase line updates the same value instead of scrolling output.
            sys.stdout.write("\r\033[2K" + display.line(time.monotonic()))
            sys.stdout.flush()

    init()
    node = MonitorNode()
    try:
        spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        sys.stdout.write("\n")
        sys.stdout.flush()
        node.destroy_node()
        shutdown()


if __name__ == "__main__":
    main()
