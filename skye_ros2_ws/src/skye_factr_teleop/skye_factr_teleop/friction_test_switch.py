# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Interactive off/on switch for one running stage-7.2 friction test."""

import argparse
import select
import sys
import time


class SwitchSession:
    def __init__(self):
        self.mode = None
        self.phase = None
        self.pending = None
        self.pending_at = None

    def set_mode(self, value):
        self.mode = value
        if value != "FRICTION_TEST":
            self.pending = None
            self.pending_at = None

    def set_phase(self, value):
        self.phase = value
        if value == self.pending or value == "idle":
            self.pending = None
            self.pending_at = None

    def next_phase(self):
        if self.mode != "FRICTION_TEST" or self.pending is not None:
            return None
        return {"off": "on", "on": "off"}.get(self.phase)

    def mark_sent(self, value, now):
        self.pending = value
        self.pending_at = now

    def expire_pending(self, now):
        if self.pending_at is not None and now - self.pending_at > 1.0:
            self.pending = None
            self.pending_at = None
            return True
        return False


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    args = parser.parse_args()
    if not sys.stdin.isatty():
        parser.error("interactive switch requires a terminal")

    from rclpy import init, ok, shutdown, spin_once
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    state_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    event_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.VOLATILE)
    session = SwitchSession()

    class SwitchNode(Node):
        def __init__(self):
            super().__init__(f"phase7_2_friction_switch_{args.side}")
            self.output_pub = self.create_publisher(
                String, f"/mode/{args.side}/friction_test_output", event_qos,
            )
            self.stop_pub = self.create_publisher(String, "/mode/switch_stop", event_qos)
            self.create_subscription(String, "/teleop/state", self.on_mode, state_qos)
            self.create_subscription(String,
                f"/leader_arm/{args.side}/friction_test_phase", self.on_phase, state_qos)

        def on_mode(self, message):
            if message.data != session.mode:
                session.set_mode(message.data)
                print(f"模式: {message.data}", flush=True)

        def on_phase(self, message):
            if message.data != session.phase:
                session.set_phase(message.data)
                print(f"输出阶段: {message.data}", flush=True)

    init()
    node = SwitchNode()
    print("回车切换 off/on；输入 s 后回车停止测试；Ctrl+C 仅退出切换工具。", flush=True)
    try:
        while True:
            spin_once(node, timeout_sec=0.05)
            if session.expire_pending(time.monotonic()):
                print("未收到阶段切换确认，可重试；请查看阶段监视窗口。", flush=True)
            if not select.select([sys.stdin], [], [], 0)[0]:
                continue
            action = sys.stdin.readline()
            if not action:
                break
            if action.strip().lower() == "s":
                if node.stop_pub.get_subscription_count() == 0:
                    print("未发现测试节点的停止订阅，未发送停止请求。", flush=True)
                    continue
                node.stop_pub.publish(String(data="switch_stop"))
                print("已发送停止请求；请确认模式回到 IDLE。", flush=True)
            elif not action.strip():
                target = session.next_phase()
                if target is None:
                    print("当前无法切换：需处于 FRICTION_TEST 的 off/on 阶段且上一请求已确认。",
                          flush=True)
                elif node.output_pub.get_subscription_count() == 0:
                    print("未发现测试节点的切换订阅，未发送请求。", flush=True)
                else:
                    node.output_pub.publish(String(data=target))
                    session.mark_sent(target, time.monotonic())
                    print(f"已请求切换到 {target}；等待阶段确认。", flush=True)
            else:
                print("仅接受回车切换，或输入 s 后回车停止。", flush=True)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if ok():
            shutdown()


if __name__ == "__main__":
    main()
