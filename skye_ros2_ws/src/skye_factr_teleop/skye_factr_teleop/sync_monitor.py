# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Display synchronization feedback around the operator's ros2 pub --once."""

import json
import os
import shutil
import subprocess
import sys
import time


class SyncFeedback:
    def __init__(self, sides, started_at, output=print):
        self.sides = set(sides)
        self.started_at = started_at
        self.outcomes = {}
        self.output = output

    def receive(self, message):
        packet = json.loads(message.data)
        side = packet["side"]
        # The progress topic retains its last message for late subscribers.
        # Ignore feedback from attempts before this command.
        if side not in self.sides or packet["stamp_s"] < self.started_at:
            return
        self.output(packet["text"], flush=True)
        self.outcomes[side] = packet["outcome"]

    @property
    def done(self):
        return all(self.outcomes.get(side) in ("synced", "failed", "stopped")
                   for side in self.sides)

    @property
    def succeeded(self):
        return self.done and all(value == "synced" for value in self.outcomes.values())


def monitor(command, sides, *, connect_timeout=5.0, timeout=90.0):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String

    rclpy.init(args=[])
    node = Node(f"skye_sync_monitor_{os.getpid()}")
    feedback = SyncFeedback(sides, float("inf"))
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    topics = [f"/leader_arm/{side}/sync_progress" for side in sides]
    subscriptions = [node.create_subscription(String, topic, feedback.receive, qos)
                     for topic in topics]
    publisher = None
    try:
        deadline = time.monotonic() + connect_timeout
        while not all(node.count_publishers(topic) for topic in topics):
            if time.monotonic() >= deadline:
                print("未找到所选侧的同步反馈，请检查终端 A、侧别和 ROS_DOMAIN_ID；未发送同步指令。",
                      file=sys.stderr)
                return 1
            rclpy.spin_once(node, timeout_sec=0.1)

        feedback.started_at = time.time()
        print("同步反馈（Ctrl+C 仅退出显示；停止小臂请发送 /mode/switch_stop）：", flush=True)
        # Subscribe before invoking the real CLI so fast replies are not lost.
        publisher = subprocess.Popen(command)
        deadline = time.monotonic() + timeout
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            result = publisher.poll()
            if result is not None:
                if result != 0:
                    return result
                if feedback.done:
                    return 0 if feedback.succeeded else 1
            if time.monotonic() >= deadline:
                print("等待同步反馈超时；请检查终端 A，停止小臂请发送 /mode/switch_stop。",
                      file=sys.stderr)
                return 1
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n已退出同步显示；停止小臂请发送 /mode/switch_stop。", flush=True)
        return 130
    finally:
        if publisher is not None and publisher.poll() is None:
            publisher.terminate()
            publisher.wait()
        node.destroy_node()
        rclpy.try_shutdown()


def main(argv=None):
    import yaml

    args = list(sys.argv[1:] if argv is None else argv)
    command = [shutil.which("ros2") or "ros2", *args]
    try:
        payload = yaml.safe_load(args[-1]) if args else None
    except yaml.YAMLError:
        payload = None
    if (len(args) != 6 or args[:5] != ["topic", "pub", "--once", "/mode/switch_sync",
                                      "std_msgs/msg/String"]
            or payload != {"data": "switch_sync"}):
        return subprocess.call(command)
    sides = [side for side in ("left", "right") if os.environ.get(f"{side.upper()}_CONFIG")]
    if not sides:
        print("请先 source setup_factr_teleop_env.sh 选择同步侧别。", file=sys.stderr)
        return 1
    return monitor(command, sides)


if __name__ == "__main__":
    sys.exit(main())
