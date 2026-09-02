#!/usr/bin/env python3
"""命令小臂走到绝对零位（绕过 sync），用于隔离测试 Dynamixel 走位。

用法（托住小臂，确保前方无障碍）:
  export ROS_DOMAIN_ID=21
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
  /usr/bin/python3 scripts/debug/move_small_arm_zero.py --side right
  /usr/bin/python3 scripts/debug/move_small_arm_zero.py --side right --mode telepose
  /usr/bin/python3 scripts/debug/move_small_arm_zero.py --side right --mode sync

说明:
  - --mode stop: switch_stop 后发 target（FACTR 在 IDLE 可能忽略 target）
  - --mode telepose: switch_telepose 后发 target（文档：定姿/测试用）
  - --mode sync: switch_sync，让小臂跟大臂 /gento/joint_states（大臂在 0 时应回零）
  - 读 /{side}_leader_arm/current_state = Dynamixel 编码器反馈，不是大臂。
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String


MODE_TOPICS = {
    "stop": "/mode/switch_stop",
    "telepose": "/mode/switch_telepose",
    "sync": "/mode/switch_sync",
    "teleop": "/mode/switch_teleop",
}


def _qos_reliable() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
    )


class MoveSmallArmZero(Node):
    def __init__(
        self,
        side: str,
        mode: str,
        target_topic: str,
        duration_s: float,
        rate_hz: float,
        positions: list[float],
    ) -> None:
        super().__init__("move_small_arm_zero")
        self._side = side
        self._mode = mode
        self._duration_s = duration_s
        self._rate_hz = rate_hz
        self._positions = positions
        self._leader_topic = f"/{side}_leader_arm/current_state"
        self._latest: JointState | None = None
        self._mode_topic = MODE_TOPICS[mode]

        self._mode_pub = self.create_publisher(String, self._mode_topic, 10)
        self._stop_pub = self.create_publisher(String, "/mode/switch_stop", 10)
        per_side_target = f"/{side}_leader_arm/target_joint_state"
        resolved_target = target_topic or (
            per_side_target if mode in ("stop", "telepose") else "/leader_arm/target_joint_state"
        )
        self._target_topic = resolved_target
        self._target_pub = self.create_publisher(
            JointState, resolved_target, _qos_reliable()
        )
        self.create_subscription(
            JointState, self._leader_topic, self._on_leader, _qos_reliable()
        )

    def _on_leader(self, msg: JointState) -> None:
        self._latest = msg

    def _publish_mode(self, topic_suffix: str, repeats: int = 5) -> None:
        msg = String()
        msg.data = topic_suffix
        pub = (
            self._stop_pub
            if topic_suffix == "switch_stop"
            else self._mode_pub
        )
        for _ in range(repeats):
            pub.publish(msg)
            time.sleep(0.05)

    def _publish_stop(self) -> None:
        if self._mode == "sync":
            print("      先 switch_stop 再 switch_sync（避免已在 SYNCED 时不再走位）")
            self._publish_mode("switch_stop")
            time.sleep(1.0)
            self._publish_mode("switch_sync")
            return
        self._publish_mode(self._mode_topic.rsplit("/", 1)[-1])

    def _publish_target(self) -> None:
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.position = list(self._positions)
        self._target_pub.publish(msg)

    def _fmt_deg(self) -> str:
        if self._latest is None or not self._latest.position:
            return "(no feedback)"
        parts = [
            f"J{i + 1}={math.degrees(float(v)):+.1f}°"
            for i, v in enumerate(self._latest.position[:7])
        ]
        return " ".join(parts)

    def run(self) -> int:
        print(f"[1/3] {self._mode_topic} …")
        self._publish_stop()
        time.sleep(0.5)

        if self._mode == "sync":
            print(
                f"[2/3] sync 模式：让小臂跟大臂 /gento/joint_states"
                f"（右臂 offset=7；大臂在 0 时小臂应回零）"
            )
        else:
            print(
                f"[2/3] 向 {self._target_topic} 发布 7 轴目标 (rad): "
                f"{self._positions}"
            )
        print(f"      读反馈: {self._leader_topic} (Dynamixel 编码器，不是大臂)")
        if self._target_topic == "/leader_arm/target_joint_state":
            print(
                "      注意: 该 topic 左右 factr 都订阅；请重启 factr 后用"
                " /right_leader_arm/target_joint_state（launch 已补 remap）。"
            )

        deadline = time.monotonic() + self._duration_s
        period = 1.0 / max(self._rate_hz, 1.0)
        last_print = 0.0
        while time.monotonic() < deadline:
            if self._mode != "sync":
                self._publish_target()
            rclpy.spin_once(self, timeout_sec=0.01)
            now = time.monotonic()
            if now - last_print >= 0.5:
                print(f"      feedback {self._side}: {self._fmt_deg()}")
                last_print = now
            time.sleep(period)

        rclpy.spin_once(self, timeout_sec=0.1)
        print(f"[3/3] 结束。最终 {self._side} 小臂反馈: {self._fmt_deg()}")
        return 0


def main() -> int:
    parser = argparse.ArgumentParser(description="命令小臂绝对零位（绕过 sync）")
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument(
        "--mode",
        choices=tuple(MODE_TOPICS),
        default="telepose",
        help="stop/telepose 发 target；sync 走大臂反馈对齐",
    )
    parser.add_argument(
        "--topic",
        default="",
        help="target topic；默认 /leader_arm/target_joint_state",
    )
    parser.add_argument("--duration", type=float, default=10.0, help="持续发布时间 (s)")
    parser.add_argument("--rate", type=float, default=20.0, help="发布频率 Hz")
    parser.add_argument(
        "--positions",
        default="0,0,0,0,0,0,0",
        help="7 轴目标 rad，逗号分隔",
    )
    args = parser.parse_args()

    try:
        positions = [float(x.strip()) for x in args.positions.split(",")]
    except ValueError:
        print("ERROR: --positions 格式错误", file=sys.stderr)
        return 2
    if len(positions) != 7:
        print(f"ERROR: 需要 7 个关节，收到 {len(positions)} 个", file=sys.stderr)
        return 2

    target_topic = args.topic or ""

    print("=== 小臂零位测试（托住小臂，确认前方无障碍）===")
    rclpy.init()
    node = MoveSmallArmZero(
        side=args.side,
        mode=args.mode,
        target_topic=target_topic,
        duration_s=args.duration,
        rate_hz=args.rate,
        positions=positions,
    )
    try:
        return node.run()
    except KeyboardInterrupt:
        print("\n中断")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
