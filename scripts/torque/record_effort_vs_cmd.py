#!/usr/bin/env python3
"""录制大臂 effort 与小臂/大臂指令时序，区分力环抖 vs 位环抖。

同机、同 ROS_DOMAIN_ID（默认 21），驱动与 FACTR 正常运行时使用（不独占 SDK）。

默认订阅:
  /gento/{left,right}_joint_states  — 发布后的 effort（含死区/EMA/TELEOP 门控）
  /gento/{left,right}_joint_control — 小臂→大臂位置指令
  /{left,right}_leader_arm/current_state — 小臂实测（位环对照）
  /teleop/state                     — FACTR 模式

用法（仓库根）:
  export ROS_DOMAIN_ID=21
  unset ROS_LOCALHOST_ONLY
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
  /usr/bin/python3 scripts/torque/record_effort_vs_cmd.py --side left --duration 20
"""

from __future__ import annotations

import argparse
import csv
import os
import signal
import time
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[2]


def _apply_ros_env() -> None:
    """Match host skye scripts: domain 21 + FastDDS no-SHM (before rclpy.init)."""
    os.environ.setdefault("ROS_DOMAIN_ID", "21")
    os.environ.pop("ROS_LOCALHOST_ONLY", None)
    os.environ.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    xml = REPO_ROOT / "marvin_ws" / "fastrtps_no_shm.xml"
    if xml.is_file():
        os.environ.setdefault("FASTRTPS_DEFAULT_PROFILES_FILE", str(xml))


_apply_ros_env()

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import String


def _state_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )


def _cmd_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )


def _teleop_qos() -> QoSProfile:
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class EffortCmdRecorder(Node):
    def __init__(self, side: str, output: Path) -> None:
        super().__init__("effort_vs_cmd_recorder")
        self._side = side
        self._output = output
        self._teleop_state = ""
        self._rows: list[dict[str, object]] = []
        self._effort_topic_hits = 0
        self._warned_empty_effort = False
        prefix = "left" if side == "left" else "right"
        self._effort_topic = f"/gento/{prefix}_joint_states"
        self._cmd_topic = f"/gento/{prefix}_joint_control"
        self._leader_topic = f"/{prefix}_leader_arm/current_state"

        self.create_subscription(
            JointState, self._effort_topic, self._on_effort, _state_qos()
        )
        self.create_subscription(
            JointState, self._cmd_topic, self._on_cmd, _cmd_qos()
        )
        self.create_subscription(
            JointState, self._leader_topic, self._on_leader, _state_qos()
        )
        self.create_subscription(
            String, "/teleop/state", self._on_teleop, _teleop_qos()
        )

        self._last_cmd: list[float] | None = None
        self._last_leader: list[float] | None = None
        self._t0 = time.monotonic()

    def _on_teleop(self, msg: String) -> None:
        self._teleop_state = msg.data.strip()

    def _on_cmd(self, msg: JointState) -> None:
        if len(msg.position) >= 7:
            self._last_cmd = [float(x) for x in msg.position[:7]]

    def _on_leader(self, msg: JointState) -> None:
        if len(msg.position) >= 7:
            self._last_leader = [float(x) for x in msg.position[:7]]

    def _on_effort(self, msg: JointState) -> None:
        self._effort_topic_hits += 1
        if len(msg.position) < 7 and len(msg.effort) < 7:
            return
        if len(msg.effort) < 7 and not self._warned_empty_effort:
            self._warned_empty_effort = True
            print(
                f"warn: {self._effort_topic} effort len={len(msg.effort)} "
                "(need rebuilt skye_robot_driver); recording zeros"
            )
        t = time.monotonic() - self._t0
        row: dict[str, object] = {
            "t": f"{t:.4f}",
            "teleop_state": self._teleop_state,
        }
        for i in range(7):
            if i < len(msg.effort):
                row[f"effort_j{i+1}"] = f"{float(msg.effort[i]):.6f}"
            else:
                row[f"effort_j{i+1}"] = "0.000000"
            if self._last_cmd is not None:
                row[f"cmd_j{i+1}"] = f"{self._last_cmd[i]:.6f}"
            else:
                row[f"cmd_j{i+1}"] = ""
            if self._last_leader is not None:
                row[f"leader_j{i+1}"] = f"{self._last_leader[i]:.6f}"
            else:
                row[f"leader_j{i+1}"] = ""
        self._rows.append(row)

    def write(self) -> None:
        if not self._rows:
            print(
                "no samples recorded — check driver is up and env matches:\n"
                f"  ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')}\n"
                f"  expected topic: {self._effort_topic}\n"
                "  tip: ros2 topic hz /gento/left_joint_states"
            )
            return
        fieldnames = list(self._rows[0].keys())
        self._output.parent.mkdir(parents=True, exist_ok=True)
        with self._output.open("w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._rows)
        print(f"wrote {len(self._rows)} rows → {self._output}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="CSV path (default: /tmp/effort_vs_cmd_<side>.csv)",
    )
    args = parser.parse_args()
    output = args.output or Path(f"/tmp/effort_vs_cmd_{args.side}.csv")

    # Avoid rclpy's SIGINT handler calling shutdown before our finally.
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    print(
        f"recording side={args.side} duration={args.duration}s "
        f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID')} → {output}"
    )

    rclpy.init()
    node = EffortCmdRecorder(args.side, output)
    end = time.monotonic() + args.duration
    last_status = 0.0
    interrupted = False
    try:
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.05)
            now = time.monotonic()
            if now - last_status >= 2.0:
                last_status = now
                left = max(0.0, end - now)
                print(
                    f"  … {len(node._rows)} samples "
                    f"(topic hits={node._effort_topic_hits}) "
                    f"{left:.0f}s left; teleop={node._teleop_state or '?'}"
                )
    except KeyboardInterrupt:
        interrupted = True
        print("interrupted")
    finally:
        try:
            node.write()
        except Exception as exc:  # noqa: BLE001 — always try cleanup
            print(f"write failed: {exc}")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 130 if interrupted else 0


if __name__ == "__main__":
    raise SystemExit(main())
