#!/usr/bin/env python3
"""1 Hz 对照小臂 / 大臂关节角（内部仍订 250 Hz 最新帧）。

主机与 FACTR 同 ROS_DOMAIN_ID（默认 21），FastDDS 关 SHM。

用法:
  export ROS_DOMAIN_ID=21
  unset ROS_LOCALHOST_ONLY
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
  /usr/bin/python3 scripts/debug/watch_arm_joints.py
  /usr/bin/python3 scripts/debug/watch_arm_joints.py --hz 1
  # 默认写 scripts/debug/watch_arm_joints_YYYYmmdd_HHMMSS.log（1 Hz）
"""

from __future__ import annotations

import argparse
import math
import threading
import time
from pathlib import Path
from typing import TextIO

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

# 大臂 URDF J4（rad），便于看是否贴边
J4_BIG_MIN = -2.5307
J4_BIG_MAX = 1.0472

LEADER_CANDIDATES = {
    "left": (
        "/left_leader_arm/current_state",
        "/leader_arm/current_state",
        "/factr_teleop_left/leader_arm/current_state",
    ),
    "right": (
        "/right_leader_arm/current_state",
        "/leader_arm/current_state",
        "/factr_teleop_right/leader_arm/current_state",
    ),
}


def _qos_reliable() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )


def _qos_best_effort() -> QoSProfile:
    return QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
    )


def _deg(rad: float) -> float:
    return math.degrees(rad)


def _fmt7(vals, idx: int) -> str:
    if vals is None or idx >= len(vals):
        return "  ----"
    return f"{_deg(vals[idx]):7.1f}"


class Latest:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.msg: JointState | None = None
        self.stamp = 0.0
        self.topic = ""
        self.count = 0

    def store(self, topic: str, msg: JointState) -> None:
        with self._lock:
            self.msg = msg
            self.stamp = time.monotonic()
            self.topic = topic
            self.count += 1

    def snapshot(self):
        with self._lock:
            return self.msg, self.stamp, self.topic, self.count


class WatchArmJoints(Node):
    def __init__(self, print_hz: float, log_fp: TextIO | None) -> None:
        super().__init__("watch_arm_joints")
        self._print_hz = print_hz
        self._log_fp = log_fp
        self._big = Latest()
        self._cmd = {"left": Latest(), "right": Latest()}
        self._leader = {"left": Latest(), "right": Latest()}

        self.create_subscription(
            JointState, "/gento/joint_states", self._on_big, _qos_reliable()
        )
        self.create_subscription(
            JointState,
            "/gento/left_joint_control",
            lambda m: self._cmd["left"].store("/gento/left_joint_control", m),
            _qos_best_effort(),
        )
        self.create_subscription(
            JointState,
            "/gento/right_joint_control",
            lambda m: self._cmd["right"].store("/gento/right_joint_control", m),
            _qos_best_effort(),
        )
        for side, topics in LEADER_CANDIDATES.items():
            for topic in topics:
                self.create_subscription(
                    JointState,
                    topic,
                    lambda m, s=side, t=topic: self._on_leader(s, t, m),
                    _qos_reliable(),
                )

        period = 1.0 / max(print_hz, 0.1)
        self.create_timer(period, self._print)

    def _on_big(self, msg: JointState) -> None:
        self._big.store("/gento/joint_states", msg)

    def _on_leader(self, side: str, topic: str, msg: JointState) -> None:
        slot = self._leader[side]
        # 左右都可能订到同一个 /leader_arm/current_state；有专用 topic 时优先保留专用。
        _, _, current_topic, _ = slot.snapshot()
        dedicated = LEADER_CANDIDATES[side][0]
        if current_topic and current_topic == dedicated and topic != dedicated:
            return
        slot.store(topic, msg)

    def _emit(self, text: str) -> None:
        print(text, end="" if text.endswith("\n") else "\n")
        if self._log_fp is not None:
            self._log_fp.write(text if text.endswith("\n") else text + "\n")

    def _print(self) -> None:
        now = time.monotonic()
        big, big_t, _, big_n = self._big.snapshot()
        block: list[str] = [
            f"{time.strftime('%H:%M:%S')} big_n={big_n} age={_age(now, big_t)}",
            f"{'':6} {'J':>3} {'小臂°':>7} {'大臂°':>7} {'指令°':>7} "
            f"{'小-大°':>8}  note",
        ]
        for side, offset in (("left", 0), ("right", 7)):
            lead, lead_t, lead_topic, _ = self._leader[side].snapshot()
            cmd, cmd_t, _, _ = self._cmd[side].snapshot()
            lp = list(lead.position) if lead is not None else None
            bp = list(big.position) if big is not None else None
            cp = list(cmd.position) if cmd is not None else None
            src = lead_topic.split("/")[-2] + "/" + lead_topic.split("/")[-1] if lead_topic else "no-leader"
            block.append(
                f" {side:5} src={src} age={_age(now, lead_t)} "
                f"cmd_age={_age(now, cmd_t)}"
            )
            for i in range(7):
                small = lp[i] if lp is not None and i < len(lp) else None
                big_i = (
                    bp[offset + i]
                    if bp is not None and offset + i < len(bp)
                    else None
                )
                delta = ""
                note = ""
                if small is not None and big_i is not None:
                    d = _deg(small - big_i)
                    delta = f"{d:8.1f}"
                    if abs(d) > 15.0:
                        note = "diff>15"
                if i == 3 and big_i is not None:
                    if big_i <= J4_BIG_MIN + 0.02:
                        note = (note + " " if note else "") + "J4@min"
                    elif big_i >= J4_BIG_MAX - 0.02:
                        note = (note + " " if note else "") + "J4@max"
                block.append(
                    f"{'':6} {i + 1:>3} {_fmt7(lp, i)} {_fmt7(bp, offset + i) if bp else '  ----'} "
                    f"{_fmt7(cp, i)} {delta:>8}  {note}"
                )
        block.append("")
        self._emit("\n".join(block) + "\n")
        if self._log_fp is not None:
            self._log_fp.flush()


def _age(now: float, stamp: float) -> str:
    if stamp <= 0.0:
        return "n/a"
    return f"{now - stamp:.2f}s"


def main() -> int:
    parser = argparse.ArgumentParser(description="1Hz 对照小臂/大臂关节角")
    parser.add_argument("--hz", type=float, default=1.0, help="打印/写 log 频率，默认 1")
    parser.add_argument(
        "--log",
        default="",
        help="log 路径；默认写本目录 watch_arm_joints_YYYYmmdd_HHMMSS.log；传 - 则不写文件",
    )
    args = parser.parse_args()

    log_fp: TextIO | None = None
    if args.log != "-":
        log_path = Path(args.log) if args.log else Path(__file__).resolve().parent / (
            f"watch_arm_joints_{time.strftime('%Y%m%d_%H%M%S')}.log"
        )
        log_path.parent.mkdir(parents=True, exist_ok=True)
        log_fp = log_path.open("w", encoding="utf-8")
        log_fp.write(f"# watch_arm_joints start {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        log_fp.flush()
        print(f"log -> {log_path}", flush=True)

    rclpy.init()
    node = WatchArmJoints(print_hz=args.hz, log_fp=log_fp)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if log_fp is not None:
            log_fp.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
