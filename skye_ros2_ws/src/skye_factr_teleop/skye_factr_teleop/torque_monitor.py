# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Read-only displays of torque components, measured currents, and command clipping."""

import json
import math
import sys
import time


TORQUE_TERMS = {
    "gravity": ("gravity_torque_nm", "重力补偿力矩"),
    "friction": ("friction_torque_nm", "摩擦力补偿力矩"),
    "limit": ("joint_limit_torque_nm", "限位屏障力矩"),
}


class TorqueDisplay:
    def __init__(self, side, term):
        self.side = side
        self.field, self.title = TORQUE_TERMS[term]
        self.packet = None
        self.values = None
        self.received_at = None

    def update(self, packet, now):
        if packet.get("side") != self.side:
            return
        values = packet.get(self.field, [])
        if len(values) != 8 or not all(math.isfinite(value) for value in values):
            return
        self.packet, self.values, self.received_at = packet, list(values), now

    def render(self, now):
        fresh = self.received_at is not None and now - self.received_at <= 1.0
        lines = [f"{self.side} | {self.title} | N·m", "逻辑 J1..J7 / 夹爪；电流裁剪前的力矩分项"]
        if self.packet is None:
            lines.append("等待诊断消息；请启动该侧驱动并设置 sync_diagnostics:=true")
        elif not fresh:
            lines.append("STALE：超过 1 s 无新诊断消息，当前数值不可用")
        else:
            lines.append(f"mode={self.packet['mode']}  sequence={self.packet['sequence']}  "
                         f"torque_enabled_cached={self.packet['torque_enabled_cached']}")
        lines.append("关节             力矩 (N·m)")
        for i in range(8):
            name = f"J{i + 1}" if i < 7 else "Gripper(ID8)"
            value = f"{self.values[i]:+.6f}" if fresh else "--"
            lines.append(f"{name:<14} {value:>12}")
        lines.append("Ctrl+C 退出显示；不发送停止命令")
        return "\n".join(lines)


class CurrentDisplay:
    def __init__(self, side):
        self.side = side
        self.packet = None
        self.received_at = None
        self.clipped_in_window = [False] * 8

    def update(self, packet, now):
        if packet.get("side") != self.side:
            return
        for field in ("present_current_ma", "current_before_clip_ma", "current_after_clip_ma"):
            values = packet.get(field, [])
            if len(values) != 8 or not all(math.isfinite(value) for value in values):
                return
        limits = packet.get("current_limit_ma")
        if limits is not None and (len(limits) != 8 or
                                   not all(math.isfinite(value) and value > 0 for value in limits)):
            return
        self.packet, self.received_at = packet, now
        for i, (before, after) in enumerate(zip(packet["current_before_clip_ma"],
                                               packet["current_after_clip_ma"])):
            self.clipped_in_window[i] |= abs(before - after) > 1e-9

    def render(self, now):
        fresh = self.received_at is not None and now - self.received_at <= 1.0
        lines = [f"{self.side} | 实测电流 / current_limit_ma / 命令限幅 | mA",
                 "物理舵机 ID 1..8；CLIP 表示命令被裁剪，实测占比使用电流绝对值"]
        if self.packet is None:
            lines.append("等待诊断消息；请启动该侧驱动并设置 sync_diagnostics:=true")
        elif not fresh:
            lines.append("STALE：超过 1 s 无新诊断消息，当前数值不可用")
        else:
            lines.append(f"mode={self.packet['mode']}  sequence={self.packet['sequence']}  "
                         f"torque_enabled_cached={self.packet['torque_enabled_cached']}")
            if self.packet.get("current_limit_ma") is None:
                lines.append("限值缺失：请重启更新后的驱动，以发布实际软件电流上限")
        lines.append("轴          实测mA   上限mA    |实测|占比       命令前 -> 后(mA)   裁剪:帧/间隔")
        for i in range(8):
            name = f"ID{i + 1}/J{i + 1}" if i < 7 else "ID8/Grip"
            if not fresh:
                lines.append(f"{name:<9}      --       --       --                 --             --/--")
                continue
            actual = self.packet["present_current_ma"][i]
            before = self.packet["current_before_clip_ma"][i]
            after = self.packet["current_after_clip_ma"][i]
            limits = self.packet.get("current_limit_ma")
            if limits is None:
                limit_text, usage = "--", "--"
            else:
                limit = limits[i]
                ratio = abs(actual) / limit
                filled = min(10, int(ratio * 10))
                limit_text = f"{limit:.2f}"
                usage = f"[{'#' * filled}{'-' * (10 - filled)}]{ratio:4.0%}"
            clipped = "CLIP" if abs(before - after) > 1e-9 else "--"
            recent = "CLIP" if self.clipped_in_window[i] else "--"
            lines.append(f"{name:<9} {actual:+8.2f} {limit_text:>8} {usage:>16} "
                         f"{before:+8.2f}->{after:+7.2f}   {clipped}/{recent}")
        lines.append("帧/间隔 = 当前帧 / 本次刷新间隔内收到的数据；实测占比不代表命令限幅状态")
        lines.append("Ctrl+C 退出显示；不发送停止命令")
        self.clipped_in_window = [False] * 8
        return "\n".join(lines)


def monitor_torque(args):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    display = CurrentDisplay(args.side) if args.term == "current" else TorqueDisplay(args.side, args.term)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE)
    rclpy.init()
    node = Node(f"{args.side}_{args.term}_torque_monitor")

    def receive(message):
        display.update(json.loads(message.data), time.monotonic())

    def draw():
        prefix = "\033[H\033[2J" if sys.stdout.isatty() else ""
        print(prefix + display.render(time.monotonic()), flush=True)

    node.create_subscription(String, f"/leader_arm/{args.side}/sync_diagnostics", receive, qos)
    node.create_timer(1.0 / args.rate, draw)
    try:
        draw()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
