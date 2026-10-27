# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Read-only joint teleoperation recording; never publishes or stops motion."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np
import yaml

from .config import load_config
from .sync_trial import RecordStatus, TrialSummary


def joint_packet(message, side):
    """Preserve the seven-axis ROS packet, without remapping or interpolation."""
    names = [f"{'l' if side == 'left' else 'r'}_j{i}" for i in range(1, 8)]
    if list(message.name) != names:
        raise ValueError("joint names/order do not match selected arm")
    packet = {"name": list(message.name)}
    for field in ("position", "velocity", "effort"):
        values = np.asarray(getattr(message, field), dtype=float)
        if (values.shape != (7,) and not (field != "position" and values.shape == (0,))) or not np.all(np.isfinite(values)):
            raise ValueError(f"{field} must contain seven finite values (velocity/effort may be empty)")
        packet[field] = values.tolist()
    stamp = message.header.stamp
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid JointState timestamp")
    packet["stamp_s"] = stamp.sec + stamp.nanosec * 1e-9
    if packet["stamp_s"] == 0:
        raise ValueError("JointState requires a nonzero timestamp")
    return packet


class TeleopSummary:
    """Summarize diagnostics and timestamped streams; no hardware PASS inference."""

    def __init__(self, post_stop_seconds):
        self.cycles = TrialSummary()
        self.post_stop_seconds = post_stop_seconds
        self.idle_received_at = None
        self.teleop_observed = False
        self.streams = {}
        self._previous = {}

    def diagnostic(self, packet, received_at):
        self.cycles.observe(packet)
        self.cycles.started |= packet["mode"] == "TELEOP"
        self.teleop_observed |= packet["mode"] == "TELEOP"
        stopped = (self.cycles.started and packet["mode"] == "IDLE"
                   and not packet["torque_enabled_cached"] and not packet["wrote_current"]
                   and bool(np.all(np.asarray(packet["current_after_clip_ma"]) == 0)))
        if stopped:
            if self.idle_received_at is None:
                self.idle_received_at = received_at
        else:
            self.idle_received_at = None

    def stream(self, name, packet, received_at):
        stats = self.streams.setdefault(name, {
            "count": 0, "first_received_at": received_at,
            "last_received_at": received_at, "max_receive_gap_s": 0.0,
            "max_abs_position_step_rad": [0.0] * 7,
            "nonincreasing_stamp_count": 0,
            "first": packet, "last": packet,
        })
        previous = self._previous.get(name)
        if previous is not None:
            stats["max_receive_gap_s"] = max(stats["max_receive_gap_s"], received_at - stats["last_received_at"])
            stats["nonincreasing_stamp_count"] += int(packet["stamp_s"] <= previous["stamp_s"])
            step = np.abs(np.asarray(packet["position"]) - previous["position"])
            stats["max_abs_position_step_rad"] = np.maximum(stats["max_abs_position_step_rad"], step).tolist()
        stats["count"] += 1
        stats["last_received_at"] = received_at
        stats["last"] = packet
        self._previous[name] = packet

    def post_stop_complete(self, now):
        return self.idle_received_at is not None and now - self.idle_received_at >= self.post_stop_seconds

    def result(self, now):
        result = self.cycles.result()
        complete = (self.teleop_observed and self.post_stop_complete(now)
                    and all(self.streams.get(name, {}).get("count", 0) > 0
                            for name in ("command", "applied", "feedback")))
        for stats in self.streams.values():
            elapsed = stats["last_received_at"] - stats["first_received_at"]
            stats["receive_hz"] = ((stats["count"] - 1) / elapsed if elapsed > 0 else None)
        result.update({
            "assessment": "RECORDED" if complete else "INCOMPLETE",
            "teleop_observed": self.teleop_observed,
            "post_stop_seconds": self.post_stop_seconds,
            "post_stop_window_complete": self.post_stop_complete(now),
            "streams": self.streams,
        })
        return result


def record(args, config):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    # Read the original driver YAML before starting; this is a file snapshot,
    # not a claim about effective ROS parameters after launch overlays/services.
    driver_document = yaml.safe_load(args.driver_config.read_text(encoding="utf-8"))
    output, trace_path = args.output, args.output.with_suffix(".trace.jsonl")
    output.parent.mkdir(parents=True, exist_ok=True)
    if output == trace_path or output.exists() or trace_path.exists():
        raise FileExistsError("choose unused output/trace paths; existing records are preserved")
    summary = TeleopSummary(args.post_stop_seconds)
    display = RecordStatus(config)
    invalid_messages = {}
    started_utc = datetime.now(timezone.utc).isoformat()
    rclpy.init()
    node = Node(f"{args.side}_teleop_trial_recorder")
    best_effort = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
    reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    ended_by = "duration"
    try:
        with trace_path.open("x", encoding="utf-8") as trace:
            def write(event, data, received_at):
                trace.write(json.dumps({"event": event, "received_monotonic_s": received_at,
                                        "data": data}, allow_nan=False) + "\n")

            def diagnostic(message):
                packet = json.loads(message.data)
                if packet["side"] != args.side:
                    raise ValueError("diagnostic side mismatch")
                received_at = time.monotonic()
                previous_mode = summary.cycles.states[-1] if summary.cycles.states else None
                summary.diagnostic(packet, received_at)
                display.observe(packet, received_at=received_at)
                write("diagnostic", packet, received_at)
                if packet["mode"] != previous_mode:
                    trace.flush()
                    print(f"{packet['mode']} | samples={packet['sequence']} | lost={summary.cycles.missing_samples}", flush=True)

            def receive(name, side):
                def callback(message):
                    received_at = time.monotonic()
                    try:
                        packet = joint_packet(message, side)
                    except ValueError as exc:
                        invalid_messages[name] = invalid_messages.get(name, 0) + 1
                        write("invalid_" + name, {"reason": str(exc), "message": repr(message)}, received_at)
                        return
                    summary.stream(name, packet, received_at)
                    if name == "command":
                        summary.cycles.follower_command_count += 1
                    write(name, packet, received_at)
                return callback

            node.create_subscription(String, f"/leader_arm/{args.side}/sync_diagnostics", diagnostic, best_effort)
            other = "right" if args.side == "left" else "left"
            for name, topic, side, qos in (
                ("command", f"/gento/{args.side}_joint_control", args.side, best_effort),
                ("applied", f"/gento/{args.side}_joint_action_applied", args.side, reliable),
                ("feedback", f"/gento/{args.side}_joint_states", args.side, best_effort),
                ("other_command", f"/gento/{other}_joint_control", other, best_effort),
                ("absolute_command", f"/gento/{args.side}_joint_control_abs", args.side, best_effort),
            ):
                node.create_subscription(JointState, topic, receive(name, side), qos)
            for mode in ("sync", "teleop", "stop"):
                node.create_subscription(
                    String, f"/mode/switch_{mode}",
                    lambda message, mode=mode: write(f"request_{mode}", {"data": message.data}, time.monotonic()), reliable)
            print(f"Recording {trace_path}; sync -> TELEOP -> stop, then record {args.post_stop_seconds:g}s. "
                  "Duration/Ctrl+C only saves; use independent stop/hold terminals.", flush=True)
            deadline = time.monotonic() + args.duration
            try:
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    now = time.monotonic()
                    if summary.post_stop_complete(now):
                        ended_by = "post_stop_window"
                        break
                    text = display.render(now=now, ros_now=node.get_clock().now().nanoseconds * 1e-9)
                    if text is not None:
                        print(text, flush=True)
            except (KeyboardInterrupt, ExternalShutdownException):
                ended_by = "interrupted"
    finally:
        result = {
            "side": args.side, "operator": args.operator, "started_utc": started_utc,
            "ended_by": ended_by, "config_path": str(args.config.resolve()), "config": asdict(config),
            "driver_config_path": str(args.driver_config.resolve()), "driver_config_file": driver_document,
            "trace_path": str(trace_path.resolve()), "invalid_messages": invalid_messages,
            **summary.result(time.monotonic()),
        }
        try:
            with output.open("x", encoding="utf-8") as stream:
                json.dump(result, stream, ensure_ascii=False, allow_nan=False, indent=2)
                stream.write("\n")
        finally:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        print(f"{result['assessment']}: {output}; assess motion/hold from the trace and operator observations.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--driver-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--post-stop-seconds", type=float, default=3.0)
    args = parser.parse_args(argv)
    try:
        for name in ("duration", "post_stop_seconds"):
            if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        config = load_config(args.config, require_runtime=True)
        if args.side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
            raise ValueError("side does not match leader configuration")
        record(args, config)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.exit(2, f"{exc}\n")


if __name__ == "__main__":
    main()
