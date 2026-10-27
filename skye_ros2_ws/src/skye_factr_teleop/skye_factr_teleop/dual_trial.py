# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""One read-only ROS node recording both arms; never publishes or stops motion."""

import argparse
from contextlib import ExitStack
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import shlex
import tempfile
import time

import numpy as np
import yaml

from .config import load_config
from .sync_trial import RecordStatus, TrialSummary
from .teleop_trial import TeleopSummary, joint_packet


SIDES = ("left", "right")


class DualRecorder:
    """Keep independent arm summaries and compatible per-arm trace files."""

    def __init__(self, args):
        self.args = args
        for name in ("duration", "post_stop_seconds"):
            if not np.isfinite(getattr(args, name)) or getattr(args, name) <= 0:
                raise ValueError(f"{name} must be finite and positive")
        self.configs = {}
        explicit = [getattr(args, f"{side}_output") is not None for side in SIDES]
        output_dir = getattr(args, "output_dir", None)
        if any(explicit) and not all(explicit):
            raise ValueError("provide both --left-output and --right-output, or omit both for automatic sessions")
        if any(explicit) and output_dir is not None:
            raise ValueError("choose --output-dir or explicit per-arm output paths")
        self.output_dir = Path(output_dir or "/tmp/skye_factr_trials").resolve()
        self.session_dir = None
        self.outputs = ({side: getattr(args, f"{side}_output").resolve() for side in SIDES}
                        if all(explicit) else {})
        self.trace_paths = {side: path.with_suffix(".trace.jsonl") for side, path in self.outputs.items()}
        paths = list(self.outputs.values()) + list(self.trace_paths.values())
        if paths and len(set(paths)) != 4:
            raise ValueError("both output and trace paths must be distinct")
        if any(path.exists() for path in paths):
            raise FileExistsError("choose unused output/trace paths; existing records are preserved")
        for side in SIDES:
            config = load_config(getattr(args, f"{side}_config"), require_runtime=True)
            if side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
                raise ValueError(f"{side} configuration does not match arm side")
            self.configs[side] = config
        self.driver_document = None
        if args.phase == "teleop":
            if args.driver_config is None:
                raise ValueError("teleop phase requires --driver-config")
            self.driver_document = yaml.safe_load(args.driver_config.read_text(encoding="utf-8"))
        self.summaries = {
            side: TeleopSummary(args.post_stop_seconds) if args.phase == "teleop" else TrialSummary()
            for side in SIDES
        }
        self.cycles = {side: summary.cycles if args.phase == "teleop" else summary
                       for side, summary in self.summaries.items()}
        self.displays = {side: RecordStatus(config) for side, config in self.configs.items()}
        self.invalid_messages = {side: {} for side in SIDES}
        self.started_utc = datetime.now(timezone.utc).isoformat()

    def __enter__(self):
        self.stack = ExitStack()
        self.traces = {}
        try:
            if not self.outputs:
                self.output_dir.mkdir(parents=True, exist_ok=True)
                prefix = f"dual_{self.args.phase}_{datetime.now():%Y%m%d_%H%M%S}_"
                self.session_dir = Path(tempfile.mkdtemp(prefix=prefix, dir=self.output_dir))
                self.outputs = {side: self.session_dir / f"{side}.json" for side in SIDES}
                self.trace_paths = {side: path.with_suffix(".trace.jsonl") for side, path in self.outputs.items()}
            for side in SIDES:
                self.trace_paths[side].parent.mkdir(parents=True, exist_ok=True)
                self.traces[side] = self.stack.enter_context(
                    self.trace_paths[side].open("x", encoding="utf-8"))
        except Exception:
            self.stack.close()
            raise
        return self

    def __exit__(self, *exc):
        self.stack.close()

    @staticmethod
    def emit(side, text):
        for line in text.splitlines():
            print(f"[{side}] {line}", flush=True)

    def write(self, side, event, data, received_at):
        if self.args.phase == "sync":
            packet = {"received_monotonic_s": received_at}
            packet["data" if event == "diagnostic" else "unexpected_follower_command"] = data
        else:
            packet = {"event": event, "received_monotonic_s": received_at, "data": data}
        self.traces[side].write(json.dumps(packet, allow_nan=False) + "\n")

    def diagnostic(self, side, packet, received_at):
        if packet["side"] != side:
            raise ValueError(f"{side} diagnostic side mismatch")
        cycles = self.cycles[side]
        previous = cycles.states[-1] if cycles.states else None
        if self.args.phase == "teleop":
            self.summaries[side].diagnostic(packet, received_at)
        else:
            cycles.observe(packet)
        self.displays[side].observe(packet, received_at=received_at)
        self.write(side, "diagnostic", packet, received_at)
        if packet["mode"] != previous:
            self.traces[side].flush()
            self.emit(side, f"{packet['mode']} | samples={packet['sequence']} | lost={cycles.missing_samples}")

    def joint(self, side, event, message, received_at):
        if self.args.phase == "sync":
            self.cycles[side].follower_command_count += 1
            self.write(side, "command", list(message.position), received_at)
            return
        recipients = [(side, event)]
        if event == "command":
            recipients.append(("right" if side == "left" else "left", "other_command"))
        try:
            packet = joint_packet(message, side)
        except ValueError as exc:
            for target, name in recipients:
                invalid = self.invalid_messages[target]
                invalid[name] = invalid.get(name, 0) + 1
                self.write(target, "invalid_" + name,
                           {"reason": str(exc), "message": repr(message)}, received_at)
            return
        for target, name in recipients:
            self.summaries[target].stream(name, packet, received_at)
            if name == "command":
                self.cycles[target].follower_command_count += 1
            self.write(target, name, packet, received_at)

    def request(self, mode, data, received_at):
        for side in SIDES:
            self.write(side, f"request_{mode}", {"data": data}, received_at)

    def ready_to_finish(self, now, ros_now):
        for side in SIDES:
            display = self.displays[side]
            packet = display.packet
            if packet is None or packet["mode"] != "IDLE":
                return False
            if not (0 <= now - display.received_at <= display.runtime.state_timeout
                    and -display.runtime.future_tolerance <= ros_now - packet["stamp_s"]
                    <= display.runtime.state_timeout):
                return False
            if (packet["torque_enabled_cached"] or packet["wrote_current"]
                    or any(value != 0 for value in packet["current_after_clip_ma"])):
                return False
            if self.args.phase == "teleop":
                if not self.summaries[side].post_stop_complete(now):
                    return False
            elif not self.cycles[side].stopped:
                return False
        return True

    def render(self, now, ros_now):
        for side, display in self.displays.items():
            text = display.render(now=now, ros_now=ros_now)
            if text is not None:
                self.emit(side, text)

    def save(self, ended_by, now):
        for side in SIDES:
            summary = self.summaries[side]
            result = {
                "side": side, "operator": self.args.operator, "started_utc": self.started_utc,
                "ended_by": ended_by, "phase": self.args.phase,
                "config_path": str(getattr(self.args, f"{side}_config").resolve()),
                "config": asdict(self.configs[side]), "trace_path": str(self.trace_paths[side]),
                **(summary.result(now) if self.args.phase == "teleop" else summary.result()),
            }
            if self.args.phase == "teleop":
                result.update({
                    "driver_config_path": str(self.args.driver_config.resolve()),
                    "driver_config_file": self.driver_document,
                    "invalid_messages": self.invalid_messages[side],
                })
            self.traces[side].flush()
            with self.outputs[side].open("x", encoding="utf-8") as stream:
                json.dump(result, stream, ensure_ascii=False, allow_nan=False, indent=2)
                stream.write("\n")
            self.emit(side, f"{result['assessment']}: {self.outputs[side]}; assess hardware acceptance separately.")
        print("先 stop/hold 并退出双臂小臂节点、释放串口后，可在本终端执行以下独立失能回读命令：", flush=True)
        for side in SIDES:
            output = self.outputs[side]
            command = ["ros2", "run", "skye_factr_teleop", "skye_factr_dynamixel_readonly_check", "--",
                       "--side", side, "--config", str(getattr(self.args, f"{side}_config").resolve()),
                       "--frames", "2", "--interval", "0", "--output",
                       str(output.with_name(output.stem + "_torque_disable.json"))]
            print(shlex.join(command), flush=True)


def record(args):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    with DualRecorder(args) as recorder:
        rclpy.init()
        node = Node("dual_arm_trial_recorder")
        best_effort = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        reliable = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
        ended_by = "duration"
        try:
            for side in SIDES:
                def diagnostic(message, side=side):
                    recorder.diagnostic(side, json.loads(message.data), time.monotonic())

                node.create_subscription(String, f"/leader_arm/{side}/sync_diagnostics", diagnostic, best_effort)
                topics = [("command", f"/gento/{side}_joint_control", best_effort)]
                if args.phase == "teleop":
                    topics += [("applied", f"/gento/{side}_joint_action_applied", reliable),
                               ("feedback", f"/gento/{side}_joint_states", best_effort),
                               ("absolute_command", f"/gento/{side}_joint_control_abs", best_effort)]
                for event, topic, qos in topics:
                    def joint(message, side=side, event=event):
                        recorder.joint(side, event, message, time.monotonic())

                    node.create_subscription(JointState, topic, joint, qos)
            if args.phase == "teleop":
                for mode in ("sync", "teleop", "stop"):
                    def request(message, mode=mode):
                        recorder.request(mode, message.data, time.monotonic())

                    node.create_subscription(String, f"/mode/switch_{mode}", request, reliable)
            if recorder.session_dir is not None:
                print(f"Session directory: {recorder.session_dir}", flush=True)
            for side in SIDES:
                recorder.emit(side, f"Recording {recorder.trace_paths[side]}")
            print(f"Dual {args.phase} recording; waiting for both arms to stop"
                  + (f", then {args.post_stop_seconds:g}s per arm." if args.phase == "teleop" else ".")
                  + " Duration/Ctrl+C only saves; use an independent stop/hold terminal.", flush=True)
            deadline = time.monotonic() + args.duration
            try:
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(node, timeout_sec=0.05)
                    now = time.monotonic()
                    ros_now = node.get_clock().now().nanoseconds * 1e-9
                    recorder.render(now, ros_now)
                    if recorder.ready_to_finish(now, ros_now):
                        ended_by = "post_stop_window" if args.phase == "teleop" else "both_stopped"
                        break
            except (KeyboardInterrupt, ExternalShutdownException):
                ended_by = "interrupted"
        finally:
            try:
                recorder.save(ended_by, time.monotonic())
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--phase", choices=("sync", "teleop"), required=True)
    for side in SIDES:
        parser.add_argument(f"--{side}-config", type=Path, required=True)
        parser.add_argument(f"--{side}-output", type=Path)
    parser.add_argument("--output-dir", type=Path,
                        help="parent for a new automatic session (default: /tmp/skye_factr_trials); omit per-arm outputs")
    parser.add_argument("--driver-config", type=Path)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--duration", type=float, default=180.0)
    parser.add_argument("--post-stop-seconds", type=float, default=3.0)
    args = parser.parse_args(argv)
    try:
        record(args)
    except (OSError, ValueError, yaml.YAMLError) as exc:
        parser.exit(2, f"{exc}\n")


if __name__ == "__main__":
    main()
