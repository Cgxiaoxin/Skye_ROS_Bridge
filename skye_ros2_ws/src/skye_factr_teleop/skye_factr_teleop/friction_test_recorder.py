# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Read-only ROS recorder for a single stage-7.2 static-friction trial."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from .config import load_config
from .control_core import FrictionTestSettings
from .friction_pair_trial import FrictionPairTrial


class FrictionTrial:
    def __init__(self, joint_id, current_limit_ma, enabled, enable_speed):
        self.joint_id = joint_id
        self.current_limit_ma = current_limit_ma
        self.enabled = enabled
        self.enable_speed = enable_speed
        self.phase = "waiting"
        self.states = []
        self.issues = []
        self.initial_idle = False
        self.start_position = None
        self.end_position = None
        self.last_position = None
        self.max_abs_velocity = [0.0] * 7
        self.above_threshold_samples = 0
        self.command_samples = 0
        self.present_samples = 0
        self.target_present_min_ma = None
        self.target_present_max_ma = None
        self.other_present_peak_ma = 0.0
        self.positive_samples = 0
        self.negative_samples = 0
        self.target_peak_ma = 0.0
        self.other_peak_ma = 0.0
        self.final_zero_observed = False

    def state(self, value, elapsed):
        self.states.append({"time_s": elapsed, "state": value})
        if self.phase == "waiting":
            if value == "IDLE":
                self.initial_idle = True
            elif value == "FRICTION_TEST":
                if not self.initial_idle:
                    self.issues.append("FRICTION_TEST observed without initial IDLE")
                self.phase = "testing"
                self.start_position = self.last_position
            else:
                self.issues.append(f"unexpected state before test: {value}")
        elif self.phase == "testing":
            if value == "IDLE":
                self.phase = "finished"
            elif value != "FRICTION_TEST":
                self.issues.append(f"unexpected state during test: {value}")

    def current(self, values):
        if len(values) != 8 or not all(math.isfinite(v) for v in values):
            self.issues.append("invalid eight-ID commanded current")
            return
        if self.phase == "finished" and all(v == 0 for v in values):
            self.final_zero_observed = True
        if self.phase != "testing":
            return
        target = values[self.joint_id - 1]
        self.command_samples += 1
        self.target_peak_ma = max(self.target_peak_ma, abs(target))
        self.other_peak_ma = max(self.other_peak_ma, *(abs(v) for i, v in enumerate(values)
                                                       if i != self.joint_id - 1))
        self.positive_samples += target > 0
        self.negative_samples += target < 0

    def present_current(self, values):
        if len(values) != 8 or not all(math.isfinite(v) for v in values):
            self.issues.append("invalid eight-ID present current")
        elif self.phase == "testing":
            self.present_samples += 1
            target = values[self.joint_id - 1]
            self.target_present_min_ma = (target if self.target_present_min_ma is None
                                          else min(self.target_present_min_ma, target))
            self.target_present_max_ma = (target if self.target_present_max_ma is None
                                          else max(self.target_present_max_ma, target))
            self.other_present_peak_ma = max(
                self.other_present_peak_ma,
                *(abs(v) for i, v in enumerate(values) if i != self.joint_id - 1),
            )

    def position(self, values, velocities):
        if (len(values) != 7 or len(velocities) != 7
                or not all(math.isfinite(v) for v in (*values, *velocities))):
            self.issues.append("invalid seven-joint state")
            return
        if self.phase == "testing":
            if self.start_position is None:
                self.start_position = list(values)
            self.end_position = list(values)
            self.max_abs_velocity = [max(a, abs(v)) for a, v in
                                     zip(self.max_abs_velocity, velocities)]
            self.above_threshold_samples += abs(velocities[self.joint_id - 1]) >= self.enable_speed
        self.last_position = list(values)

    def summary(self, *, effect, threshold_observed, motion, end_method, note):
        issues = list(self.issues)
        if self.phase != "finished":
            issues.append("IDLE -> FRICTION_TEST -> IDLE not fully observed")
        if not self.command_samples or not self.present_samples or self.start_position is None:
            issues.append("command, present-current, or position samples missing")
        if not self.final_zero_observed:
            issues.append("final zero command not observed")
        violation = self.other_peak_ma > 0 or self.target_peak_ma > self.current_limit_ma + 1e-9
        if violation:
            issues.append("command current violated target-only cap")
        if self.enabled and (not self.positive_samples or not self.negative_samples):
            issues.append("both dither signs were not observed")
        if self.enabled and not self.above_threshold_samples:
            issues.append("selected joint did not reach the speed threshold in recorded states")
        if not self.enabled and self.target_peak_ma > 0:
            issues.append("disabled comparison run commanded nonzero current")
        if (violation or motion in ("sudden", "other")
                or (self.enabled and (effect == "worse" or threshold_observed == "no"))
                or (not self.enabled and self.target_peak_ma > 0)):
            assessment = "FAIL"
        elif issues:
            assessment = "INCOMPLETE"
        elif motion != "none" or (self.enabled and
                                  (effect != "less" or threshold_observed != "yes")):
            assessment = "INCONCLUSIVE"
        elif end_method not in ("stop", "timeout"):
            assessment = "INCONCLUSIVE"
        else:
            assessment = "PENDING_SHUTDOWN_CHECK"
        return {
            "state_events": self.states,
            "start_position_rad": self.start_position,
            "end_position_rad": self.end_position,
            "max_abs_velocity_rad_s": self.max_abs_velocity,
            "above_threshold_state_samples": self.above_threshold_samples,
            "command_samples": self.command_samples,
            "present_current_samples": self.present_samples,
            "target_present_current_min_ma": self.target_present_min_ma,
            "target_present_current_max_ma": self.target_present_max_ma,
            "other_ids_peak_abs_present_current_ma": self.other_present_peak_ma,
            "positive_command_samples": self.positive_samples,
            "negative_command_samples": self.negative_samples,
            "target_peak_command_ma": self.target_peak_ma,
            "other_ids_peak_command_ma": self.other_peak_ma,
            "final_zero_command_observed": self.final_zero_observed,
            "operator_effect_vs_disabled": effect,
            "operator_threshold_observed": threshold_observed,
            "operator_motion": motion,
            "operator_end_method": end_method,
            "operator_note": note,
            "issues": issues,
            "assessment": assessment,
        }


def finalize_friction_record(record, readback, *, process_exited_normally,
                             readback_path, baseline=None, baseline_path=None):
    devices = readback.get("devices", [])
    all_disabled = ([device.get("id") for device in devices] == list(range(1, 9))
                    and all(device.get("torque_enabled") is False for device in devices))
    readback_valid = (readback.get("automated_result") == "PASS" and all_disabled
                      and record.get("side") == readback.get("side"))
    baseline_valid = (record.get("baseline_within_pose") is True
                      if record.get("paired") else not record.get("enabled"))
    if not record.get("paired") and record.get("enabled") and baseline is not None:
        fields = ("side", "joint_id", "config_path", "usb_by_id", "servo_model",
                  "urdf", "joint_offsets", "joint_signs", "configured_current_limit_ma",
                  "gravity_mode", "gravity_gain", "friction_gain", "enable_speed",
                  "gain_scale", "current_limit_ma", "ramp_seconds",
                  "max_duration_seconds")
        a, b = record.get("start_position_rad"), baseline.get("start_position_rad")
        baseline_valid = (baseline.get("assessment") == "PASS"
                          and baseline.get("enabled") is False
                          and all(record.get(field) == baseline.get(field) for field in fields)
                          and isinstance(a, list) and isinstance(b, list)
                          and len(a) == len(b) == 7
                          and max(abs(x - y) for x, y in zip(a, b)) <= 0.1)
    result = dict(record)
    result["process_exit"] = "normal" if process_exited_normally else "abnormal"
    result["torque_disable_readback_path"] = str(readback_path)
    result["torque_disable_ids_disabled"] = all_disabled
    result["baseline_path"] = str(baseline_path) if baseline_path else None
    result["baseline_valid"] = baseline_valid
    if not all_disabled and len(devices) == 8:
        result["assessment"] = "FAIL"
    elif result.get("assessment") == "PENDING_SHUTDOWN_CHECK":
        result["assessment"] = ("PASS" if process_exited_normally and readback_valid
                                and baseline_valid else "INCOMPLETE")
    return result


def _choice(prompt, allowed):
    while True:
        answer = input(prompt).strip().lower()
        if answer in allowed:
            return answer
        print(f"请输入：{' / '.join(allowed)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint-id", type=int, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--gain-scale", type=float, required=True)
    parser.add_argument("--current-limit-ma", type=float, required=True)
    parser.add_argument("--ramp-seconds", type=float, required=True)
    parser.add_argument("--max-duration-seconds", type=float, required=True)
    parser.add_argument("--enabled", choices=("true", "false"), required=True)
    parser.add_argument("--paired", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    settings = FrictionTestSettings(args.joint_id, args.gain_scale, args.current_limit_ma,
                                    args.ramp_seconds, args.max_duration_seconds,
                                    enabled=args.enabled == "true")
    if args.paired and settings.enabled:
        raise ValueError("paired run must start with --enabled false")
    config = load_config(args.config)
    if args.side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
        raise ValueError("selected side does not match leader URDF")
    if (config.dynamixel.joint_offsets is None or config.dynamixel.current_limit_ma is None
            or settings.current_limit_ma > min(config.dynamixel.current_limit_ma)):
        raise ValueError("test requires calibrated offsets and a configured current cap")
    index = settings.joint_id - 1
    gravity_gain = config.controller.joint_gravity_comp.gain[index]
    friction_gain = config.controller.static_friction_comp.gain[index]
    enable_speed = config.controller.static_friction_comp.enable_speed[index]
    if min(gravity_gain, friction_gain, enable_speed) <= 0:
        raise ValueError("selected joint gains and speed threshold must be positive")
    if settings.enabled and not config.controller.static_friction_comp.enable:
        raise ValueError("static friction must be enabled in the configuration")

    from rclpy import init, shutdown, spin_once
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String

    state_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                           reliability=ReliabilityPolicy.RELIABLE,
                           durability=DurabilityPolicy.TRANSIENT_LOCAL)
    data_qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                          reliability=ReliabilityPolicy.RELIABLE,
                          durability=DurabilityPolicy.VOLATILE)
    trial = (FrictionPairTrial(settings.joint_id, settings.current_limit_ma, enable_speed)
             if args.paired else FrictionTrial(settings.joint_id, settings.current_limit_ma,
                                                settings.enabled, enable_speed))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    trace_path = args.output.with_suffix(".trace.jsonl")
    if args.output.exists() or trace_path.exists():
        raise FileExistsError("output JSON or trace already exists")
    started_utc = datetime.now(timezone.utc).isoformat()
    started_monotonic = time.monotonic()
    init()
    try:
        with trace_path.open("x", encoding="utf-8", buffering=1) as trace:
            class RecorderNode(Node):
                def __init__(self):
                    super().__init__("phase7_2_friction_recorder")
                    self.create_subscription(String, "/teleop/state", self.on_state, state_qos)
                    if args.paired:
                        self.create_subscription(String,
                            f"/leader_arm/{args.side}/friction_test_phase",
                            self.on_phase, state_qos)
                    self.create_subscription(Float64MultiArray,
                        f"/leader_arm/{args.side}/friction_test_current_ma", self.on_current, data_qos)
                    self.create_subscription(Float64MultiArray,
                        f"/leader_arm/{args.side}/friction_test_present_current_ma",
                        self.on_present_current, data_qos)
                    self.create_subscription(JointState, "/leader_arm/current_state",
                                             self.on_position, data_qos)

                def log(self, topic, data):
                    elapsed = time.monotonic() - started_monotonic
                    trace.write(json.dumps({"time_s": elapsed, "topic": topic, "data": data}) + "\n")
                    return elapsed

                def on_state(self, message):
                    trial.state(message.data, self.log("state", message.data))
                    print(f"状态: {message.data}", flush=True)

                def on_phase(self, message):
                    trial.phase_event(message.data, self.log("friction_phase", message.data))
                    print(f"输出阶段: {message.data}", flush=True)

                def on_current(self, message):
                    values = list(message.data)
                    elapsed = self.log("command_current_ma", values)
                    if args.paired:
                        trial.current(values, elapsed)
                    else:
                        trial.current(values)

                def on_present_current(self, message):
                    values = list(message.data)
                    elapsed = self.log("present_current_ma", values)
                    if args.paired:
                        trial.present_current(values, elapsed)
                    else:
                        trial.present_current(values)

                def on_position(self, message):
                    values, velocities = list(message.position), list(message.velocity)
                    self.log("joint_state", {"name": list(message.name),
                                             "position": values, "velocity": velocities})
                    trial.position(values, velocities)

            node = RecorderNode()
            try:
                print("记录已启动；在另一终端启动目标侧测试。", flush=True)
                while True:
                    spin_once(node, timeout_sec=0.1)
                    if (trial.phase == "finished" and trial.states
                            and time.monotonic() - started_monotonic
                            - trial.states[-1]["time_s"] >= 0.5):
                        break
            except KeyboardInterrupt:
                print("记录中断；保存已收到的数据。", flush=True)
            finally:
                node.destroy_node()
    finally:
        shutdown()

    effect = threshold = motion = end_method = None
    note = ""
    if trial.phase != "waiting":
        if settings.enabled or args.paired:
            effect = _choice("对比关闭轮的起动阻力 [less/same/unclear/worse]: ",
                             ("less", "same", "unclear", "worse"))
            threshold = _choice("速度达到阈值时命令归零 [yes/no/unclear]: ",
                                ("yes", "no", "unclear"))
        motion = _choice("异常运动 [none/sudden/other]: ", ("none", "sudden", "other"))
        end_method = _choice("结束方式 [stop/timeout/other]: ",
                             ("stop", "timeout", "other"))
        note = input("现场备注（可留空）: ").strip()
    result = {
        "date_utc": started_utc,
        "operator": args.operator,
        "side": args.side,
        "config_path": str(args.config.resolve()),
        "usb_by_id": config.dynamixel.dynamixel_port,
        "urdf": config.controller.joint_gravity_comp.dynamics.leader_urdf,
        "joint_offsets": list(config.dynamixel.joint_offsets),
        "joint_signs": list(config.dynamixel.joint_signs),
        "configured_current_limit_ma": list(config.dynamixel.current_limit_ma),
        "joint_id": settings.joint_id,
        "servo_model": config.dynamixel.servo_types[index],
        "gravity_mode": config.controller.joint_gravity_comp.mode,
        "gravity_gain": gravity_gain,
        "friction_gain": friction_gain,
        "enable_speed": enable_speed,
        "enabled": settings.enabled or args.paired,
        "paired": args.paired,
        "gain_scale": settings.gain_scale,
        "current_limit_ma": settings.current_limit_ma,
        "ramp_seconds": settings.ramp_seconds,
        "max_duration_seconds": settings.max_duration_seconds,
        "trace_path": str(trace_path.resolve()),
        "process_exit": "PENDING",
        "torque_disable_readback_path": None,
        **trial.summary(effect=effect, threshold_observed=threshold,
                        motion=motion, end_method=end_method, note=note),
    }
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, ensure_ascii=False, indent=2)
        output.write("\n")
    print(f"记录: {args.output}\n初步判定: {result['assessment']}")
