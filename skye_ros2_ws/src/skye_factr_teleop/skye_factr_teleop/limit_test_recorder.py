# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Read-only recorder for one stage-7.2 limit barrier trial."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from .config import load_config
from .control_core import LimitTestSettings


class LimitTrial:
    def __init__(self, joint_id, boundary, current_limit_ma):
        self.joint_id = joint_id
        self.boundary = boundary
        self.current_limit_ma = current_limit_ma
        self.phase = "waiting"
        self.states = []
        self.issues = []
        self.positions = []
        self.velocities = []
        self.command_samples = 0
        self.present_samples = 0
        self.target_peak_ma = 0.0
        self.other_peak_ma = 0.0
        self.target_present_min_ma = None
        self.target_present_max_ma = None
        self.final_zero_observed = False
        self.wrong_sign_samples = 0
        self.correct_sign_samples = 0

    def state(self, value, elapsed):
        self.states.append({"time_s": elapsed, "state": value})
        if self.phase == "waiting":
            if value == "IDLE":
                return
            if value != "LIMIT_TEST" or not any(s["state"] == "IDLE" for s in self.states):
                self.issues.append("test started without observed IDLE")
            self.phase = "testing"
        elif self.phase == "testing":
            if value == "IDLE":
                self.phase = "finished"
            elif value != "LIMIT_TEST":
                self.issues.append(f"unexpected test state: {value}")

    def position(self, position, velocity):
        if not math.isfinite(position) or not math.isfinite(velocity):
            self.issues.append("non-finite position or velocity")
        elif self.phase == "testing":
            self.positions.append(position)
            self.velocities.append(velocity)

    def current(self, values, physical_sign):
        if len(values) != 8 or not all(math.isfinite(x) for x in values):
            self.issues.append("invalid eight-ID command current")
            return
        if self.phase == "finished" and all(value == 0 for value in values):
            self.final_zero_observed = True
        if self.phase != "testing":
            return
        target = values[self.joint_id - 1]
        self.command_samples += 1
        self.target_peak_ma = max(self.target_peak_ma, abs(target))
        self.other_peak_ma = max(self.other_peak_ma, *(abs(v) for i, v in enumerate(values)
                                                      if i != self.joint_id - 1))
        if target != 0:
            inward = 1 if self.boundary == "lower" else -1
            if target * physical_sign * inward > 0:
                self.correct_sign_samples += 1
            else:
                self.wrong_sign_samples += 1

    def present_current(self, values):
        if len(values) != 8 or not all(math.isfinite(x) for x in values):
            self.issues.append("invalid eight-ID present current")
            return
        if self.phase == "testing":
            target = values[self.joint_id - 1]
            self.present_samples += 1
            self.target_present_min_ma = (target if self.target_present_min_ma is None
                                          else min(self.target_present_min_ma, target))
            self.target_present_max_ma = (target if self.target_present_max_ma is None
                                          else max(self.target_present_max_ma, target))

    def summary(self, observation=None, end_method=None, note=""):
        issues = list(self.issues)
        if self.phase != "finished":
            issues.append("IDLE -> LIMIT_TEST -> IDLE not fully observed")
        if not self.positions or not self.command_samples or not self.present_samples:
            issues.append("position, command, or present current samples missing")
        if not self.final_zero_observed:
            issues.append("final zero command not observed")
        if not self.correct_sign_samples:
            issues.append("no inward nonzero command observed")
        if self.other_peak_ma > 0 or self.target_peak_ma > self.current_limit_ma + 1e-9:
            issues.append("command current violated target-only cap")
        if self.wrong_sign_samples:
            issues.append("outward nonzero command observed")
        if self.wrong_sign_samples or self.other_peak_ma > 0 or observation in ("outward", "overrun", "sudden"):
            assessment = "FAIL"
        elif issues:
            assessment = "INCOMPLETE"
        elif observation == "inward" and end_method in ("stop", "timeout"):
            assessment = "PENDING_SHUTDOWN_CHECK"
        else:
            assessment = "INCONCLUSIVE"
        return {
            "state_events": self.states,
            "position_min_rad": min(self.positions) if self.positions else None,
            "position_max_rad": max(self.positions) if self.positions else None,
            "max_abs_velocity_rad_s": max(map(abs, self.velocities)) if self.velocities else None,
            "command_samples": self.command_samples,
            "target_peak_command_ma": self.target_peak_ma,
            "other_ids_peak_command_ma": self.other_peak_ma,
            "present_current_samples": self.present_samples,
            "target_present_current_min_ma": self.target_present_min_ma,
            "target_present_current_max_ma": self.target_present_max_ma,
            "correct_sign_samples": self.correct_sign_samples,
            "wrong_sign_samples": self.wrong_sign_samples,
            "final_zero_command_observed": self.final_zero_observed,
            "operator_observation": observation,
            "operator_end_method": end_method,
            "operator_note": note,
            "issues": issues,
            "assessment": assessment,
            "assessment_note": "PASS requires normal process exit and separate Torque Disable readback",
        }


def finalize_limit_record(record, readback, *, process_exited_normally, readback_path):
    """Combine recorded trial evidence with an independent read-only bus check."""
    devices = readback.get("devices", [])
    ids_match = [device.get("id") for device in devices] == list(range(1, 9))
    all_disabled = ids_match and all(device.get("torque_enabled") is False
                                     for device in devices)
    side_match = record.get("side") == readback.get("side")
    readback_valid = (readback.get("automated_result") == "PASS"
                      and all_disabled and side_match)
    result = dict(record)
    result["process_exit"] = "normal" if process_exited_normally else "abnormal"
    result["torque_disable_readback_path"] = str(readback_path)
    result["torque_disable_ids_disabled"] = all_disabled
    result["torque_disable_side_matches"] = side_match
    if not all_disabled and ids_match:
        result["assessment"] = "FAIL"
    elif result.get("assessment") == "PENDING_SHUTDOWN_CHECK":
        result["assessment"] = ("PASS" if process_exited_normally and readback_valid
                                else "INCOMPLETE")
    return result


def _choice(prompt, choices):
    while True:
        value = input(prompt).strip().lower()
        if value in choices:
            return value
        print(f"请输入：{' / '.join(choices)}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint-id", type=int, required=True)
    parser.add_argument("--boundary", choices=("lower", "upper"), required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--gain-scale", type=float, required=True)
    parser.add_argument("--current-limit-ma", type=float, required=True)
    parser.add_argument("--ramp-seconds", type=float, required=True)
    parser.add_argument("--max-duration-seconds", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    settings = LimitTestSettings(args.joint_id, args.gain_scale, args.current_limit_ma,
                                 args.ramp_seconds, args.max_duration_seconds)
    config = load_config(args.config)
    if args.side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
        raise ValueError("selected side does not match leader URDF")
    if (config.dynamixel.joint_offsets is None or config.dynamixel.current_limit_ma is None
            or settings.current_limit_ma > min(config.dynamixel.current_limit_ma)
            or not config.controller.limit_barrier_control.enable):
        raise ValueError("limit test requires calibration, enabled barrier, and valid current cap")
    barrier = config.controller.limit_barrier_control
    kp = barrier.gripper.kp if settings.joint_id == 8 else barrier.joint.kp
    if kp <= 0:
        raise ValueError("limit test requires positive position gain")
    if settings.joint_id == 8:
        lower, upper = barrier.gripper.limit_min, barrier.gripper.limit_max
    else:
        index = settings.joint_id - 1
        lower = barrier.joint.limit_min[index] + barrier.joint.safety_margin
        upper = barrier.joint.limit_max[index] - barrier.joint.safety_margin
    args.output.parent.mkdir(parents=True, exist_ok=True)
    trace_path = args.output.with_suffix(".trace.jsonl")
    if args.output.exists() or trace_path.exists():
        raise FileExistsError("output or trace exists; choose a new run name")

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
    trial = LimitTrial(settings.joint_id, args.boundary, settings.current_limit_ma)
    started = datetime.now(timezone.utc).isoformat()
    start_clock = time.monotonic()
    init()
    try:
        with trace_path.open("x", encoding="utf-8", buffering=1) as trace:
            class Recorder(Node):
                def __init__(self):
                    super().__init__("phase7_2_limit_recorder")
                    self.create_subscription(String, "/teleop/state", self.on_state, state_qos)
                    prefix = f"/leader_arm/{args.side}/limit_test"
                    self.create_subscription(Float64MultiArray, prefix + "_current_ma", self.on_current, data_qos)
                    self.create_subscription(Float64MultiArray, prefix + "_present_current_ma", self.on_present, data_qos)
                    self.create_subscription(Float64MultiArray, prefix + "_gripper_state", self.on_gripper, data_qos)
                    self.create_subscription(JointState, "/leader_arm/current_state", self.on_joint, data_qos)

                def log(self, topic, value):
                    elapsed = time.monotonic() - start_clock
                    trace.write(json.dumps({"time_s": elapsed, "topic": topic, "data": value}) + "\n")
                    return elapsed

                def on_state(self, msg):
                    trial.state(msg.data, self.log("state", msg.data))
                    print(f"状态: {msg.data}", flush=True)

                def on_current(self, msg):
                    values = list(msg.data)
                    self.log("command_current_ma", values)
                    trial.current(values, config.dynamixel.joint_signs[settings.joint_id - 1])

                def on_present(self, msg):
                    values = list(msg.data)
                    self.log("present_current_ma", values)
                    trial.present_current(values)

                def on_joint(self, msg):
                    self.log("joint_state", {"position": list(msg.position), "velocity": list(msg.velocity)})
                    if settings.joint_id < 8 and len(msg.position) == len(msg.velocity) == 7:
                        trial.position(msg.position[settings.joint_id - 1], msg.velocity[settings.joint_id - 1])

                def on_gripper(self, msg):
                    values = list(msg.data)
                    self.log("gripper_state", values)
                    if settings.joint_id == 8 and len(values) == 2:
                        trial.position(*values)

            node = Recorder()
            try:
                print("记录已启动；在另一终端启动一次限位测试。", flush=True)
                while True:
                    spin_once(node, timeout_sec=0.1)
                    if trial.phase == "finished":
                        # Allow the following cycle's zero-current topic to arrive.
                        for _ in range(10):
                            spin_once(node, timeout_sec=0.1)
                        break
            except KeyboardInterrupt:
                print("记录中断，保存已有数据。", flush=True)
            finally:
                node.destroy_node()
    finally:
        shutdown()
    observation = end_method = None
    note = ""
    if trial.phase != "waiting":
        observation = _choice("现场施力 [inward/outward/unclear/overrun/sudden]: ",
                              ("inward", "outward", "unclear", "overrun", "sudden"))
        end_method = _choice("结束方式 [stop/timeout/other]: ", ("stop", "timeout", "other"))
        note = input("补充观察（可留空）: ").strip()
    result = {
        "date_utc": started, "operator": args.operator, "side": args.side,
        "joint_id": settings.joint_id, "servo_model": config.dynamixel.servo_types[settings.joint_id - 1],
        "boundary": args.boundary, "effective_lower_rad": lower, "effective_upper_rad": upper,
        "config_path": str(args.config.resolve()), "usb_by_id": config.dynamixel.dynamixel_port,
        "gain_scale": settings.gain_scale, "current_limit_ma": settings.current_limit_ma,
        "ramp_seconds": settings.ramp_seconds, "max_duration_seconds": settings.max_duration_seconds,
        "trace_path": str(trace_path.resolve()), "process_exit": "PENDING",
        "torque_disable_readback_path": None,
        **trial.summary(observation, end_method, note),
    }
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, ensure_ascii=False, indent=2)
        output.write("\n")
    print(f"记录: {args.output}\n初步判定: {result['assessment']}")
    print("节点退出后须另做 Torque Disable 只读回读，方可完成判定。")


if __name__ == "__main__":
    main()
