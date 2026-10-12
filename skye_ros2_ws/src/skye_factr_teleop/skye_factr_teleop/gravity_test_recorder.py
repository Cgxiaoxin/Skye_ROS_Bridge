# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Read-only ROS topic recorder for one phase-7.2 gravity-test run."""

import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time

from .config import load_config
from .control_core import GravityTestSettings


class GravityTrial:
    """Summarize observed topic messages without commanding hardware."""

    def __init__(self, joint_id, current_limit_ma):
        self.joint_id = joint_id
        self.current_limit_ma = current_limit_ma
        self.phase = "waiting"
        self.states = []
        self.issues = []
        self.saw_initial_idle = False
        self.started_at_s = None
        self.ended_at_s = None
        self.pre_position = None
        self.start_position = None
        self.end_position = None
        self.post_position = None
        self.current_samples = 0
        self.target_peak_ma = 0.0
        self.other_peak_ma = 0.0
        self.present_current_samples = 0
        self.target_present_current_min_ma = None
        self.target_present_current_max_ma = None
        self.other_peak_present_current_ma = 0.0
        self.max_abs_velocity_rad_s = [0.0] * 7
        self.max_position_step_rad = [0.0] * 7
        self._last_test_position = None
        self.final_zero_observed = False

    def state(self, value, elapsed_s):
        self.states.append({"time_s": elapsed_s, "state": value})
        if self.phase == "waiting":
            if value == "IDLE":
                self.saw_initial_idle = True
            elif value == "GRAVITY_TEST":
                if not self.saw_initial_idle:
                    self.issues.append("GRAVITY_TEST observed without initial IDLE")
                self.phase = "testing"
                self.started_at_s = elapsed_s
                self.start_position = self.pre_position
                self._last_test_position = self.pre_position
            else:
                self.issues.append(f"unexpected state before test: {value}")
        elif self.phase == "testing":
            if value == "IDLE":
                self.phase = "finished"
                self.ended_at_s = elapsed_s
            elif value != "GRAVITY_TEST":
                self.issues.append(f"unexpected state during test: {value}")

    def current(self, values):
        if len(values) != 8 or not all(math.isfinite(x) for x in values):
            self.issues.append("invalid eight-ID current message")
            return
        if self.phase == "testing":
            self.current_samples += 1
            self.target_peak_ma = max(self.target_peak_ma, abs(values[self.joint_id - 1]))
            self.other_peak_ma = max(
                self.other_peak_ma,
                *(abs(value) for index, value in enumerate(values)
                  if index != self.joint_id - 1),
            )
        elif self.phase == "finished" and all(value == 0.0 for value in values):
            self.final_zero_observed = True

    def present_current(self, values):
        if len(values) != 8 or not all(math.isfinite(x) for x in values):
            self.issues.append("invalid eight-ID present current message")
            return
        if self.phase != "testing":
            return
        self.present_current_samples += 1
        target = values[self.joint_id - 1]
        self.target_present_current_min_ma = (
            target if self.target_present_current_min_ma is None
            else min(self.target_present_current_min_ma, target)
        )
        self.target_present_current_max_ma = (
            target if self.target_present_current_max_ma is None
            else max(self.target_present_current_max_ma, target)
        )
        self.other_peak_present_current_ma = max(
            self.other_peak_present_current_ma,
            *(abs(value) for index, value in enumerate(values)
              if index != self.joint_id - 1),
        )

    def position(self, values, velocities):
        if (len(values) != 7 or len(velocities) != 7
                or not all(math.isfinite(x) for x in (*values, *velocities))):
            self.issues.append("invalid seven-joint state message")
            return
        values = list(values)
        if self.phase == "waiting":
            self.pre_position = values
        elif self.phase == "testing":
            if self.start_position is None:
                self.start_position = values
            if self._last_test_position is not None:
                self.max_position_step_rad = [
                    max(old, abs(value - previous))
                    for old, value, previous in zip(
                        self.max_position_step_rad, values, self._last_test_position
                    )
                ]
            self.max_abs_velocity_rad_s = [
                max(old, abs(velocity))
                for old, velocity in zip(self.max_abs_velocity_rad_s, velocities)
            ]
            self._last_test_position = values
            self.end_position = values
        else:
            self.post_position = values

    def summary(self, force_effect=None, motion=None, end_method=None, note=""):
        issues = list(self.issues)
        if self.phase != "finished":
            issues.append("IDLE -> GRAVITY_TEST -> IDLE not fully observed")
        if self.current_samples == 0:
            issues.append("no gravity-test current samples")
        if self.present_current_samples == 0:
            issues.append("no gravity-test present current samples")
        if self.start_position is None or self.end_position is None:
            issues.append("start or end joint position missing")
        if self.other_peak_ma > 0.0:
            issues.append("non-target ID had nonzero commanded current")
        if self.target_peak_ma > self.current_limit_ma + 1e-9:
            issues.append("target commanded current exceeded run limit")
        if not self.final_zero_observed:
            issues.append("zero commanded current after IDLE not observed")

        command_violation = (self.other_peak_ma > 0.0
                             or self.target_peak_ma > self.current_limit_ma + 1e-9)
        if (motion in ("sudden", "reverse", "other")
                or force_effect == "more" or command_violation):
            assessment = "FAIL"
        elif issues:
            assessment = "INCOMPLETE"
        elif force_effect == "less" and motion == "none" and self.target_peak_ma > 0.0:
            assessment = "PENDING_SHUTDOWN_CHECK"
        else:
            assessment = "INCONCLUSIVE"
        return {
            "state_events": self.states,
            "start_position_rad": self.start_position,
            "end_position_rad": self.end_position,
            "post_position_rad": self.post_position,
            "test_duration_s": (self.ended_at_s - self.started_at_s
                                if self.ended_at_s is not None else None),
            "current_samples": self.current_samples,
            "target_peak_command_ma": self.target_peak_ma,
            "other_ids_peak_command_ma": self.other_peak_ma,
            "present_current_samples": self.present_current_samples,
            "target_present_current_min_ma": self.target_present_current_min_ma,
            "target_present_current_max_ma": self.target_present_current_max_ma,
            "other_ids_peak_abs_present_current_ma": self.other_peak_present_current_ma,
            "final_zero_command_observed": self.final_zero_observed,
            "max_abs_velocity_rad_s": self.max_abs_velocity_rad_s,
            "max_position_step_rad": self.max_position_step_rad,
            "operator_force_effect": force_effect,
            "operator_motion_observation": motion,
            "operator_end_method": end_method,
            "operator_note": note,
            "issues": issues,
            "assessment": assessment,
            "assessment_note": "PASS requires normal process exit and separate Torque Disable readback",
        }


def _choice(prompt, choices):
    while True:
        answer = input(prompt).strip().lower()
        if answer in choices:
            return answer
        print(f"请输入：{' / '.join(choices)}")


def _arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), required=True)
    parser.add_argument("--joint-id", type=int, required=True)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--operator", required=True)
    parser.add_argument("--gain-scale", type=float, required=True)
    parser.add_argument("--current-limit-ma", type=float, required=True)
    parser.add_argument("--ramp-seconds", type=float, required=True)
    parser.add_argument("--max-duration-seconds", type=float, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main():
    args = _arguments()
    settings = GravityTestSettings(
        args.joint_id, args.gain_scale, args.current_limit_ma,
        args.ramp_seconds, args.max_duration_seconds,
    )
    config = load_config(args.config)
    urdf = config.controller.joint_gravity_comp.dynamics.leader_urdf
    if args.side not in Path(urdf).stem:
        raise ValueError("selected side does not match leader URDF")
    if (config.dynamixel.joint_offsets is None
            or config.dynamixel.current_limit_ma is None
            or settings.current_limit_ma > min(config.dynamixel.current_limit_ma)):
        raise ValueError("test requires offsets and a current cap within configured limits")
    gain = config.controller.joint_gravity_comp.gain[settings.joint_id - 1]
    if not config.controller.joint_gravity_comp.enable or gain <= 0:
        raise ValueError("target gravity gain must be enabled and positive")
    if config.controller.joint_gravity_comp.mode != "position_only":
        raise ValueError("phase 7.2 recorder expects position_only")

    from rclpy import init, shutdown, spin_once
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String

    state_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )
    data_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST, depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    trial = GravityTrial(settings.joint_id, settings.current_limit_ma)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    trace_path = args.output.with_suffix(".trace.jsonl")
    if args.output.exists() or trace_path.exists():
        raise FileExistsError("output JSON or trace already exists; choose a new run name")
    started_utc = datetime.now(timezone.utc).isoformat()
    started_monotonic = time.monotonic()
    init()
    try:
        with trace_path.open("x", encoding="utf-8", buffering=1) as trace:
            class RecorderNode(Node):
                def __init__(self):
                    super().__init__("phase7_2_gravity_recorder")
                    self.create_subscription(String, "/teleop/state", self.on_state, state_qos)
                    self.create_subscription(
                        Float64MultiArray,
                        f"/leader_arm/{args.side}/gravity_test_current_ma",
                        self.on_current, data_qos,
                    )
                    self.create_subscription(
                        Float64MultiArray,
                        f"/leader_arm/{args.side}/gravity_test_present_current_ma",
                        self.on_present_current, data_qos,
                    )
                    self.create_subscription(
                        JointState, "/leader_arm/current_state", self.on_position, data_qos,
                    )

                def log(self, topic, values):
                    elapsed = time.monotonic() - started_monotonic
                    trace.write(json.dumps({"time_s": elapsed, "topic": topic, "data": values}) + "\n")
                    return elapsed

                def on_state(self, message):
                    elapsed = self.log("state", message.data)
                    trial.state(message.data, elapsed)
                    print(f"状态: {message.data}", flush=True)

                def on_current(self, message):
                    values = list(message.data)
                    self.log("command_current_ma", values)
                    trial.current(values)

                def on_present_current(self, message):
                    values = list(message.data)
                    self.log("present_current_ma", values)
                    trial.present_current(values)

                def on_position(self, message):
                    values = list(message.position)
                    velocities = list(message.velocity)
                    self.log("joint_state", {"name": list(message.name),
                                             "position": values, "velocity": velocities})
                    trial.position(values, velocities)

            node = RecorderNode()
            try:
                print("记录已启动；按清单在另一终端启动节点并执行一次测试。", flush=True)
                while True:
                    spin_once(node, timeout_sec=0.1)
                    if (trial.phase == "finished" and trial.ended_at_s is not None
                            and time.monotonic() - started_monotonic - trial.ended_at_s >= 0.5):
                        break
            except KeyboardInterrupt:
                print("记录中断；仍将保存已收到的数据。", flush=True)
            finally:
                node.destroy_node()
    finally:
        shutdown()

    force_effect = motion = end_method = None
    note = ""
    if trial.started_at_s is not None:
        print("确认小臂已停止并保持支撑后，填写现场观察。")
        force_effect = _choice("同姿态支撑力 [less=减轻/unclear=不清楚/more=加重]: ",
                               ("less", "unclear", "more"))
        motion = _choice("异常运动 [none=无/sudden=突动/reverse=反向牵引/other=其它关节]: ",
                         ("none", "sudden", "reverse", "other"))
        end_method = _choice("本轮结束 [stop/timeout/other]: ",
                             ("stop", "timeout", "other"))
        note = input("补充现象（可直接回车跳过）: ").strip()
    result = {
        "date_utc": started_utc,
        "operator": args.operator,
        "side": args.side,
        "config_path": str(args.config.resolve()),
        "usb_by_id": config.dynamixel.dynamixel_port,
        "urdf_package": config.controller.joint_gravity_comp.dynamics.leader_urdf_package,
        "urdf": config.controller.joint_gravity_comp.dynamics.leader_urdf,
        "gravity_mode": config.controller.joint_gravity_comp.mode,
        "joint_id": settings.joint_id,
        "servo_model": config.dynamixel.servo_types[settings.joint_id - 1],
        "configured_gravity_gain": gain,
        "gain_scale": settings.gain_scale,
        "current_limit_ma": settings.current_limit_ma,
        "ramp_seconds": settings.ramp_seconds,
        "max_duration_seconds": settings.max_duration_seconds,
        "trace_path": str(trace_path.resolve()),
        "process_exit": "PENDING",
        "torque_disable_readback_path": None,
        **trial.summary(force_effect, motion, end_method, note),
    }
    with args.output.open("x", encoding="utf-8") as output:
        json.dump(result, output, ensure_ascii=False, indent=2)
        output.write("\n")
    print(f"记录: {args.output}\n初步判定: {result['assessment']}")
    print("退出测试节点后按清单回读 Torque Disable；将进程退出结果和回读文件交给我完成判定。")


if __name__ == "__main__":
    main()
