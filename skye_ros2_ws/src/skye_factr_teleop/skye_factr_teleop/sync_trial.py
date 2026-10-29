# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Cycle diagnostics, read-only torque monitoring/recording, and explicit joint_move trials."""

import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
from pathlib import Path
import time

import numpy as np

from .config import load_config


def vector(values, name):
    array = np.asarray(values, dtype=float)
    if array.shape != (8,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} requires eight finite values")
    return array


def cycle_packet(result, *, side, sequence, stamp_s, monotonic_s, cycle_dt_s,
                 torque_enabled_cached, present_current_ma, sync_status=None, current_limit_ma=None):
    """All arrays use logical J1..J7/gripper order, except physical ID currents."""
    packet = {
        "schema_version": 1, "side": side, "sequence": sequence,
        "stamp_s": stamp_s, "monotonic_s": monotonic_s, "cycle_dt_s": cycle_dt_s,
        "mode": result.mode.name,
        "sync_status": sync_status,
        "torque_enabled_cached": bool(torque_enabled_cached),
        "wrote_current": result.wrote_current,
        "position_rad": np.append(result.state.arm_position, result.state.gripper_position).tolist(),
        "velocity_rad_s": np.append(result.state.arm_velocity, result.state.gripper_velocity).tolist(),
        "target_position_rad": (None if result.target_position is None else result.target_position.tolist()),
        "target_velocity_rad_s": (None if result.target_velocity is None else result.target_velocity.tolist()),
        "current_before_clip_ma": result.unclipped_current_ma.tolist(),
        "current_after_clip_ma": result.physical_current_ma.tolist(),
        "present_current_ma": vector(present_current_ma, "present_current_ma").tolist(),
        "current_limit_ma": (None if current_limit_ma is None else
                             vector(current_limit_ma, "current_limit_ma").tolist()),
    }
    for name in ("joint_limit", "gravity", "friction", "nullspace", "tracking", "logical"):
        packet[f"{name}_torque_nm"] = getattr(result, f"{name}_torque").tolist()
    return packet


def preview_move(config, position, velocity, *, joint_id=None, delta=None, target=None, deltas=None):
    """Validate logical bounds and compute the full eight-axis trajectory offline."""
    from ruckig import InputParameter, Ruckig, Trajectory

    position, velocity = vector(position, "position"), vector(velocity, "velocity")
    if deltas is not None:
        if target is not None or joint_id is not None or delta is not None:
            raise ValueError("choose --deltas OR --target OR --joint-id with --delta")
        increments = np.asarray(deltas, dtype=float)
        if increments.shape != (7,) or not np.all(np.isfinite(increments)):
            raise ValueError("deltas requires seven finite relative positions")
        arm = position[:7] + increments
    elif target is None:
        if joint_id not in range(1, 8) or delta is None or not np.isfinite(delta):
            raise ValueError("relative move requires --joint-id 1..7 and finite --delta")
        arm = position[:7].copy()
        arm[joint_id - 1] += delta
    else:
        if joint_id is not None or delta is not None:
            raise ValueError("choose --target OR --joint-id with --delta")
        arm = np.asarray(target, dtype=float)
        if arm.shape != (7,) or not np.all(np.isfinite(arm)):
            raise ValueError("target requires seven finite positions")
    sync = config.controller.sync_hold_control
    barrier = config.controller.limit_barrier_control
    goal = np.append(arm, barrier.gripper.gripper_setpoint)
    lower = np.append(np.asarray(barrier.joint.limit_min) + barrier.joint.safety_margin,
                      barrier.gripper.limit_min)
    upper = np.append(np.asarray(barrier.joint.limit_max) - barrier.joint.safety_margin,
                      barrier.gripper.limit_max)
    if np.any(position < lower) or np.any(position > upper):
        raise ValueError("current pose is outside configured safe bounds; reposition before trial")
    if np.any(goal < lower) or np.any(goal > upper):
        raise ValueError("target is outside configured safe bounds")
    # Intended for stopped, supported starts; large measured velocity changes the path.
    if np.any(np.abs(velocity) > np.minimum(sync.hold.sync_velocity_tolerance,
                                           sync.ruckig.max_velocity)):
        raise ValueError("leader is moving; wait for a stationary start")
    inp = InputParameter(8)
    inp.current_position, inp.current_velocity = position.tolist(), velocity.tolist()
    inp.current_acceleration = [0.0] * 8
    inp.target_position = goal.tolist()
    inp.target_velocity = inp.target_acceleration = [0.0] * 8
    for name in ("max_velocity", "max_acceleration", "max_jerk"):
        setattr(inp, name, list(getattr(sync.ruckig, name)))
    generator = Ruckig(8, 1 / config.controller.frequency)
    generator.validate_input(inp, True, True)
    trajectory = Trajectory(8)
    result = generator.calculate(inp, trajectory)
    if int(result) < 0:
        raise ValueError(f"Ruckig preview failed: {result}")
    if trajectory.duration + 0.25 >= sync.ruckig.trajectory_timeout:
        raise ValueError(f"trajectory {trajectory.duration:.3f}s leaves insufficient margin "
                         f"before {sync.ruckig.trajectory_timeout:.3f}s timeout; shorten move")
    for i, bounds in enumerate(trajectory.position_extrema):
        if bounds.min < lower[i] - 1e-9 or bounds.max > upper[i] + 1e-9:
            raise ValueError(f"trajectory leaves configured safe bounds on axis {i + 1}")
    return {"start_position_rad": position.tolist(), "start_velocity_rad_s": velocity.tolist(),
            "target_position_rad": goal.tolist(), "trajectory_duration_s": trajectory.duration}


class TrialSummary:
    """Summarize received samples; never turn software observation into hardware PASS."""

    def __init__(self):
        self.states = []
        self.phases = {}
        self.transitions = []
        self.missing_samples = 0
        self.sequence_restarts = 0
        self.previous = None
        self.started = False
        self.stopped = False
        self.follower_command_count = 0

    def observe(self, packet):
        mode = packet["mode"]
        previous = self.previous
        if previous is not None:
            jump = packet["sequence"] - previous["sequence"]
            self.missing_samples += max(0, jump - 1)
            self.sequence_restarts += int(jump <= 0)
        if not self.states or self.states[-1] != mode:
            self.states.append(mode)
            if previous is not None:
                self.transitions.append({
                    "from": previous["mode"], "to": mode,
                    "sequence_gap": packet["sequence"] - previous["sequence"],
                    "current_change_ma": (vector(packet["current_after_clip_ma"], "current")
                                          - vector(previous["current_after_clip_ma"], "current")).tolist(),
                })
        self.started |= mode in ("TELEOP_SYNCING", "HOLD_SETTLING", "SYNCED")
        self.stopped |= (self.started and mode == "IDLE"
                         and not packet["torque_enabled_cached"]
                         and not packet["wrote_current"]
                         and bool(np.all(np.asarray(packet["current_after_clip_ma"]) == 0)))
        phase = self.phases.setdefault(mode, {
            "samples": 0, "first_monotonic_s": packet["monotonic_s"],
            "max_abs_error_rad": [0.0] * 8, "max_abs_velocity_rad_s": [0.0] * 8,
            "max_abs_current_before_clip_ma": [0.0] * 8,
            "max_abs_current_after_clip_ma": [0.0] * 8,
            "max_abs_present_current_ma": [0.0] * 8,
            "clipped_samples_per_axis": [0] * 8,
            "cycle_dt_count": 0, "cycle_dt_sum_s": 0.0, "cycle_dt_max_s": 0.0,
        })
        phase["samples"] += 1
        phase["last_monotonic_s"] = packet["monotonic_s"]
        for field in ("velocity_rad_s", "current_before_clip_ma", "current_after_clip_ma",
                      "present_current_ma"):
            key = f"max_abs_{field}"
            phase[key] = np.maximum(phase[key], np.abs(vector(packet[field], field))).tolist()
        if packet["target_position_rad"] is not None:
            error = vector(packet["target_position_rad"], "target") - vector(packet["position_rad"], "position")
            phase["max_abs_error_rad"] = np.maximum(phase["max_abs_error_rad"], np.abs(error)).tolist()
        clipped = np.abs(np.asarray(packet["current_before_clip_ma"])
                         - np.asarray(packet["current_after_clip_ma"])) > 1e-9
        phase["clipped_samples_per_axis"] = (np.asarray(phase["clipped_samples_per_axis"]) + clipped).tolist()
        dt = packet["cycle_dt_s"]
        if dt is not None:
            phase["cycle_dt_count"] += 1
            phase["cycle_dt_sum_s"] += dt
            phase["cycle_dt_max_s"] = max(phase["cycle_dt_max_s"], dt)
        self.previous = packet

    def result(self):
        return {
            "assessment": "RECORDED" if self.started and self.stopped else "INCOMPLETE",
            "states": self.states, "phases": self.phases, "transitions": self.transitions,
            "missing_samples": self.missing_samples, "sequence_restarts": self.sequence_restarts,
            "synced_observed": "SYNCED" in self.states,
            "idle_zero_output_and_cached_disable_observed": self.stopped,
            "follower_command_count": self.follower_command_count,
            "hardware_torque_disable_verified": False,
        }


class RecordStatus:
    """Show the same concise waiting conditions as move, without sending commands."""

    def __init__(self, config):
        self.sync = config.controller.sync_hold_control
        self.runtime = config.runtime
        self.packet = None
        self.last_active = None
        self.received_at = None
        self.last_display_at = float("-inf")
        self.last_display_key = None

    def observe(self, packet, *, received_at):
        self.packet, self.received_at = packet, received_at
        if packet["mode"] in ("TELEOP_SYNCING", "HOLD_SETTLING"):
            self.last_active = packet

    def render(self, *, now, ros_now):
        packet = self.packet
        age = None if packet is None else now - self.received_at
        stamp_age = None if packet is None else ros_now - packet["stamp_s"]
        freshness = ("WAITING" if packet is None else
                     "FRESH" if (0 <= age <= self.runtime.state_timeout
                                 and -self.runtime.future_tolerance <= stamp_age
                                 <= self.runtime.state_timeout) else "STALE")
        key = (None if packet is None else packet["mode"], freshness)
        changed = key != self.last_display_key
        if not changed and now - self.last_display_at < 1.0:
            return None
        self.last_display_at, self.last_display_key = now, key
        if packet is None:
            return "WAITING: 等待诊断消息；检查该侧 sync_diagnostics:=true 和 ROS_DOMAIN_ID。"
        if freshness == "STALE":
            return (f"STALE: 接收间隔={age:.3f} s，消息时间差={stamp_age:.3f} s；"
                    f"最后状态={packet['mode']}，当前状态和误差未知。")

        mode = packet["mode"]
        if mode == "SYNCED":
            return ("Synchronization complete: SYNCED. 同步完成；继续保持目标姿态。 "
                    "Use /mode/switch_stop to stop." if changed else None)
        if mode == "IDLE":
            if changed and self.last_active is not None:
                reason = (packet.get("sync_status") or {}).get("transition_reason")
                if reason:
                    return _sync_failure(packet, self.last_active["target_position_rad"], self.sync)
            return None
        if mode not in ("TELEOP_SYNCING", "HOLD_SETTLING"):
            return None
        status = packet.get("sync_status") or {}
        elapsed = status.get("mode_elapsed_s")
        timeout = (self.sync.ruckig.trajectory_timeout if mode == "TELEOP_SYNCING" else
                   self.sync.hold.settle_timeout)
        timing = "" if elapsed is None else f" ({elapsed:.3f} / {timeout:.3f} s)"
        if mode == "TELEOP_SYNCING":
            return "Waiting for HOLD_SETTLING: Ruckig reference trajectory is not finished" + timing
        return ("Waiting for SYNCED: HOLD_SETTLING" + timing + "\n"
                + _sync_conditions(packet, packet["target_position_rad"], self.sync.hold))


def fresh_idle(packet, *, side, ros_now, received_at, monotonic_now, runtime):
    if packet["side"] != side or packet["mode"] != "IDLE" or packet["torque_enabled_cached"]:
        raise ValueError("move requires selected arm IDLE with cached torque disabled")
    age = ros_now - packet["stamp_s"]
    receipt_age = monotonic_now - received_at
    if not (-runtime.future_tolerance <= age <= runtime.state_timeout
            and 0 <= receipt_age <= runtime.state_timeout):
        raise ValueError("leader diagnostics are stale")


def move_message(side, target, stamp):
    from sensor_msgs.msg import JointState
    msg = JointState()
    msg.header.stamp = stamp
    msg.name = [f"{'l' if side == 'left' else 'r'}_j{i}" for i in range(1, 8)]
    msg.position = list(target[:7])
    return msg


def _load(args):
    config = load_config(args.config, require_runtime=True)
    if args.side not in Path(config.controller.joint_gravity_comp.dynamics.leader_urdf).stem:
        raise ValueError("side does not match configuration URDF")
    return config


def _sync_conditions(packet, target, hold):
    """Describe measured blockers against the final goal, including the gripper."""
    position = vector(packet["position_rad"], "position")
    velocity = vector(packet["velocity_rad_s"], "velocity")
    target = vector(target, "target")
    lines = []
    for i in range(8):
        name = f"J{i + 1}" if i < 7 else "gripper(ID8)"
        error = abs(target[i] - position[i])
        if error > hold.sync_position_tolerance[i]:
            lines.append(f"{name}: |position error|={error:.6f} rad > "
                         f"{hold.sync_position_tolerance[i]:.6f} rad "
                         f"(actual={position[i]:.6f}, target={target[i]:.6f})")
        if abs(velocity[i]) > hold.sync_velocity_tolerance[i]:
            lines.append(f"{name}: |velocity|={abs(velocity[i]):.6f} rad/s > "
                         f"{hold.sync_velocity_tolerance[i]:.6f} rad/s")
    if not lines:
        lines.append("All eight axes meet instantaneous tolerances in this sample; "
                     f"they must remain within tolerance continuously for {hold.settle_duration:.3f} s.")
    status = packet.get("sync_status") or {}
    if packet["mode"] == "HOLD_SETTLING" and "settled_for_s" in status:
        lines.append(f"Continuous settling: {status['settled_for_s']:.3f} / "
                     f"{hold.settle_duration:.3f} s")
    return "\n".join(lines)


def _sync_failure(packet, target, sync):
    status = packet.get("sync_status") or {}
    reason = status.get("transition_reason")
    if reason == "sync_timeout":
        cause = (f"sync_timeout: Ruckig reference trajectory did not finish within "
                 f"{sync.ruckig.trajectory_timeout:.3f} s; HOLD_SETTLING was not reached")
    elif reason == "settle_timeout":
        cause = (f"settle_timeout: all eight axes did not remain within position/velocity "
                 f"tolerances continuously for {sync.hold.settle_duration:.3f} s "
                 f"within {sync.hold.settle_timeout:.3f} s of HOLD_SETTLING")
    else:
        cause = f"driver transition reason: {reason or 'not reported; inspect driver terminal'}"
    return cause + "\nLatest measured conditions:\n" + _sync_conditions(packet, target, sync.hold)


def _sync_effort(packet, clip_counts, sample_count):
    """Preserve output evidence from before IDLE clears the control terms."""
    if packet is None:
        return "\nNo active diagnostic sample was received."

    def value(field, index, precision=2):
        values = packet.get(field)
        return "--" if values is None else f"{values[index]:+.{precision}f}"

    lines = [f"\nLast active sample: mode={packet['mode']} sequence={packet['sequence']}",
             "Currents: physical ID order, mA; torques: logical joint order, N·m.",
             "CLIP counts cover received active samples only."]
    for i in range(8):
        name = f"J{i + 1}/ID{i + 1}" if i < 7 else "gripper/ID8"
        lines.append(
            f"{name}: present={value('present_current_ma', i)} mA, "
            f"command={value('current_before_clip_ma', i)} -> "
            f"{value('current_after_clip_ma', i)} mA, "
            f"limit={value('current_limit_ma', i)} mA, "
            f"CLIP samples={clip_counts[i]}/{sample_count}")
        lines.append(
            f"  torque: gravity={value('gravity_torque_nm', i, 5)}, "
            f"friction={value('friction_torque_nm', i, 5)}, "
            f"barrier={value('joint_limit_torque_nm', i, 5)}, "
            f"tracking={value('tracking_torque_nm', i, 5)} N·m")
    return "\n".join(lines)


def _wait_for_sync(node, latest, *, packet, target, config, wait_seconds):
    """Observe acknowledgement and completion without publishing more commands."""
    import rclpy

    deadline = time.monotonic() + wait_seconds
    sequence = packet["sequence"]
    mode = "IDLE"
    started = False
    last_progress_at = float("-inf")
    last_active = None
    active_samples = 0
    clip_counts = np.zeros(8, dtype=int)
    sync = config.controller.sync_hold_control
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        current, received_at = latest
        now = time.monotonic()
        age = node.get_clock().now().nanoseconds * 1e-9 - current["stamp_s"]
        if not (-config.runtime.future_tolerance <= age <= config.runtime.state_timeout
                and 0 <= now - received_at <= config.runtime.state_timeout):
            raise TimeoutError("leader diagnostics are stale; synchronization completion unknown; "
                               "inspect driver and use /mode/switch_stop to stop\n"
                               "Last received conditions (stale):\n"
                               + _sync_conditions(current, target, sync.hold)
                               + _sync_effort(last_active, clip_counts, active_samples))
        if current["side"] != packet["side"] or current["sequence"] <= sequence:
            continue
        sequence = current["sequence"]
        current_mode = current["mode"]
        if current_mode in ("TELEOP_SYNCING", "HOLD_SETTLING"):
            last_active = current
            active_samples += 1
            clip_counts += (np.abs(vector(current["current_before_clip_ma"], "current_before_clip_ma")
                                   - vector(current["current_after_clip_ma"], "current_after_clip_ma")) > 1e-9)
        if not started and current_mode == "IDLE":
            continue
        if not started:
            started = True
            deadline = (now + sync.ruckig.trajectory_timeout
                        + sync.hold.settle_timeout + wait_seconds)
        mode_changed = current_mode != mode
        if mode_changed:
            print(f"Observed mode: {current_mode}", flush=True)
            mode = current_mode
        if mode == "SYNCED":
            print("Synchronization complete: SYNCED. 同步完成；继续保持目标姿态。 "
                  "Use /mode/switch_stop to stop.", flush=True)
            return
        if mode not in ("TELEOP_SYNCING", "HOLD_SETTLING"):
            raise ValueError(f"synchronization did not complete: entered {mode} before SYNCED; "
                             + _sync_failure(current, target, sync)
                             + _sync_effort(last_active, clip_counts, active_samples))
        if now - last_progress_at >= 1.0 or (mode == "HOLD_SETTLING" and mode_changed):
            if mode == "HOLD_SETTLING":
                print("Waiting for SYNCED:\n" + _sync_conditions(current, target, sync.hold), flush=True)
            else:
                status = current.get("sync_status") or {}
                elapsed = status.get("mode_elapsed_s")
                timing = "" if elapsed is None else f" ({elapsed:.3f} / {sync.ruckig.trajectory_timeout:.3f} s)"
                print("Waiting for HOLD_SETTLING: Ruckig reference trajectory is not finished" + timing,
                      flush=True)
            last_progress_at = now
    if not started:
        raise TimeoutError("motion acknowledgement not observed; inspect driver and issue switch_stop")
    raise TimeoutError(f"synchronization completion not observed; last mode: {mode}; "
                       "inspect driver and use /mode/switch_stop to stop\n"
                       + _sync_failure(latest[0], target, sync)
                       + _sync_effort(last_active, clip_counts, active_samples))


def _move(args, config):
    def preview(position, velocity):
        return preview_move(config, position, velocity, joint_id=args.joint_id,
                            delta=args.delta, target=args.target, deltas=args.deltas)

    if args.start is not None:
        if args.execute:
            raise ValueError("--start is offline preview only; omit --execute")
        print(json.dumps(preview(args.start, [0.0] * 8), indent=2))
        return
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    from .ros_driver import command_qos, state_qos

    rclpy.init()
    node = Node(f"{args.side}_joint_move_trial")
    latest = []
    def receive(msg):
        latest[:] = [json.loads(msg.data), time.monotonic()]
    node.create_subscription(String, f"/leader_arm/{args.side}/sync_diagnostics", receive, command_qos())
    publisher = node.create_publisher(JointState, f"/{args.side}_joint_move", state_qos())
    try:
        deadline = time.monotonic() + args.wait_seconds
        while not latest or publisher.get_subscription_count() == 0:
            if time.monotonic() >= deadline:
                raise TimeoutError("no diagnostics or joint_move subscriber; start selected arm with sync_diagnostics:=true")
            rclpy.spin_once(node, timeout_sec=0.05)
        packet, received_at = latest
        fresh_idle(packet, side=args.side, ros_now=node.get_clock().now().nanoseconds * 1e-9,
                   received_at=received_at, monotonic_now=time.monotonic(), runtime=config.runtime)
        plan = preview(packet["position_rad"], packet["velocity_rad_s"])
        print(json.dumps(plan, indent=2), flush=True)
        if not args.execute:
            print("Preview only. Use --execute to send one target from a newly sampled pose.")
            return
        fresh_idle(packet, side=args.side, ros_now=node.get_clock().now().nanoseconds * 1e-9,
                   received_at=received_at, monotonic_now=time.monotonic(), runtime=config.runtime)
        publisher.publish(move_message(args.side, plan["target_position_rad"], node.get_clock().now().to_msg()))
        print("Target published once. Waiting for SYNCED; use /mode/switch_stop to stop.", flush=True)
        _wait_for_sync(node, latest, packet=packet, target=plan["target_position_rad"],
                       config=config, wait_seconds=args.wait_seconds)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def _record(args, config):
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String

    output = args.output
    output.parent.mkdir(parents=True, exist_ok=True)
    trace_path = output.with_suffix(".trace.jsonl")
    if output == trace_path or output.exists() or trace_path.exists():
        raise FileExistsError("choose a new .json output path; existing records are preserved")
    trial = TrialSummary()
    status_display = RecordStatus(config)
    started_utc = datetime.now(timezone.utc).isoformat()
    config_snapshot = asdict(config)
    rclpy.init()
    node = Node(f"{args.side}_sync_trial_recorder")
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
    try:
        with trace_path.open("x", encoding="utf-8") as trace:
            def receive(msg):
                packet = json.loads(msg.data)
                if packet["side"] != args.side:
                    raise ValueError("diagnostic side mismatch")
                received_at = time.monotonic()
                trace.write(json.dumps({"received_monotonic_s": received_at, "data": packet}) + "\n")
                previous_mode = trial.states[-1] if trial.states else None
                trial.observe(packet)
                status_display.observe(packet, received_at=received_at)
                if previous_mode != packet["mode"]:
                    trace.flush()
                    print(f"{packet['mode']} | samples={packet['sequence']} | lost={trial.missing_samples}", flush=True)
            def follower_command(msg):
                trial.follower_command_count += 1
                trace.write(json.dumps({"received_monotonic_s": time.monotonic(),
                                        "unexpected_follower_command": list(msg.position)}) + "\n")
            node.create_subscription(String, f"/leader_arm/{args.side}/sync_diagnostics", receive, qos)
            node.create_subscription(JointState, f"/gento/{args.side}_joint_control", follower_command, qos)
            print(f"Recording {trace_path}; awaiting move, then IDLE. Ctrl+C saves a partial record.", flush=True)
            deadline = time.monotonic() + args.duration
            try:
                while rclpy.ok() and time.monotonic() < deadline and not trial.stopped:
                    rclpy.spin_once(node, timeout_sec=0.1)
                    text = status_display.render(now=time.monotonic(),
                                                 ros_now=node.get_clock().now().nanoseconds * 1e-9)
                    if text is not None:
                        print(text, flush=True)
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
    finally:
        result = {"side": args.side, "operator": args.operator, "started_utc": started_utc,
                  "config_path": str(args.config.resolve()), "config": config_snapshot,
                  "trace_path": str(trace_path.resolve()), **trial.result()}
        with output.open("x", encoding="utf-8") as stream:
            json.dump(result, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print(f"{result['assessment']}: {output}; hardware acceptance requires observation and independent readback.")


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    move = commands.add_parser("move", help="preview or explicitly send one joint_move target")
    record = commands.add_parser("record", help="record one trial without publishing commands")
    monitor = commands.add_parser("monitor", help="show eight-axis torque or measured current and clipping")
    monitor.add_argument("--side", choices=("left", "right"), required=True)
    monitor.add_argument("--term", choices=("gravity", "friction", "limit", "current"), required=True)
    monitor.add_argument("--rate", type=float, default=5.0, help="display refresh rate in Hz (default: 5)")
    for child in (move, record):
        child.add_argument("--side", choices=("left", "right"), required=True)
        child.add_argument("--config", type=Path, required=True)
    move.add_argument("--joint-id", type=int, choices=range(1, 8))
    move.add_argument("--delta", type=float, help="relative radians for one joint")
    move.add_argument("--deltas", type=float, nargs=7,
                      help="relative radians for J1..J7 from the sampled pose; zero keeps that joint's position")
    move.add_argument("--target", type=float, nargs=7, help="absolute seven-joint logical target")
    move.add_argument("--start", type=float, nargs=8, help="offline start pose; never connects to ROS")
    move.add_argument("--execute", action="store_true", help="publish one target; default is preview only")
    move.add_argument("--wait-seconds", type=float, default=5.0,
                      help="connection/acknowledgement timeout and margin beyond configured sync timeouts")
    record.add_argument("--output", type=Path, required=True)
    record.add_argument("--operator", required=True)
    record.add_argument("--duration", type=float, default=120.0, help="maximum recording seconds; does not stop motion")
    args = parser.parse_args(argv)
    try:
        if args.command == "monitor":
            if not np.isfinite(args.rate) or not 0 < args.rate <= 50:
                raise ValueError("display rate must be finite and in (0, 50] Hz")
            from .torque_monitor import monitor_torque
            monitor_torque(args)
            return
        config = _load(args)
        duration = args.wait_seconds if args.command == "move" else args.duration
        if not np.isfinite(duration) or duration <= 0:
            raise ValueError("duration/wait must be finite and positive")
        (_move if args.command == "move" else _record)(args, config)
    except (ValueError, TimeoutError, FileExistsError) as exc:
        parser.exit(2, f"{exc}\n")


if __name__ == "__main__":
    main()
