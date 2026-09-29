# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Measure two 500 Hz ROS adapters with real Pinocchio/Ruckig and fake hardware.

Run after sourcing the workspace install/setup.bash. This never opens a serial port.
"""

import argparse
from collections import defaultdict
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import platform
import time

import numpy as np
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from skye_factr_teleop.control_core import RuckigTrajectory
from skye_factr_teleop.dynamics import PinocchioDynamics
from skye_factr_teleop.ros_driver import SkyeFactrTeleopNode, event_qos, state_qos
from skye_factr_teleop.state_machine import ControlMode


class FakeHardware:
    def __init__(self):
        self.torque_enabled = False
        self.reads = 0
        self.writes = 0

    def open(self, *, enable_torque=False):
        self.torque_enabled = enable_torque

    def read_state(self):
        self.reads += 1
        return np.zeros(8), np.zeros(8)

    def write_currents(self, currents_ma):
        assert self.torque_enabled and len(currents_ma) == 8
        self.writes += 1

    def set_torque_enabled(self, enabled):
        self.torque_enabled = enabled

    def close(self):
        self.torque_enabled = False


class MeasuredDynamics(PinocchioDynamics):
    def __init__(self, path, samples):
        super().__init__(path)
        self.samples = samples

    def gravity(self, q, dq):
        start = time.perf_counter_ns()
        try:
            return super().gravity(q, dq)
        finally:
            self.samples["pinocchio_ns"].append(time.perf_counter_ns() - start)


class MeasuredTrajectory(RuckigTrajectory):
    def __init__(self, samples, **kwargs):
        super().__init__(**kwargs)
        self.samples = samples

    def update(self):
        start = time.perf_counter_ns()
        try:
            return super().update()
        finally:
            self.samples["ruckig_ns"].append(time.perf_counter_ns() - start)


class MeasuredNode(SkyeFactrTeleopNode):
    def __init__(self, side, samples, **kwargs):
        self.samples = samples
        super().__init__(side, **kwargs)

    def _tick(self):
        active = self.core is not None and self.core.mode == ControlMode.TELEOP_SYNCING
        start = time.perf_counter_ns()
        if active:
            self.samples["tick_starts_ns"].append(start)
        try:
            super()._tick()
        finally:
            if active:
                self.samples["tick_ns"].append(time.perf_counter_ns() - start)

    def _on_follower(self, message):
        start = time.perf_counter_ns()
        try:
            super()._on_follower(message)
        finally:
            self.samples["follower_callback_ns"].append(time.perf_counter_ns() - start)


def summary(values, divisor):
    values = np.asarray(values, dtype=float) / divisor
    if not values.size:
        return {"count": 0}
    return {"count": int(values.size), "mean": float(values.mean()),
            "p50": float(np.percentile(values, 50)),
            "p95": float(np.percentile(values, 95)),
            "p99": float(np.percentile(values, 99)), "max": float(values.max())}


def spin_until(executor, predicate, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        executor.spin_once(timeout_sec=0.01)
    raise RuntimeError("ROS graph or mode transition timed out")


def measure(package_root, seconds):
    context = Context()
    context.init(args=[], domain_id=207)
    executor = SingleThreadedExecutor(context=context)
    feed = rclpy.create_node("stage6_fake_follower", context=context,
                             use_global_arguments=False)
    nodes, hardware, samples = {}, {}, {}
    executor.add_node(feed)
    try:
        for side in ("left", "right"):
            samples[side] = defaultdict(list)
            hardware[side] = FakeHardware()
            config = package_root / "config" / f"grav_comp_m6_{side}.yaml"
            urdf = package_root.parent / "skye_factr_description" / "urdf" / f"m6{side}_isomorphism.urdf"
            if not config.is_file() or not urdf.is_file():
                raise FileNotFoundError(f"missing config or URDF for {side}")
            limits = {"max_velocity": [0.02] * 8,
                      "max_acceleration": [1.0] * 8, "max_jerk": [4.0] * 8}
            params = {"config_path": str(config), "joint_offsets": [0.0] * 8,
                      "current_limit_ma": 50.0, **limits}
            records = samples[side]
            node = MeasuredNode(
                side, records, context=context, use_global_arguments=False,
                cli_args=["--ros-args", "-r", f"/joint_state:=/gento/{side}_joint_states",
                          "-r", f"/joint_control:=/gento/{side}_joint_control"],
                parameter_overrides=[Parameter(name, value=value)
                                     for name, value in params.items()],
                hardware_factory=lambda cfg, side=side: hardware[side],
                dynamics_factory=lambda cfg, records=records, urdf=urdf:
                    MeasuredDynamics(urdf, records),
                trajectory_factory=lambda records=records, limits=limits:
                    MeasuredTrajectory(records, frequency=500.0, **limits),
            )
            nodes[side] = node
            executor.add_node(node)
        follower_pubs = {side: feed.create_publisher(
            JointState, f"/gento/{side}_joint_states", state_qos()) for side in nodes}
        sync_pub = feed.create_publisher(String, "/mode/switch_sync", event_qos())
        stop_pub = feed.create_publisher(String, "/mode/switch_stop", event_qos())

        def publish_followers():
            for side, pub in follower_pubs.items():
                message = JointState()
                message.header.stamp = feed.get_clock().now().to_msg()
                message.name = [f"{'l' if side == 'left' else 'r'}_j{i}" for i in range(1, 8)]
                message.position = [0.2] * 7
                pub.publish(message)

        feed.create_timer(0.01, publish_followers)
        spin_until(executor, lambda: all(node._follower is not None for node in nodes.values())
                   and feed.count_subscribers("/mode/switch_sync") == 2)
        sync_pub.publish(String(data="switch_sync"))
        spin_until(executor, lambda: all(node.core.mode == ControlMode.TELEOP_SYNCING
                                         for node in nodes.values()))
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.005)
        stop_pub.publish(String(data="switch_stop"))
        spin_until(executor, lambda: all(node.core.mode == ControlMode.IDLE
                                         for node in nodes.values()))

        result = {"timestamp_utc": datetime.now(timezone.utc).isoformat(),
                  "host": platform.node(), "platform": platform.platform(),
                  "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "default"),
                  "fake_hardware": True, "nodes": 2, "executor": "single_threaded",
                  "target_hz": 500, "measurement_seconds": seconds, "sides": {}}
        for side, records in samples.items():
            starts = np.asarray(records["tick_starts_ns"], dtype=np.int64)
            intervals = np.diff(starts)
            result["sides"][side] = {
                "timer_interval_ms": summary(intervals, 1e6),
                "tick_callback_us": summary(records["tick_ns"], 1e3),
                "pinocchio_us": summary(records["pinocchio_ns"], 1e3),
                "ruckig_us": summary(records["ruckig_ns"], 1e3),
                "follower_callback_us": summary(records["follower_callback_ns"], 1e3),
                "intervals_over_2ms": int(np.count_nonzero(intervals > 2_000_000)),
                "intervals_over_4ms": int(np.count_nonzero(intervals > 4_000_000)),
                "estimated_missed_timer_slots": int(sum(
                    max(0, round(interval / 2_000_000) - 1) for interval in intervals)),
                "fake_reads": hardware[side].reads,
                "fake_current_writes": hardware[side].writes,
            }
        return result
    finally:
        for node in nodes.values():
            executor.remove_node(node)
            node.destroy_node()
        executor.remove_node(feed)
        feed.destroy_node()
        executor.shutdown()
        context.try_shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")
    package_root = Path(__file__).resolve().parents[1]
    result = measure(package_root, args.seconds)
    output = json.dumps(result, indent=2, ensure_ascii=False) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(output)
    print(output, end="")


if __name__ == "__main__":
    main()
