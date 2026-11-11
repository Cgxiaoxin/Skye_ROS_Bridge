#!/usr/bin/env python3
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0

"""Record leader/follower rates and per-motor state while teleop runs.

Runs *alongside* the teleop launch (it never touches the Dynamixel serial port,
so the driver keeps full ownership of the bus) and answers one question:
which topic stopped, when, and what were the motors doing at that moment.

Records
-------
  /left_leader_arm/current_state      leader joints  (l_j1..l_j7)
  /right_leader_arm/current_state     leader joints
  /gento/left_joint_states            follower joints
  /gento/right_joint_states           follower joints
  /leader_arm/{left,right}/sync_diagnostics   per-servo current + cycle time
  /teleop/state                       IDLE / SYNCED / TELEOP transitions

Output (one timestamped directory per run, safe to archive)
-----------------------------------------------------------
  meta.json            host, git commit, args, topic graph at start/stop
  leader_left.csv      wall time, mono time, dt, 7 positions, 7 velocities
  sync_diag_left.csv   wall time, cycle dt, torque flag, 8 currents, 8 limits
  ...
  events.jsonl         GAP / SILENT / JUMP / NAN / RATE / MODE / DRIVER-LOG
  summary.json         rate + gap stats, per-joint and per-servo extremes
  driver_log.txt       matching lines from the teleop driver log, if found

Usage
-----
    # terminal 1
    ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
        robot_profile:=orin sync_diagnostics:=true
    # terminal 2
    /usr/bin/python3 scripts/debug/teleop_motor_diag.py --seconds 300

Output goes to scripts/debug/logs/teleop_diag_<timestamp>/ (git-ignored).
Press Ctrl+C to stop early; the summary and meta files are always written.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import re
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import (
        DurabilityPolicy,
        HistoryPolicy,
        QoSProfile,
        ReliabilityPolicy,
    )
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
except ImportError as exc:  # pragma: no cover - environment guard
    sys.exit(
        "ROS environment missing: source skye_ros2_ws/install/setup.bash and use "
        f"/usr/bin/python3 ({exc})"
    )

SCRIPT_VERSION = "1.0"
REPO_ROOT = Path(__file__).resolve().parents[2]
LOG_ROOT = Path(__file__).resolve().parent / "logs"
LEADER_JOINTS = 7
SERVO_COUNT = 8

_JOINT_SAMPLE = "joint"
_DIAG_SAMPLE = "diag"


def state_qos() -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.VOLATILE)


def command_qos() -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.VOLATILE)


def latched_qos() -> QoSProfile:
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                      reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


def wall_iso() -> str:
    return datetime.now().astimezone().isoformat(timespec="milliseconds")


class CsvWriter:
    """Minimal buffered CSV writer; one file per topic."""

    def __init__(self, path: Path, header: list[str]) -> None:
        self.path = path
        self._file = path.open("w", encoding="utf-8", newline="")
        self._buffer: list[str] = []
        self._file.write(",".join(header) + "\n")

    def row(self, values: list[str]) -> None:
        self._buffer.append(",".join(values))
        if len(self._buffer) >= 200:
            self.flush()

    def flush(self) -> None:
        if self._buffer:
            self._file.write("\n".join(self._buffer) + "\n")
            self._buffer.clear()
        self._file.flush()

    def close(self) -> None:
        self.flush()
        self._file.close()


class TopicStats:
    def __init__(self, name: str, expected_hz: float, gap_s: float) -> None:
        self.name = name
        self.expected_hz = expected_hz
        self.gap_s = gap_s
        self.count = 0
        self.first_s: float | None = None
        self.last_s: float | None = None
        self.max_gap_s = 0.0
        self.gaps = 0
        self.silent_episodes = 0
        self.silent = False
        self.intervals: list[float] = []
        self.last_mode = "UNKNOWN"

    def on_message(self, now: float) -> float | None:
        """Return the inter-arrival interval in seconds, if measurable."""
        self.count += 1
        if self.first_s is None:
            self.first_s = now
        interval = None if self.last_s is None else now - self.last_s
        self.last_s = now
        if interval is not None:
            self.intervals.append(interval)
            if interval > self.max_gap_s:
                self.max_gap_s = interval
            if interval > self.gap_s:
                self.gaps += 1
        self.silent = False
        return interval

    def hz(self) -> float:
        if self.first_s is None or self.last_s is None:
            return 0.0
        span = self.last_s - self.first_s
        return (self.count - 1) / span if span > 1e-9 and self.count > 1 else 0.0

    def percentile(self, fraction: float) -> float:
        if not self.intervals:
            return 0.0
        ordered = sorted(self.intervals)
        return ordered[min(len(ordered) - 1, int(len(ordered) * fraction))]

    def snapshot(self) -> dict[str, Any]:
        return {
            "count": self.count,
            "hz": round(self.hz(), 2),
            "gaps_over_threshold": self.gaps,
            "max_gap_ms": round(self.max_gap_s * 1000, 2),
            "p50_interval_ms": round(self.percentile(0.5) * 1000, 3),
            "p99_interval_ms": round(self.percentile(0.99) * 1000, 3),
            "silent_episodes": self.silent_episodes,
            "last_mode": self.last_mode,
        }


class JointTracker:
    """Per-joint extremes and discontinuity detection."""

    def __init__(self, names: list[str], jump_rad: float) -> None:
        self.names = names
        self.jump_rad = jump_rad
        count = len(names)
        self.count = [0] * count
        self.nan = [0] * count
        self.max_delta = [0.0] * count
        self.max_abs_vel = [0.0] * count
        self.min_pos = [math.inf] * count
        self.max_pos = [-math.inf] * count
        self.prev: list[float | None] = [None] * count

    def update(self, position: list[float], velocity: list[float]) -> list[dict[str, Any]]:
        jumps: list[dict[str, Any]] = []
        for index in range(len(self.names)):
            pos = position[index] if index < len(position) else math.nan
            vel = velocity[index] if index < len(velocity) else 0.0
            self.count[index] += 1
            if not math.isfinite(pos):
                self.nan[index] += 1
                continue
            self.min_pos[index] = min(self.min_pos[index], pos)
            self.max_pos[index] = max(self.max_pos[index], pos)
            if math.isfinite(vel):
                self.max_abs_vel[index] = max(self.max_abs_vel[index], abs(vel))
            previous = self.prev[index]
            if previous is not None:
                delta = abs(pos - previous)
                self.max_delta[index] = max(self.max_delta[index], delta)
                if delta > self.jump_rad:
                    jumps.append({
                        "joint": self.names[index], "delta_rad": round(pos - previous, 4),
                    })
            self.prev[index] = pos
        return jumps

    def snapshot(self) -> dict[str, Any]:
        return {
            name: {
                "samples": self.count[i],
                "nan": self.nan[i],
                "max_step_rad": round(self.max_delta[i], 4),
                "max_abs_velocity_rad_s": round(self.max_abs_vel[i], 3),
                "position_range_rad": [
                    round(self.min_pos[i], 4) if math.isfinite(self.min_pos[i]) else None,
                    round(self.max_pos[i], 4) if math.isfinite(self.max_pos[i]) else None,
                ],
            }
            for i, name in enumerate(self.names)
        }


class ServoTracker:
    """Per-servo current extremes from sync_diagnostics."""

    def __init__(self) -> None:
        self.max_current = [0.0] * SERVO_COUNT
        self.sum_current = [0.0] * SERVO_COUNT
        self.samples = [0] * SERVO_COUNT
        self.limit = [0.0] * SERVO_COUNT
        self.max_cycle_dt = 0.0
        self.torque_changes: list[dict[str, Any]] = []
        self._torque: bool | None = None

    def update(self, payload: dict[str, Any], now_s: float) -> list[dict[str, Any]]:
        events: list[dict[str, Any]] = []
        currents = payload.get("present_current_ma") or []
        if isinstance(currents, list) and len(currents) == SERVO_COUNT:
            for index, value in enumerate(currents):
                if not isinstance(value, (int, float)) or not math.isfinite(value):
                    continue
                magnitude = abs(float(value))
                self.max_current[index] = max(self.max_current[index], magnitude)
                self.sum_current[index] += magnitude
                self.samples[index] += 1
        limits = payload.get("current_limit_ma") or []
        if isinstance(limits, list) and len(limits) == SERVO_COUNT:
            self.limit = [float(v) if isinstance(v, (int, float)) else 0.0 for v in limits]
        cycle = payload.get("cycle_dt_s")
        if isinstance(cycle, (int, float)) and math.isfinite(cycle):
            if cycle > self.max_cycle_dt:
                self.max_cycle_dt = float(cycle)
            if cycle > 0.02:
                events.append({"kind": "SLOW-CYCLE", "cycle_dt_ms": round(cycle * 1000, 2)})
        torque = payload.get("torque_enabled_cached")
        if isinstance(torque, bool):
            if self._torque is not None and torque != self._torque:
                events.append({
                    "kind": "TORQUE", "from": self._torque, "to": torque,
                })
            self._torque = torque
        return events

    def snapshot(self) -> dict[str, Any]:
        return {
            "max_cycle_dt_ms": round(self.max_cycle_dt * 1000, 2),
            "servos": {
                f"id{i + 1}": {
                    "max_current_ma": round(self.max_current[i], 1),
                    "mean_current_ma": round(self.sum_current[i] / self.samples[i], 1)
                    if self.samples[i] else 0.0,
                    "current_limit_ma": self.limit[i],
                    "samples": self.samples[i],
                }
                for i in range(SERVO_COUNT)
            },
        }


class DriverLogTail:
    """Follow the newest ROS log file and keep the interesting lines."""

    PATTERN = re.compile(
        r"(status packet|Hardware Error|read_state|disable torque|Control stopped|"
        r"DROPOUT|silent|exception|Traceback|Error)", re.IGNORECASE,
    )

    def __init__(self, glob: str) -> None:
        self.glob = Path(glob).expanduser()
        self.path: Path | None = None
        self.offset = 0
        self.hits: list[str] = []

    def poll(self) -> list[str]:
        try:
            candidates = sorted(_glob(self.glob), key=lambda item: item.stat().st_mtime)
        except OSError:
            candidates = []
        if not candidates:
            return []
        newest = candidates[-1]
        if self.path != newest:
            self.path = newest
            self.offset = newest.stat().st_size if self.offset == 0 else 0
            if self.offset:
                return []
        try:
            with self.path.open("r", encoding="utf-8", errors="replace") as handle:
                handle.seek(self.offset)
                chunk = handle.read()
                self.offset = handle.tell()
        except OSError:
            return []
        lines = [line for line in chunk.splitlines() if self.PATTERN.search(line)]
        self.hits.extend(lines)
        return lines


class Recorder(Node):
    def __init__(self, args: argparse.Namespace, out_dir: Path) -> None:
        super().__init__("teleop_motor_diag")
        self.args = args
        self.out_dir = out_dir
        self.started = time.monotonic()
        self.events_file = (out_dir / "events.jsonl").open("w", encoding="utf-8")
        self.events = 0
        self.event_log: list[dict[str, Any]] = []
        self.mode = "UNKNOWN"
        self.mode_history: list[dict[str, Any]] = []
        self.stop = False
        self.reported_missing = False
        self.graph_checked = False
        self.log_tail = DriverLogTail(args.log_glob) if args.log_glob else None

        specs = [
            ("leader_left", "/left_leader_arm/current_state", _JOINT_SAMPLE),
            ("leader_right", "/right_leader_arm/current_state", _JOINT_SAMPLE),
            ("follower_left", "/gento/left_joint_states", _JOINT_SAMPLE),
            ("follower_right", "/gento/right_joint_states", _JOINT_SAMPLE),
            ("sync_diag_left", "/leader_arm/left/sync_diagnostics", _DIAG_SAMPLE),
            ("sync_diag_right", "/leader_arm/right/sync_diagnostics", _DIAG_SAMPLE),
        ]
        self.stats: dict[str, TopicStats] = {}
        self.writers: dict[str, CsvWriter] = {}
        self.joint_trackers: dict[str, JointTracker] = {}
        self.servo_trackers: dict[str, ServoTracker] = {}
        self.last_payload: dict[str, dict[str, Any]] = {}

        for key, topic, kind in specs:
            gap = max(0.25 / args.expect_hz, args.gap_ms / 1000.0)
            self.stats[key] = TopicStats(topic, args.expect_hz, gap)
            if kind == _JOINT_SAMPLE:
                self.writers[key] = CsvWriter(
                    out_dir / f"{key}.csv",
                    ["wall", "mono_s", "seq", "dt_ms"]
                    + [f"{name}_pos" for name in _default_names(key)]
                    + [f"{name}_vel" for name in _default_names(key)],
                )
                self.joint_trackers[key] = JointTracker(_default_names(key), args.jump_rad)
                self.create_subscription(
                    JointState, topic, self._make_joint_cb(key), state_qos(),
                )
            else:
                self.writers[key] = CsvWriter(
                    out_dir / f"{key}.csv",
                    ["wall", "mono_s", "cycle_dt_ms", "torque"]
                    + [f"cur{i}" for i in range(1, SERVO_COUNT + 1)]
                    + [f"lim{i}" for i in range(1, SERVO_COUNT + 1)],
                )
                self.servo_trackers[key] = ServoTracker()
                self.create_subscription(
                    String, topic, self._make_diag_cb(key), command_qos(),
                )

        self.create_subscription(String, "/teleop/state", self._on_mode, latched_qos())
        self.create_timer(0.2, self._watchdog)
        self.create_timer(1.0, self._status)

    # -- callbacks -----------------------------------------------------------
    def _make_joint_cb(self, key: str):
        stats = self.stats[key]
        writer = self.writers[key]
        tracker = self.joint_trackers[key]

        def callback(message: JointState) -> None:
            now = time.monotonic()
            interval = stats.on_message(now)
            stats.last_mode = self.mode
            position = list(message.position)
            velocity = list(message.velocity)
            seq = stats.count
            dt_ms = "" if interval is None else f"{interval * 1000:.3f}"
            writer.row(
                [wall_iso(), f"{now - self.started:.4f}", str(seq), dt_ms]
                + [f"{value:.6f}" for value in position]
                + [f"{value:.6f}" for value in velocity]
            )
            self.last_payload[key] = {
                "position": position, "velocity": velocity, "names": list(message.name),
                "mono_s": round(now - self.started, 4),
            }
            jumps = tracker.update(position, velocity)
            for jump in jumps:
                self.emit("JUMP", key=key, **jump)
            if any(not math.isfinite(value) for value in position):
                self.emit("NAN", key=key)
            if interval is not None and interval > stats.gap_s:
                self.emit(
                    "GAP", key=key,
                    interval_ms=round(interval * 1000, 2),
                    limit_ms=round(stats.gap_s * 1000, 2),
                )

        return callback

    def _make_diag_cb(self, key: str):
        stats = self.stats[key]
        writer = self.writers[key]
        tracker = self.servo_trackers[key]

        def callback(message: String) -> None:
            now = time.monotonic()
            stats.on_message(now)
            stats.last_mode = self.mode
            try:
                payload = json.loads(message.data)
            except json.JSONDecodeError:
                self.emit("BAD-JSON", key=key, raw=message.data[:120])
                return
            if not isinstance(payload, dict):
                return
            self.last_payload[key] = payload | {"mono_s": round(now - self.started, 4)}
            cycle = payload.get("cycle_dt_s")
            torque = payload.get("torque_enabled_cached")
            currents = payload.get("present_current_ma") or []
            limits = payload.get("current_limit_ma") or []
            writer.row(
                [wall_iso(), f"{now - self.started:.4f}",
                 f"{cycle * 1000:.2f}" if isinstance(cycle, (int, float)) else "",
                 str(torque)]
                + [_fmt(value) for value in currents]
                + [_fmt(value) for value in limits]
            )
            for event in tracker.update(payload, now - self.started):
                self.emit(event.pop("kind"), key=key, **event)

        return callback

    def _on_mode(self, message: String) -> None:
        if message.data == self.mode:
            return
        previous = self.mode
        self.mode = message.data
        now = time.monotonic() - self.started
        self.mode_history.append({
            "mono_s": round(now, 3), "wall": wall_iso(),
            "from": previous, "to": self.mode,
        })
        self.emit("MODE", **{"from": previous, "to": self.mode})

    # -- supervision ---------------------------------------------------------
    def _watchdog(self) -> None:
        now = time.monotonic()
        silence = self.args.silent_ms / 1000.0
        for key, stats in self.stats.items():
            if stats.last_s is None or stats.silent:
                continue
            if now - stats.last_s > silence:
                stats.silent = True
                stats.silent_episodes += 1
                self.emit(
                    "SILENT", key=key,
                    silent_ms=round((now - stats.last_s) * 1000, 1),
                    count=stats.count,
                )
        if not self.graph_checked and now - self.started > 2.5:
            self.graph_checked = True
            self._check_graph()
        if not self.reported_missing and now - self.started > 3.0:
            self.reported_missing = True
            for key, stats in self.stats.items():
                if stats.count == 0:
                    self.emit("NO-DATA", key=key)
        if self.log_tail is not None:
            for line in self.log_tail.poll():
                self.emit("DRIVER-LOG", line=line.strip()[:400])

    def _check_graph(self) -> None:
        """Distinguish 'publisher is silent' from 'wrong ROS_DOMAIN_ID / RMW'."""
        missing = [
            stats.name for stats in self.stats.values() if self.count_publishers(stats.name) == 0
        ]
        env = _env_summary()
        if not missing:
            return
        self.emit("TOPIC-ABSENT", missing=missing, **env)
        if len(missing) == len(self.stats):
            print(
                "\n" + "!" * 78 + "\n"
                "no expected topic exists in this ROS graph -- almost certainly an "
                "environment mismatch,\nnot a hardware fault.\n"
                f"  ROS_DOMAIN_ID={env['ros_domain_id']}  "
                f"RMW_IMPLEMENTATION={env['rmw_implementation']}\n"
                f"  FASTRTPS_DEFAULT_PROFILES_FILE={env['fastrtps_profile'] or '(unset)'}\n"
                "  teleop runs on domain 21 (robot_profile=orin). Start the recorder in an "
                "identical shell:\n"
                "    conda deactivate\n"
                "    source skye_ros2_ws/scripts/setup_factr_teleop_env.sh orin\n"
                "    /usr/bin/python3 scripts/debug/teleop_motor_diag.py\n"
                + "!" * 78 + "\n",
                flush=True,
            )
        else:
            print(f"\nmissing topics (no publisher in graph): {missing}\n", flush=True)

    def _status(self) -> None:
        elapsed = time.monotonic() - self.started
        parts = [f"{elapsed:6.1f}s", f"mode={self.mode}"]
        for key, stats in self.stats.items():
            parts.append(f"{key}={stats.hz():5.1f}Hz")
        parts.append(f"events={self.events}")
        print("\r" + " ".join(parts)[:180], end="", flush=True)

    # -- output --------------------------------------------------------------
    def emit(self, kind: str, **fields: Any) -> None:
        self.events += 1
        record = {
            "wall": wall_iso(),
            "mono_s": round(time.monotonic() - self.started, 4),
            "kind": kind,
            "mode": self.mode,
        }
        record.update(fields)
        record["context"] = {
            key: _compact(value) for key, value in self.last_payload.items()
        }
        self.event_log.append(record)
        self.events_file.write(json.dumps(record, ensure_ascii=False) + "\n")
        self.events_file.flush()

    def close(self) -> None:
        for writer in self.writers.values():
            writer.close()
        self.events_file.close()

    def summary(self) -> dict[str, Any]:
        return {
            "script_version": SCRIPT_VERSION,
            "started_wall": self.args.started_wall,
            "duration_s": round(time.monotonic() - self.started, 2),
            "events": self.events,
            "modes": self.mode_history,
            "topics": {key: stats.snapshot() for key, stats in self.stats.items()},
            "joints": {key: tracker.snapshot() for key, tracker in self.joint_trackers.items()},
            "servos": {key: tracker.snapshot() for key, tracker in self.servo_trackers.items()},
            "driver_log_hits": len(self.log_tail.hits) if self.log_tail else 0,
            "diagnosis": _collect_hints(self.event_log),
        }


def _collect_hints(records: list[dict[str, Any]]) -> list[str]:
    """One hint per outage episode; a stall emits SILENT and GAP for the same gap."""
    hints: list[str] = []
    last_seen: dict[str, float] = {}
    for record in records:
        hint = _hint(record)
        if hint is None:
            continue
        key = str(record.get("key", "?"))
        when = float(record.get("mono_s", 0.0))
        if when - last_seen.get(key, -10.0) < 0.6:
            continue
        last_seen[key] = when
        hints.append(hint)
    return hints


def _hint(record: dict[str, Any]) -> str | None:
    """One line that a human can act on, per anomaly."""
    kind = record.get("kind")
    if kind not in ("SILENT", "GAP", "NO-DATA"):
        return None
    key = record.get("key", "?")
    context = record.get("context", {})
    side = "left" if key.endswith("_left") else "right"
    diag = context.get(f"sync_diag_{side}")
    top: list[tuple[int, float]] = []
    if isinstance(diag, dict):
        currents = diag.get("present_current_ma") or []
        pairs = [(index + 1, float(value)) for index, value in enumerate(currents)
                 if isinstance(value, (int, float))]
        pairs.sort(key=lambda item: -abs(item[1]))
        top = pairs[:2]
    when = f"{record.get('mono_s', 0.0):.2f}s, mode={record.get('mode')}"
    if kind == "NO-DATA":
        return (f"{key}: no message during the run ({when}) -- topic not published, "
                f"wrong name, or sync_diagnostics disabled")
    amount = record.get("silent_ms") or record.get("interval_ms")
    text = f"{key}: stalled {amount} ms at {when}"
    if top:
        text += "; servo current then: " + ", ".join(
            f"ID{servo}={abs(value):.0f}mA" for servo, value in top
        )
    return text


def _env_summary() -> dict[str, str]:
    return {
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0 (unset)"),
        "rmw_implementation": os.environ.get("RMW_IMPLEMENTATION", "(unset)"),
        "fastrtps_profile": os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE", ""),
        "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", "(unset)"),
    }


def _glob(pattern: Path) -> list[Path]:
    """Expand a pattern containing '**' or wildcards without assuming the root exists."""
    parts = pattern.parts
    root_parts: list[str] = []
    for part in parts:
        if any(token in part for token in "*?["):
            break
        root_parts.append(part)
    root = Path(*root_parts) if root_parts else Path("/")
    tail = str(Path(*parts[len(root_parts):]))
    return list(root.glob(tail)) if root.exists() else []


def _fmt(value: Any) -> str:
    return f"{value:.2f}" if isinstance(value, (int, float)) else ""


def _compact(payload: dict[str, Any]) -> dict[str, Any]:
    keep = ("mono_s", "cycle_dt_s", "torque_enabled_cached", "mode")
    compact = {key: payload[key] for key in keep if key in payload}
    currents = payload.get("present_current_ma")
    if isinstance(currents, list):
        compact["present_current_ma"] = [round(float(v), 1) if isinstance(v, (int, float)) else None
                                         for v in currents]
    velocity = payload.get("velocity")
    if isinstance(velocity, list):
        compact["max_abs_velocity"] = round(max((abs(v) for v in velocity), default=0.0), 3)
    return compact


def _default_names(key: str) -> list[str]:
    if key.startswith("follower"):
        return [f"j{i}" for i in range(1, LEADER_JOINTS + 1)]
    prefix = "l" if key.endswith("_left") else "r"
    return [f"{prefix}_j{i}" for i in range(1, LEADER_JOINTS + 1)]


def _git_commit() -> str:
    try:
        return subprocess.run(
            ["git", "-C", str(REPO_ROOT), "rev-parse", "--short", "HEAD"],
            capture_output=True, text=True, timeout=3, check=False,
        ).stdout.strip() or "unknown"
    except (OSError, subprocess.SubprocessError):
        return "unknown"


def _install_signals(recorder_ref: dict[str, Any]) -> None:
    def handler(signum, frame) -> None:
        count = recorder_ref.get("signals", 0) + 1
        recorder_ref["signals"] = count
        if count == 1:
            print("\nstop requested: writing summary ...", flush=True)
            recorder_ref["stop"] = True
            return
        print("\nforced exit", flush=True)
        os._exit(130)

    signal.signal(signal.SIGINT, handler)
    signal.signal(signal.SIGTERM, handler)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record teleop topic rates and per-servo state for post-mortem.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--seconds", type=float, default=0.0,
                        help="stop after N seconds, 0 runs until Ctrl+C")
    parser.add_argument("--out-dir", type=Path, default=LOG_ROOT,
                        help="parent directory for the run folder")
    parser.add_argument("--expect-hz", type=float, default=250.0,
                        help="nominal topic rate, used to size gap thresholds")
    parser.add_argument("--gap-ms", type=float, default=50.0,
                        help="flag an inter-arrival interval above this")
    parser.add_argument("--silent-ms", type=float, default=300.0,
                        help="flag a topic as stalled above this")
    parser.add_argument("--jump-rad", type=float, default=0.15,
                        help="flag a per-sample position step above this")
    parser.add_argument("--log-glob", default="~/.ros/log/**/*.log",
                        help="driver log glob to tail, empty disables")
    parser.add_argument("--no-log", action="store_true",
                        help="shorthand for --log-glob ''")
    parser.add_argument("--domain", type=int, default=None,
                        help="set ROS_DOMAIN_ID before init (must match the teleop launch, "
                             "orin profile uses 21)")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_dir = args.out_dir / f"teleop_diag_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=False)
    args.started_wall = wall_iso()
    if args.no_log:
        args.log_glob = ""
    if args.domain is not None:
        os.environ["ROS_DOMAIN_ID"] = str(args.domain)

    rclpy.init()
    recorder = Recorder(args, out_dir)
    state = {"signals": 0, "stop": False}
    _install_signals(state)
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)

    print(f"recording to {out_dir}")
    print("topics: leader L/R, follower L/R, sync_diagnostics L/R, /teleop/state")
    deadline = time.monotonic() + args.seconds if args.seconds > 0 else float("inf")
    meta = {
        "script": str(Path(__file__).resolve()),
        "script_version": SCRIPT_VERSION,
        "started_wall": args.started_wall,
        "host": socket.gethostname(),
        "user": os.environ.get("USER", "unknown"),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0 (unset)"),
        "git_commit": _git_commit(),
        "argv": sys.argv[1:],
        "arguments": {key: str(value) for key, value in vars(args).items()},
        "topics_at_start": {},
        "env": _env_summary(),
    }
    meta["topics_at_start"] = {
        name: sorted(types) for name, types in recorder.get_topic_names_and_types()
    }
    (out_dir / "meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8",
    )

    try:
        while not state["stop"] and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
    finally:
        print()
        try:
            summary = recorder.summary()
            meta_stop = {
                "stopped_wall": wall_iso(),
                "topics_at_stop": {
                    name: sorted(types)
                    for name, types in recorder.get_topic_names_and_types()
                },
                "nodes_at_stop": recorder.get_node_names(),
            }
            (out_dir / "meta_stop.json").write_text(
                json.dumps(meta_stop, ensure_ascii=False, indent=2) + "\n",
                encoding="utf-8",
            )
            (out_dir / "summary.json").write_text(
                json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8",
            )
            if recorder.log_tail is not None and recorder.log_tail.hits:
                (out_dir / "driver_log.txt").write_text(
                    "\n".join(recorder.log_tail.hits) + "\n", encoding="utf-8",
                )
            _print_summary(summary, out_dir)
        finally:
            recorder.close()
            executor.shutdown()
            try:
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass
    return 0


def _print_summary(summary: dict[str, Any], out_dir: Path) -> None:
    print(f"duration {summary['duration_s']}s, events {summary['events']}")
    print(f"{'topic':<34}{'count':>8}{'Hz':>8}{'gaps':>6}{'max gap ms':>12}{'silent':>8}")
    for key, stats in summary["topics"].items():
        print(
            f"{key:<34}{stats['count']:>8}{stats['hz']:>8.1f}{stats['gaps_over_threshold']:>6}"
            f"{stats['max_gap_ms']:>12.1f}{stats['silent_episodes']:>8}"
        )
    print("servo ID -> joint: ID1..2 = l_j1..l_j2 (XM430-W350), ID3..7 = l_j3..l_j7, ID8 = gripper")
    if summary.get("diagnosis"):
        print("diagnosis:")
        for line in summary["diagnosis"]:
            print(f"  - {line}")
    if summary["events"]:
        print("anomalies recorded -> events.jsonl (see JUMP / GAP / SILENT / SLOW-CYCLE)")
    else:
        print("no anomalies: every topic kept its rate and every joint stayed continuous")
    print(f"files: {out_dir}")


if __name__ == "__main__":
    sys.exit(main())
