#!/usr/bin/env python3
"""Record dual-arm teleop health: 小臂 publish + 大臂 feedback freshness.

Per side monitors:
  - /{side}_leader_arm/current_state          (小臂自报；懒汉/掉频主指标)
  - /leader_arm/{side}/sync_diagnostics       (需 sync_diagnostics:=true)
  - /gento/{side}_joint_states                (大臂反馈；复现 stale 断连)

Logs default to scripts/debug/logs/ (jsonl + summary). Terminal prints 1 Hz
status only; gaps / stale / stuck details stay in the file.

Usage (same ROS_DOMAIN_ID as teleop, e.g. Orin 21):

  conda deactivate
  source .../setup_factr_teleop_env.sh orin
  /usr/bin/python3 scripts/debug/record_leader_health.py

Ctrl+C or --duration to stop.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)
DIAG_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

MOTOR_NAMES = [f"ID{i}" for i in range(1, 9)]
SCRIPT_DIR = Path(__file__).resolve().parent
LOG_DIR = SCRIPT_DIR / "logs"


class SideTracker:
    """Per-arm counters; shares one jsonl writer."""

    def __init__(
        self,
        side: str,
        gap_s: float,
        freeze_rad: float,
        stuck_min_interval_s: float,
        follower_max_age_s: float,
        future_tolerance_s: float,
    ):
        self.side = side
        self.gap_s = gap_s
        self.freeze_rad = freeze_rad
        self.stuck_min_interval_s = stuck_min_interval_s
        self.follower_max_age_s = follower_max_age_s
        self.future_tolerance_s = future_tolerance_s
        self.js_topic = f"/{side}_leader_arm/current_state"
        self.diag_topic = f"/leader_arm/{side}/sync_diagnostics"
        self.follower_topic = f"/gento/{side}_joint_states"

        self._js_last_mono = None
        self._js_n = 0
        self._js_gaps = 0
        self._js_max_dt = 0.0
        self._js_prev_q = None
        self._js_n_window = 0
        self._js_max_dt_window = 0.0

        self._diag_n = 0
        self._diag_last_seq = None
        self._diag_missing = 0
        self._diag_restarts = 0
        self._diag_max_cycle_dt = 0.0
        self._diag_n_window = 0
        self._diag_max_cycle_window = 0.0

        self._fl_last_mono = None
        self._fl_n = 0
        self._fl_gaps = 0
        self._fl_max_dt = 0.0
        self._fl_n_window = 0
        self._fl_max_dt_window = 0.0
        self._fl_stale = 0
        self._fl_future = 0
        self._fl_stale_window = 0
        self._fl_max_age = 0.0
        self._fl_max_age_window = 0.0
        self._fl_last_stale_log_mono = 0.0

        self._motor_freeze = defaultdict(int)
        self._motor_high_i_stuck = defaultdict(int)
        self._motor_max_abs_i = [0.0] * 8
        self._motor_max_abs_dq = [0.0] * 8
        self._stuck_window = defaultdict(int)
        self._last_stuck_log_mono = 0.0

    def on_js(self, msg: JointState, emit):
        now = time.monotonic()
        self._js_n += 1
        self._js_n_window += 1
        q = list(msg.position) if msg.position else []
        if self._js_last_mono is not None:
            dt = now - self._js_last_mono
            self._js_max_dt = max(self._js_max_dt, dt)
            self._js_max_dt_window = max(self._js_max_dt_window, dt)
            if dt > self.gap_s:
                self._js_gaps += 1
                emit("js_gap", dt_ms=round(dt * 1000, 1), n=self._js_n, q=q[:8])
            if self._js_prev_q and len(q) >= 7 and len(self._js_prev_q) >= 7:
                for i in range(min(8, len(q), len(self._js_prev_q))):
                    dq = abs(q[i] - self._js_prev_q[i])
                    self._motor_max_abs_dq[i] = max(self._motor_max_abs_dq[i], dq)
                    if dq < self.freeze_rad:
                        self._motor_freeze[i] += 1
        self._js_last_mono = now
        self._js_prev_q = q

    def on_follower(self, msg: JointState, ros_now: float, emit):
        """Mirror teleop validate_joint_state age check on /gento/*_joint_states."""
        now = time.monotonic()
        self._fl_n += 1
        self._fl_n_window += 1
        if self._fl_last_mono is not None:
            dt = now - self._fl_last_mono
            self._fl_max_dt = max(self._fl_max_dt, dt)
            self._fl_max_dt_window = max(self._fl_max_dt_window, dt)
            if dt > self.gap_s:
                self._fl_gaps += 1
                emit("follower_gap", dt_ms=round(dt * 1000, 1), n=self._fl_n)
        self._fl_last_mono = now

        sec = int(msg.header.stamp.sec)
        nanosec = int(msg.header.stamp.nanosec)
        if sec < 0 or not 0 <= nanosec < 1_000_000_000 or (sec == 0 and nanosec == 0):
            self._fl_stale += 1
            self._fl_stale_window += 1
            emit("follower_bad_stamp", sec=sec, nanosec=nanosec)
            return

        stamp = sec + nanosec * 1e-9
        age = ros_now - stamp
        self._fl_max_age = max(self._fl_max_age, age)
        self._fl_max_age_window = max(self._fl_max_age_window, age)

        if age > self.follower_max_age_s:
            self._fl_stale += 1
            self._fl_stale_window += 1
            if now - self._fl_last_stale_log_mono >= self.stuck_min_interval_s:
                self._fl_last_stale_log_mono = now
                emit(
                    "follower_stale",
                    age_s=round(age, 4),
                    max_age_s=self.follower_max_age_s,
                    stamp_s=round(stamp, 6),
                    ros_now_s=round(ros_now, 6),
                )
        elif -age > self.future_tolerance_s:
            self._fl_future += 1
            self._fl_stale_window += 1
            if now - self._fl_last_stale_log_mono >= self.stuck_min_interval_s:
                self._fl_last_stale_log_mono = now
                emit(
                    "follower_future",
                    age_s=round(age, 4),
                    future_tol_s=self.future_tolerance_s,
                )

    def on_diag(self, msg: String, emit):
        try:
            packet = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            emit("diag_bad_json", error=str(exc))
            return
        self._diag_n += 1
        self._diag_n_window += 1
        seq = packet.get("sequence")
        if self._diag_last_seq is not None and seq is not None:
            jump = seq - self._diag_last_seq
            if jump <= 0:
                self._diag_restarts += 1
                emit("diag_seq_restart", prev=self._diag_last_seq, seq=seq)
            elif jump > 1:
                miss = jump - 1
                self._diag_missing += miss
                emit("diag_seq_gap", prev=self._diag_last_seq, seq=seq, missing=miss)
        if seq is not None:
            self._diag_last_seq = seq

        cdt = packet.get("cycle_dt_s")
        if isinstance(cdt, (int, float)) and math.isfinite(cdt):
            self._diag_max_cycle_dt = max(self._diag_max_cycle_dt, float(cdt))
            self._diag_max_cycle_window = max(self._diag_max_cycle_window, float(cdt))
            if cdt > self.gap_s:
                emit(
                    "diag_slow_cycle",
                    cycle_dt_ms=round(cdt * 1000, 1),
                    mode=packet.get("mode"),
                    seq=seq,
                )

        present = packet.get("present_current_ma") or []
        for i in range(min(8, len(present))):
            try:
                i_ma = abs(float(present[i]))
            except (TypeError, ValueError):
                continue
            self._motor_max_abs_i[i] = max(self._motor_max_abs_i[i], i_ma)

        vel = packet.get("velocity_rad_s") or []
        after = packet.get("current_after_clip_ma") or []
        stuck = []
        for i in range(min(8, len(present), len(vel), len(after))):
            try:
                i_ma = abs(float(present[i]))
                v = abs(float(vel[i]))
                cmd = abs(float(after[i]))
            except (TypeError, ValueError):
                continue
            if cmd > 50 and i_ma > 40 and v < 0.02:
                self._motor_high_i_stuck[i] += 1
                self._stuck_window[i] += 1
                stuck.append(MOTOR_NAMES[i])
        if stuck:
            now = time.monotonic()
            if now - self._last_stuck_log_mono >= self.stuck_min_interval_s:
                self._last_stuck_log_mono = now
                emit(
                    "motor_stuck_hint",
                    motors=stuck,
                    mode=packet.get("mode"),
                    seq=seq,
                    present_mA=[round(abs(float(x)), 1) for x in present[:8]],
                    vel=[round(float(x), 4) for x in vel[:8]],
                )

    def take_window(self, window_s: float) -> dict:
        js_hz = self._js_n_window / window_s if window_s > 0 else 0.0
        diag_hz = self._diag_n_window / window_s if window_s > 0 else 0.0
        fl_hz = self._fl_n_window / window_s if window_s > 0 else 0.0
        stuck = {
            MOTOR_NAMES[i]: int(self._stuck_window[i])
            for i in range(8) if self._stuck_window[i]
        }
        row = {
            "js_hz": round(js_hz, 1),
            "diag_hz": round(diag_hz, 1),
            "follower_hz": round(fl_hz, 1),
            "js_n": self._js_n_window,
            "diag_n": self._diag_n_window,
            "follower_n": self._fl_n_window,
            "max_dt_ms": round(self._js_max_dt_window * 1000, 1),
            "follower_max_dt_ms": round(self._fl_max_dt_window * 1000, 1),
            "follower_max_age_ms": round(self._fl_max_age_window * 1000, 1),
            "follower_stale_n": self._fl_stale_window,
            "max_cycle_dt_ms": round(self._diag_max_cycle_window * 1000, 1),
            "stuck_counts": stuck or None,
        }
        self._js_n_window = 0
        self._diag_n_window = 0
        self._fl_n_window = 0
        self._js_max_dt_window = 0.0
        self._diag_max_cycle_window = 0.0
        self._fl_max_dt_window = 0.0
        self._fl_max_age_window = 0.0
        self._fl_stale_window = 0
        self._stuck_window = defaultdict(int)
        return row

    def summary(self, duration_s: float) -> dict:
        return {
            "side": self.side,
            "duration_s": round(duration_s, 2),
            "joint_state": {
                "topic": self.js_topic,
                "samples": self._js_n,
                "hz": round(self._js_n / duration_s, 1) if duration_s > 0 else 0.0,
                "gaps": self._js_gaps,
                "max_dt_ms": round(self._js_max_dt * 1000, 1),
            },
            "follower_state": {
                "topic": self.follower_topic,
                "samples": self._fl_n,
                "hz": round(self._fl_n / duration_s, 1) if duration_s > 0 else 0.0,
                "gaps": self._fl_gaps,
                "max_dt_ms": round(self._fl_max_dt * 1000, 1),
                "stale_count": self._fl_stale,
                "future_count": self._fl_future,
                "max_stamp_age_ms": round(self._fl_max_age * 1000, 1),
                "max_age_s": self.follower_max_age_s,
                "future_tol_s": self.future_tolerance_s,
            },
            "diagnostics": {
                "topic": self.diag_topic,
                "samples": self._diag_n,
                "hz": round(self._diag_n / duration_s, 1) if duration_s > 0 else 0.0,
                "seq_missing": self._diag_missing,
                "seq_restarts": self._diag_restarts,
                "max_cycle_dt_ms": round(self._diag_max_cycle_dt * 1000, 1),
            },
            "motors": [
                {
                    "id": i + 1,
                    "name": MOTOR_NAMES[i],
                    "max_abs_present_mA": round(self._motor_max_abs_i[i], 1),
                    "max_abs_dq_rad": round(self._motor_max_abs_dq[i], 5),
                    "js_near_freeze_counts": int(self._motor_freeze[i]),
                    "high_I_low_vel_counts": int(self._motor_high_i_stuck[i]),
                }
                for i in range(8)
            ],
        }


class DualLeaderHealthRecorder(Node):
    def __init__(
        self,
        sides: list[str],
        gap_ms: float,
        freeze_rad: float,
        out_path: Path,
        status_period_s: float,
        stuck_min_interval_s: float,
        follower_max_age_s: float,
        future_tolerance_s: float,
        watch_follower: bool,
    ):
        super().__init__("leader_health_dual")
        self.out_path = out_path
        self.out = out_path.open("w", encoding="utf-8")
        self.status_period_s = status_period_s
        self.watch_follower = watch_follower
        self._t0 = time.time()
        self._last_status_mono = time.monotonic()
        self.trackers = {
            side: SideTracker(
                side, gap_ms / 1000.0, freeze_rad, stuck_min_interval_s,
                follower_max_age_s, future_tolerance_s,
            )
            for side in sides
        }
        for side, tr in self.trackers.items():
            self.create_subscription(
                JointState, tr.js_topic,
                lambda msg, s=side: self.trackers[s].on_js(msg, self._emit_fn(s)),
                STATE_QOS,
            )
            self.create_subscription(
                String, tr.diag_topic,
                lambda msg, s=side: self.trackers[s].on_diag(msg, self._emit_fn(s)),
                DIAG_QOS,
            )
            if watch_follower:
                self.create_subscription(
                    JointState, tr.follower_topic,
                    lambda msg, s=side: self._on_follower(s, msg),
                    STATE_QOS,
                )
        self.create_timer(status_period_s, self._on_status)
        self._emit(
            None, "session_start",
            sides=sides,
            gap_ms=gap_ms,
            freeze_rad=freeze_rad,
            status_period_s=status_period_s,
            follower_max_age_s=follower_max_age_s,
            future_tolerance_s=future_tolerance_s,
            watch_follower=watch_follower,
            out=str(out_path),
        )
        self.get_logger().info(
            f"recording sides={sides} follower={watch_follower} -> {out_path} "
            f"(status every {status_period_s:.0f}s; events file-only)"
        )

    def _ros_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_follower(self, side: str, msg: JointState):
        self.trackers[side].on_follower(msg, self._ros_now(), self._emit_fn(side))

    def _emit_fn(self, side: str):
        def emit(kind: str, **fields):
            self._emit(side, kind, **fields)
        return emit

    def _emit(self, side: str | None, kind: str, **fields):
        row = {"t_wall": time.time(), "kind": kind, **fields}
        if side is not None:
            row["side"] = side
        self.out.write(json.dumps(row, ensure_ascii=False) + "\n")
        self.out.flush()

    def _on_status(self):
        now = time.monotonic()
        window_s = now - self._last_status_mono
        self._last_status_mono = now
        elapsed = time.time() - self._t0
        parts = []
        sides_payload = {}
        for side, tr in self.trackers.items():
            w = tr.take_window(window_s)
            sides_payload[side] = w
            stuck = w["stuck_counts"]
            stuck_s = f" stuck={stuck}" if stuck else ""
            fl = ""
            if self.watch_follower:
                stale = w["follower_stale_n"]
                stale_s = f" stale={stale}" if stale else ""
                fl = (
                    f" fl={w['follower_hz']:6.1f}Hz "
                    f"age={w['follower_max_age_ms']:5.1f}ms{stale_s}"
                )
            parts.append(
                f"{side} js={w['js_hz']:6.1f}Hz max_dt={w['max_dt_ms']:5.1f}ms"
                f"{fl}{stuck_s}"
            )
        self._emit(
            None, "hz_sample",
            elapsed_s=round(elapsed, 2),
            window_s=round(window_s, 3),
            sides=sides_payload,
        )
        print(f"[{elapsed:6.1f}s] " + " | ".join(parts), flush=True)

    def build_summary(self, duration_s: float) -> dict:
        sides = {s: tr.summary(duration_s) for s, tr in self.trackers.items()}
        suspects = {}
        for side, summary in sides.items():
            hits = [
                m for m in summary["motors"]
                if m["high_I_low_vel_counts"] >= 5 or (
                    m["js_near_freeze_counts"] > summary["joint_state"]["samples"] * 0.8
                    and summary["joint_state"]["samples"] > 100
                    and m["max_abs_dq_rad"] < 2e-4
                )
            ]
            if hits:
                suspects[side] = [m["name"] for m in hits]
        return {
            "duration_s": round(duration_s, 2),
            "out": str(self.out_path),
            "sides": sides,
            "suspect_motors": suspects,
            "notes": {
                "hz_sample": "periodic window Hz in jsonl (kind=hz_sample)",
                "leader": "js_* = /{side}_leader_arm/current_state",
                "follower": (
                    "follower_* / follower_stale = /gento/{side}_joint_states "
                    "stamp age vs teleop max_age"
                ),
                "events": (
                    "js_gap / follower_stale / diag_* / throttled "
                    "motor_stuck_hint also in jsonl"
                ),
            },
        }

    def close_out(self):
        self.out.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sides", default="both",
        help="left, right, or both (default both)",
    )
    parser.add_argument("--duration", type=float, default=0.0,
                        help="seconds; 0 = until Ctrl+C")
    parser.add_argument("--gap-ms", type=float, default=20.0)
    parser.add_argument("--freeze-rad", type=float, default=1e-4)
    parser.add_argument(
        "--status-period", type=float, default=1.0,
        help="seconds between terminal Hz lines and hz_sample rows",
    )
    parser.add_argument(
        "--stuck-log-interval", type=float, default=1.0,
        help="min seconds between motor_stuck_hint / follower_stale rows (file)",
    )
    parser.add_argument(
        "--follower-max-age", type=float, default=0.50,
        help="mirror teleop runtime.state_timeout (s); stamp older => follower_stale",
    )
    parser.add_argument(
        "--future-tolerance", type=float, default=0.05,
        help="mirror teleop runtime.future_tolerance (s)",
    )
    parser.add_argument(
        "--no-follower", action="store_true",
        help="only watch leader current_state + diagnostics",
    )
    parser.add_argument(
        "--out", type=Path, default=None,
        help="jsonl path (default: scripts/debug/logs/leader_health_<timestamp>.jsonl)",
    )
    args = parser.parse_args()

    side_arg = args.sides.strip().lower()
    if side_arg in ("both", "dual", "lr", "all"):
        sides = ["left", "right"]
    elif side_arg in ("left", "right"):
        sides = [side_arg]
    else:
        parser.error("--sides must be left, right, or both")

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_path = args.out or (LOG_DIR / f"leader_health_{stamp}.jsonl")
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = DualLeaderHealthRecorder(
        sides, args.gap_ms, args.freeze_rad, out_path,
        args.status_period, args.stuck_log_interval,
        args.follower_max_age, args.future_tolerance,
        watch_follower=not args.no_follower,
    )
    print(f"logging -> {out_path}", flush=True)
    print("terminal: 1Hz status only; detail in jsonl. Ctrl+C to stop.", flush=True)
    t0 = time.time()
    try:
        while rclpy.ok():
            if args.duration > 0 and (time.time() - t0) >= args.duration:
                break
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    duration = time.time() - t0
    node._on_status()
    summary = node.build_summary(duration)
    summary_path = out_path.with_suffix(".summary.json")
    summary_path.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")
    node._emit(None, "session_end", duration_s=round(duration, 2), summary=summary_path.name)
    node.close_out()

    print(json.dumps({
        "duration_s": summary["duration_s"],
        "out": str(out_path),
        "summary": str(summary_path),
        "joint_state_hz": {
            s: summary["sides"][s]["joint_state"]["hz"] for s in summary["sides"]
        },
        "follower_hz": {
            s: summary["sides"][s]["follower_state"]["hz"] for s in summary["sides"]
        },
        "follower_stale": {
            s: summary["sides"][s]["follower_state"]["stale_count"]
            for s in summary["sides"]
        },
        "suspect_motors": summary["suspect_motors"],
    }, indent=2, ensure_ascii=False), flush=True)
    for side, s in summary["sides"].items():
        if s["diagnostics"]["samples"] == 0:
            print(
                f"NOTE: {side}: no sync_diagnostics — launch with sync_diagnostics:=true",
                flush=True,
            )
        if not args.no_follower and s["follower_state"]["samples"] == 0:
            print(
                f"NOTE: {side}: no /gento/{side}_joint_states — is skye_robot_driver up?",
                flush=True,
            )
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
