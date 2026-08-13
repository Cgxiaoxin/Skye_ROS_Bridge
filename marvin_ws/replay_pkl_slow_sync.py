#!/usr/bin/env python3
"""Replay a recorded Marvin pkl episode through ROS 2 command topics.

Default mode is dry-run: it loads the requested pkl, prints the replay plan,
and does not publish any robot command. Add --execute to actually publish.

Typical dry-run:
  /home/wsj/miniconda3/envs/vitacsdk/bin/python replay_pkl.py --index 6

Actual replay:
  cd /home/wsj/pycode/marvin_ws
  source install/setup.bash
  /home/wsj/miniconda3/envs/vitacsdk/bin/python replay_pkl.py --index 6 --execute

sudo -E bash

cd /home/wsj/pycode/marvin_ws_humble_add_impedance/marvin_ws
source install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
conda activate vitacsdk
python replay_pkl_slow_sync.py   --index 37   --start 0   --end 1500   --speed 0.3   --sync-to-first-state   --require-subscribers   --execute

Safety:
- The script replays open-loop joint targets from the pkl.
- Make sure the robot workspace is clear.
- Start with dry-run and a short frame range first.
"""
from __future__ import annotations

import argparse
import pickle
import signal
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple

import numpy as np

DEFAULT_RECORDS_DIR = Path("/home/wsj/marvin_records/0612_new")
DEFAULT_LEFT_JOINT_TOPIC = "/left_joint_control"
DEFAULT_RIGHT_JOINT_TOPIC = "/right_joint_control"
DEFAULT_LEFT_GRIPPER_TOPIC = "/left_teleop_gripper/ctrl"
DEFAULT_RIGHT_GRIPPER_TOPIC = "/right_teleop_gripper/ctrl"
DEFAULT_SWITCH_TELEOP_TOPIC = "/mode/switch_teleop"
DEFAULT_SWITCH_STOP_TOPIC = "/mode/switch_stop"
DEFAULT_LEFT_CURRENT_JOINT_TOPIC = "/left_joint_state"
DEFAULT_RIGHT_CURRENT_JOINT_TOPIC = "/right_joint_state"


@dataclass(frozen=True)
class ReplayCommand:
    frame_index: int
    left_joint: Optional[np.ndarray]
    right_joint: Optional[np.ndarray]
    left_gripper: Optional[np.ndarray]
    right_gripper: Optional[np.ndarray]


def episode_path(records_dir: Path, index: int) -> Path:
    return records_dir / f"episode_{index:04d}.pkl"


def load_episode(path: Path) -> Dict[str, Any]:
    with path.open("rb") as f:
        data = pickle.load(f)
    if not isinstance(data, dict) or "frames" not in data:
        raise ValueError(f"Unsupported pkl format: {path}")
    return data


def as_vector(item: Optional[Dict[str, Any]], key: str = "position") -> Optional[np.ndarray]:
    if not isinstance(item, dict):
        return None
    data = item.get("data") or {}
    value = data.get(key)
    if value is None:
        return None
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
    except Exception:
        return None
    if arr.size == 0 or not np.isfinite(arr).all():
        return None
    return arr


def select_frame_indices(total: int, start: int, end: Optional[int], stride: int) -> List[int]:
    start = max(0, int(start))
    stop = total if end is None else min(total, int(end))
    if stop <= start:
        raise ValueError(f"Invalid frame range: start={start}, end={end}, total={total}")
    return list(range(start, stop, max(1, int(stride))))


def frame_to_command(frame: Dict[str, Any], frame_index: int, source: str, arms: set[str], include_grippers: bool) -> ReplayCommand:
    left = frame.get("left") or {}
    right = frame.get("right") or {}

    if source == "action":
        left_joint_item = left.get("action_joint_control")
        right_joint_item = right.get("action_joint_control")
        left_gripper_item = left.get("gripper_action")
        right_gripper_item = right.get("gripper_action")
    elif source == "state":
        left_joint_item = left.get("joint_state")
        right_joint_item = right.get("joint_state")
        left_gripper_item = left.get("gripper_state")
        right_gripper_item = right.get("gripper_state")
    else:
        raise ValueError(f"Unsupported source: {source}")

    return ReplayCommand(
        frame_index=frame_index,
        left_joint=as_vector(left_joint_item) if "left" in arms else None,
        right_joint=as_vector(right_joint_item) if "right" in arms else None,
        left_gripper=as_vector(left_gripper_item) if include_grippers and "left" in arms else None,
        right_gripper=as_vector(right_gripper_item) if include_grippers and "right" in arms else None,
    )


def build_commands(
    frames: List[Dict[str, Any]],
    indices: Iterable[int],
    source: str,
    arms: set[str],
    include_grippers: bool,
) -> List[ReplayCommand]:
    commands = [frame_to_command(frames[i], i, source, arms, include_grippers) for i in indices]
    usable = [cmd for cmd in commands if cmd.left_joint is not None or cmd.right_joint is not None]
    if not usable:
        raise RuntimeError(
            f"No usable joint commands found with source={source}. "
            "For this pkl, try --source state if action fields are empty."
        )
    return commands


def summarize_commands(commands: List[ReplayCommand]) -> Dict[str, int]:
    return {
        "left_joint": sum(cmd.left_joint is not None for cmd in commands),
        "right_joint": sum(cmd.right_joint is not None for cmd in commands),
        "left_gripper": sum(cmd.left_gripper is not None for cmd in commands),
        "right_gripper": sum(cmd.right_gripper is not None for cmd in commands),
    }


def vector_range(commands: List[ReplayCommand], attr: str) -> Optional[Tuple[np.ndarray, np.ndarray]]:
    values = [getattr(cmd, attr) for cmd in commands if getattr(cmd, attr) is not None]
    if not values:
        return None
    dim = max(v.size for v in values)
    mat = np.full((len(values), dim), np.nan, dtype=np.float64)
    for i, value in enumerate(values):
        mat[i, :value.size] = value
    return np.nanmin(mat, axis=0), np.nanmax(mat, axis=0)


def format_vec(vec: Optional[np.ndarray], precision: int = 4) -> str:
    if vec is None:
        return "None"
    return np.array2string(vec, precision=precision, suppress_small=True, separator=", ")


def metadata_fps(metadata: Dict[str, Any]) -> float:
    value = metadata.get("sample_rate_hz")
    try:
        fps = float(value)
    except Exception:
        fps = 30.0
    return fps if fps > 0 else 30.0


def print_plan(path: Path, data: Dict[str, Any], commands: List[ReplayCommand], fps: float, period: float, args: argparse.Namespace) -> None:
    metadata = data.get("metadata") or {}
    counts = summarize_commands(commands)
    first = commands[0]
    last = commands[-1]
    print("==================== PKL REPLAY PLAN ====================")
    print(f"input          : {path}")
    print(f"frames in pkl  : {len(data.get('frames') or [])}")
    print(f"selected frames: {len(commands)}  [{first.frame_index} -> {last.frame_index}]  stride={args.stride}")
    print(f"source         : {args.source}")
    print(f"arms           : {','.join(sorted(args.arms_set))}")
    print(f"fps            : {fps:.3f} Hz")
    print(f"speed          : {args.speed:.3f}x  command period={period:.4f}s")
    print(f"metadata fps   : {metadata.get('sample_rate_hz')}")
    print(f"metadata dur   : {metadata.get('duration_s')}")
    print(f"execute        : {args.execute}")
    print("available commands:")
    for key, count in counts.items():
        print(f"  {key:13s}: {count}/{len(commands)}")
    print("first command:")
    print(f"  left_joint   : {format_vec(first.left_joint)}")
    print(f"  right_joint  : {format_vec(first.right_joint)}")
    print(f"  left_gripper : {format_vec(first.left_gripper)}")
    print(f"  right_gripper: {format_vec(first.right_gripper)}")
    print("last command:")
    print(f"  left_joint   : {format_vec(last.left_joint)}")
    print(f"  right_joint  : {format_vec(last.right_joint)}")
    print(f"  left_gripper : {format_vec(last.left_gripper)}")
    print(f"  right_gripper: {format_vec(last.right_gripper)}")
    print("ranges:")
    for attr in ("left_joint", "right_joint", "left_gripper", "right_gripper"):
        rng = vector_range(commands, attr)
        if rng is None:
            print(f"  {attr:13s}: None")
        else:
            print(f"  {attr:13s}: min={format_vec(rng[0], 3)} max={format_vec(rng[1], 3)}")
    print("=========================================================")


def require_confirmation(args: argparse.Namespace) -> None:
    if not args.execute or args.yes:
        return
    print()
    print("SAFETY CONFIRMATION")
    print("This will publish open-loop robot commands to ROS 2 control topics.")
    print("Make sure the robot is powered, teleop/driver nodes are ready, and the workspace is clear.")
    print("Type 'REPLAY' to start, anything else to abort.")
    answer = input("> ").strip()
    if answer != "REPLAY":
        raise SystemExit("Aborted before publishing commands.")


def import_ros() -> Tuple[Any, Any, Any]:
    import rclpy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import String
    return rclpy, JointState, String


def make_joint_state(JointState: Any, node: Any, position: np.ndarray, names: Optional[List[str]] = None) -> Any:
    msg = JointState()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.name = names or []
    msg.position = [float(x) for x in np.asarray(position, dtype=np.float64).reshape(-1)]
    msg.velocity = []
    msg.effort = []
    return msg


def publish_command(node: Any, JointState: Any, pubs: Dict[str, Any], cmd: ReplayCommand) -> None:
    if cmd.left_joint is not None:
        pubs["left_joint"].publish(make_joint_state(JointState, node, cmd.left_joint))
    if cmd.right_joint is not None:
        pubs["right_joint"].publish(make_joint_state(JointState, node, cmd.right_joint))
    if cmd.left_gripper is not None:
        pubs["left_gripper"].publish(make_joint_state(JointState, node, cmd.left_gripper, ["gripper_joint"]))
    if cmd.right_gripper is not None:
        pubs["right_gripper"].publish(make_joint_state(JointState, node, cmd.right_gripper, ["gripper_joint"]))


def wait_for_joint_position(node: Any, JointState: Any, topic: str, timeout_s: float, qos_depth: int) -> np.ndarray:
    """Wait for one JointState message and return its position vector."""
    latest: Dict[str, Optional[np.ndarray]] = {"position": None}

    def callback(msg: Any) -> None:
        arr = np.asarray(msg.position, dtype=np.float64).reshape(-1)
        if arr.size > 0 and np.isfinite(arr).all():
            latest["position"] = arr

    sub = node.create_subscription(JointState, topic, callback, qos_depth)
    deadline = time.monotonic() + max(0.0, timeout_s)
    try:
        while time.monotonic() < deadline and latest["position"] is None:
            import rclpy
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        node.destroy_subscription(sub)

    if latest["position"] is None:
        raise RuntimeError(f"No valid JointState.position received from {topic} within {timeout_s:.2f}s")
    return latest["position"]


def smoothstep(alpha: float) -> float:
    alpha = float(np.clip(alpha, 0.0, 1.0))
    return alpha * alpha * (3.0 - 2.0 * alpha)


def interpolate_vector(start: np.ndarray, target: np.ndarray, alpha: float) -> np.ndarray:
    start = np.asarray(start, dtype=np.float64).reshape(-1)
    target = np.asarray(target, dtype=np.float64).reshape(-1)
    if start.size != target.size:
        raise ValueError(f"Vector dimension mismatch: current dim={start.size}, target dim={target.size}")
    return start + alpha * (target - start)


def slow_sync_to_first_state(node: Any, JointState: Any, pubs: Dict[str, Any], first_state: ReplayCommand, args: argparse.Namespace) -> None:
    """Slowly move the real arms from current measured joint state to the first dataset state."""
    targets: Dict[str, Optional[np.ndarray]] = {
        "left_joint": first_state.left_joint,
        "right_joint": first_state.right_joint,
    }
    current: Dict[str, Optional[np.ndarray]] = {"left_joint": None, "right_joint": None}

    if first_state.left_joint is not None:
        print(f"sync: waiting current left joint state from {args.left_current_joint_topic}")
        current["left_joint"] = wait_for_joint_position(
            node, JointState, args.left_current_joint_topic, args.sync_feedback_timeout, args.qos_depth
        )
    if first_state.right_joint is not None:
        print(f"sync: waiting current right joint state from {args.right_current_joint_topic}")
        current["right_joint"] = wait_for_joint_position(
            node, JointState, args.right_current_joint_topic, args.sync_feedback_timeout, args.qos_depth
        )

    steps = max(1, int(round(args.sync_seconds * args.sync_hz)))
    dt = 1.0 / max(args.sync_hz, 1e-6)
    print(
        f"sync: moving slowly to first dataset state for {args.sync_seconds:.2f}s "
        f"at {args.sync_hz:.1f} Hz, steps={steps}"
    )
    for i in range(steps + 1):
        a = smoothstep(i / steps)
        cmd = ReplayCommand(
            frame_index=first_state.frame_index,
            left_joint=interpolate_vector(current["left_joint"], targets["left_joint"], a)
            if targets["left_joint"] is not None else None,
            right_joint=interpolate_vector(current["right_joint"], targets["right_joint"], a)
            if targets["right_joint"] is not None else None,
            left_gripper=first_state.left_gripper if args.sync_grippers else None,
            right_gripper=first_state.right_gripper if args.sync_grippers else None,
        )
        publish_command(node, JointState, pubs, cmd)
        import rclpy
        rclpy.spin_once(node, timeout_sec=0.0)
        time.sleep(dt)
    print("sync: reached first dataset state target")


def wait_for_discovery(node: Any, pubs: Dict[str, Any], seconds: float, require_subscribers: bool) -> None:
    end = time.monotonic() + max(0.0, seconds)
    while time.monotonic() < end:
        time.sleep(0.05)
    print("publisher subscriber counts:")
    missing = []
    for name, pub in pubs.items():
        count = pub.get_subscription_count()
        print(f"  {name:13s}: {count}")
        if name.endswith("joint") and count == 0:
            missing.append(name)
    if missing:
        msg = f"No subscribers for joint command publishers: {missing}"
        if require_subscribers:
            raise RuntimeError(msg)
        print(f"WARNING: {msg}")


def execute_replay(commands: List[ReplayCommand], first_state: Optional[ReplayCommand], fps: float, period: float, args: argparse.Namespace) -> None:
    rclpy, JointState, String = import_ros()
    rclpy.init(args=None)
    node = rclpy.create_node("marvin_pkl_replay")
    interrupted = False

    def handle_signal(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    pubs = {
        "left_joint": node.create_publisher(JointState, args.left_joint_topic, args.qos_depth),
        "right_joint": node.create_publisher(JointState, args.right_joint_topic, args.qos_depth),
        "left_gripper": node.create_publisher(JointState, args.left_gripper_topic, args.qos_depth),
        "right_gripper": node.create_publisher(JointState, args.right_gripper_topic, args.qos_depth),
    }
    switch_teleop_pub = node.create_publisher(String, args.switch_teleop_topic, args.qos_depth)
    switch_stop_pub = node.create_publisher(String, args.switch_stop_topic, args.qos_depth)

    try:
        wait_for_discovery(node, pubs, args.discovery_wait, args.require_subscribers)

        if args.switch_teleop:
            msg = String()
            msg.data = "switch_teleop"
            for _ in range(max(1, args.switch_repeats)):
                switch_teleop_pub.publish(msg)
                time.sleep(0.05)
            print(f"published switch_teleop on {args.switch_teleop_topic}")

        if args.sync_to_first_state:
            if first_state is None or (first_state.left_joint is None and first_state.right_joint is None):
                raise RuntimeError("--sync-to-first-state requested, but first dataset state has no usable joint_state")
            slow_sync_to_first_state(node, JointState, pubs, first_state, args)

        if args.prep_seconds > 0:
            print(f"pre-position: publishing first command for {args.prep_seconds:.2f}s at {args.prep_hz:.1f} Hz")
            prep_period = 1.0 / max(args.prep_hz, 1e-6)
            end = time.monotonic() + args.prep_seconds
            while time.monotonic() < end:
                publish_command(node, JointState, pubs, commands[0])
                rclpy.spin_once(node, timeout_sec=0.0)
                time.sleep(prep_period)

        print("starting timed replay")
        start = time.monotonic()
        over_budget = 0
        sent = 0
        for out_idx, cmd in enumerate(commands):
            target_t = start + out_idx * period
            sleep_s = target_t - time.monotonic()
            if sleep_s > 0:
                time.sleep(sleep_s)
            else:
                over_budget += 1

            publish_command(node, JointState, pubs, cmd)
            rclpy.spin_once(node, timeout_sec=0.0)
            sent += 1
            if args.print_every > 0 and (out_idx == 0 or (out_idx + 1) % args.print_every == 0):
                print(
                    f"frame {out_idx + 1}/{len(commands)} src={cmd.frame_index} "
                    f"left={format_vec(cmd.left_joint, 3)} right={format_vec(cmd.right_joint, 3)}"
                )

        print(f"replay complete: sent={sent}/{len(commands)} over_budget={over_budget}")

    except KeyboardInterrupt:
        interrupted = True
        print("\nCtrl-C received; stopping replay loop")
    finally:
        if args.stop_on_finish:
            msg = String()
            msg.data = "switch_stop"
            try:
                switch_stop_pub.publish(msg)
                print(f"published switch_stop on {args.switch_stop_topic}")
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if interrupted:
        raise SystemExit(130)


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Replay a Marvin pkl episode by publishing recorded ROS JointState commands.")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--index", type=int, help="Episode index under --records-dir, e.g. 6 -> episode_0006.pkl")
    src.add_argument("--input", type=Path, help="Explicit pkl path")
    parser.add_argument("--records-dir", type=Path, default=DEFAULT_RECORDS_DIR)
    parser.add_argument("--source", choices=("action", "state"), default="action", help="Use recorded action commands or recorded states as joint targets.")
    parser.add_argument("--arms", choices=("left", "right", "both"), default="both")
    parser.add_argument("--skip-grippers", action="store_true")
    parser.add_argument("--start", type=int, default=0, help="First source frame index to replay.")
    parser.add_argument("--end", type=int, default=None, help="Exclusive source frame index. Default: end of episode.")
    parser.add_argument("--stride", type=int, default=1)
    parser.add_argument("--fps", type=float, default=0.0, help="Override replay fps. Default: pkl metadata sample_rate_hz.")
    parser.add_argument("--speed", type=float, default=1.0, help="Speed multiplier. 2.0 replays twice as fast.")
    parser.add_argument("--execute", action="store_true", help="Actually publish ROS commands. Without this, only dry-run summary is printed.")
    parser.add_argument("--yes", action="store_true", help="Skip interactive safety confirmation when --execute is set.")
    parser.add_argument("--prep-seconds", type=float, default=2.0, help="Publish first command for N seconds before timed replay.")
    parser.add_argument("--prep-hz", type=float, default=10.0)
    parser.add_argument("--sync-to-first-state", action="store_true", help="Before replay, slowly interpolate from current robot joint state to the first dataset joint_state.")
    parser.add_argument("--sync-seconds", type=float, default=5.0, help="Duration for slow synchronization to the first dataset state.")
    parser.add_argument("--sync-hz", type=float, default=20.0, help="Command frequency during slow synchronization.")
    parser.add_argument("--sync-feedback-timeout", type=float, default=3.0, help="Seconds to wait for current joint-state feedback topics.")
    parser.add_argument("--sync-grippers", action="store_true", help="Also command grippers to the first dataset gripper_state during synchronization. Default: arm joints only.")
    parser.add_argument("--left-current-joint-topic", default=DEFAULT_LEFT_CURRENT_JOINT_TOPIC, help="JointState feedback topic for current left arm position.")
    parser.add_argument("--right-current-joint-topic", default=DEFAULT_RIGHT_CURRENT_JOINT_TOPIC, help="JointState feedback topic for current right arm position.")
    parser.add_argument("--discovery-wait", type=float, default=1.0)
    parser.add_argument("--require-subscribers", action="store_true", help="Abort if joint command topics have no subscribers.")
    parser.add_argument("--print-every", type=int, default=30)
    parser.add_argument("--qos-depth", type=int, default=10)
    parser.add_argument("--left-joint-topic", default=DEFAULT_LEFT_JOINT_TOPIC)
    parser.add_argument("--right-joint-topic", default=DEFAULT_RIGHT_JOINT_TOPIC)
    parser.add_argument("--left-gripper-topic", default=DEFAULT_LEFT_GRIPPER_TOPIC)
    parser.add_argument("--right-gripper-topic", default=DEFAULT_RIGHT_GRIPPER_TOPIC)
    parser.add_argument("--switch-teleop", action=argparse.BooleanOptionalAction, default=True, help="Publish /mode/switch_teleop before replay.")
    parser.add_argument("--switch-repeats", type=int, default=3)
    parser.add_argument("--switch-teleop-topic", default=DEFAULT_SWITCH_TELEOP_TOPIC)
    parser.add_argument("--stop-on-finish", action=argparse.BooleanOptionalAction, default=True, help="Publish /mode/switch_stop after replay or interruption.")
    parser.add_argument("--switch-stop-topic", default=DEFAULT_SWITCH_STOP_TOPIC)
    args = parser.parse_args(argv)

    args.input_path = args.input if args.input is not None else episode_path(args.records_dir, args.index)
    if args.speed <= 0:
        parser.error("--speed must be > 0")
    if args.fps < 0:
        parser.error("--fps must be >= 0")
    if args.sync_seconds <= 0:
        parser.error("--sync-seconds must be > 0")
    if args.sync_hz <= 0:
        parser.error("--sync-hz must be > 0")
    if args.sync_feedback_timeout <= 0:
        parser.error("--sync-feedback-timeout must be > 0")
    args.arms_set = {"left", "right"} if args.arms == "both" else {args.arms}
    return args


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    path = args.input_path
    if not path.exists():
        raise FileNotFoundError(path)

    data = load_episode(path)
    frames = list(data.get("frames") or [])
    if not frames:
        raise RuntimeError(f"No frames in {path}")

    fps = float(args.fps or metadata_fps(data.get("metadata") or {}))
    period = (1.0 / fps) / args.speed
    indices = select_frame_indices(len(frames), args.start, args.end, args.stride)
    commands = build_commands(
        frames=frames,
        indices=indices,
        source=args.source,
        arms=args.arms_set,
        include_grippers=not args.skip_grippers,
    )

    first_state = frame_to_command(
        frames[indices[0]],
        indices[0],
        source="state",
        arms=args.arms_set,
        include_grippers=not args.skip_grippers,
    )

    print_plan(path, data, commands, fps, period, args)
    if args.sync_to_first_state:
        print("first dataset state used for slow sync:")
        print(f"  left_joint   : {format_vec(first_state.left_joint)}")
        print(f"  right_joint  : {format_vec(first_state.right_joint)}")
        print(f"  left_gripper : {format_vec(first_state.left_gripper)}")
        print(f"  right_gripper: {format_vec(first_state.right_gripper)}")
    require_confirmation(args)
    if args.execute:
        execute_replay(commands, first_state, fps, period, args)
    else:
        print("dry-run only. Add --execute to publish commands.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
