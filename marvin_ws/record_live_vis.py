#!/usr/bin/python3
from __future__ import annotations

import argparse
import os
import pickle
import select
import shlex
import shutil
import signal
import subprocess
import sys
import termios
import threading
import time
import tty
from dataclasses import dataclass
from typing import Any, Callable, Dict, Iterable, List, Optional

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState, PointCloud2, PointField


@dataclass(frozen=True)
class TopicSpec:
    key: str
    topic: str
    msg_type: Any
    converter: Callable[[Any], Dict[str, Any]]


DEFAULT_CONFIG_FILE = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "tele_operation",
    "config",
    "real_world_env.yaml",
)
DEFAULT_RGB_TOPICS = [
    "/external_camera/color/image_raw",
    "/right_wrist_camera/color/image_raw",
    "/left_wrist_camera/color/image_raw",
]
DEFAULT_TACTILE_RGB_TOPICS = [
    "/right_gripper_sensor_1/color/image_raw",
    "/right_gripper_sensor_2/color/image_raw",
]
DEFAULT_TACTILE_MARKER_TOPICS = [
    "/right_gripper_sensor_1/marker_offset/information",
    "/right_gripper_sensor_2/marker_offset/information",
]


class SpeechNotifier:
    def __init__(self, logger: Any, enabled: bool = True, command: str = "") -> None:
        self.logger = logger
        self.enabled = enabled
        self.command = str(command or "").strip()
        self.warned_unavailable = False

    def say(self, text: str) -> None:
        if not self.enabled:
            return
        try:
            cmd = self._build_command(text)
        except ValueError as exc:
            self.logger.warn(f"Invalid --speech-command: {exc}")
            return
        if not cmd:
            if not self.warned_unavailable:
                self.logger.warn(
                    "Speech requested but no TTS command was found. "
                    "Install speech-dispatcher/espeak-ng or set --speech-command."
                )
                self.warned_unavailable = True
            return
        try:
            subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        except Exception as exc:
            self.logger.warn(f"Failed to run speech command {cmd[0]}: {exc}")

    def _build_command(self, text: str) -> Optional[List[str]]:
        if self.command:
            parts = shlex.split(self.command)
            if not parts:
                return None
            if any("{text}" in part for part in parts):
                return [part.replace("{text}", text) for part in parts]
            return parts + [text]

        candidates = [
            ("spd-say", ["spd-say", "-l", "zh", text]),
            ("espeak-ng", ["espeak-ng", "-v", "zh", text]),
            ("espeak", ["espeak", "-v", "zh", text]),
            ("say", ["say", text]),
        ]
        for executable, cmd in candidates:
            if shutil.which(executable):
                return cmd
        return None


def stamp_to_float(stamp: Any) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def header_dict(msg: Any) -> Dict[str, Any]:
    header = getattr(msg, "header", None)
    if header is None:
        return {"stamp": None, "frame_id": ""}
    return {
        "stamp": stamp_to_float(header.stamp),
        "frame_id": str(header.frame_id),
    }


def joint_state_to_dict(msg: JointState) -> Dict[str, Any]:
    return {
        "header": header_dict(msg),
        "name": list(msg.name),
        "position": np.asarray(msg.position, dtype=np.float64),
        "velocity": np.asarray(msg.velocity, dtype=np.float64),
        "effort": np.asarray(msg.effort, dtype=np.float64),
    }


def pose_stamped_to_dict(msg: PoseStamped) -> Dict[str, Any]:
    p = msg.pose.position
    q = msg.pose.orientation
    return {
        "header": header_dict(msg),
        "position": np.asarray([p.x, p.y, p.z], dtype=np.float64),
        "orientation_xyzw": np.asarray([q.x, q.y, q.z, q.w], dtype=np.float64),
    }


def wrench_stamped_to_dict(msg: WrenchStamped) -> Dict[str, Any]:
    f = msg.wrench.force
    t = msg.wrench.torque
    return {
        "header": header_dict(msg),
        "force": np.asarray([f.x, f.y, f.z], dtype=np.float64),
        "torque": np.asarray([t.x, t.y, t.z], dtype=np.float64),
    }


def image_to_dict(msg: Image) -> Dict[str, Any]:
    return {
        "header": header_dict(msg),
        "height": int(msg.height),
        "width": int(msg.width),
        "encoding": str(msg.encoding),
        "is_bigendian": int(msg.is_bigendian),
        "step": int(msg.step),
        # In this project the camera publishers put JPEG-encoded bytes in Image.data.
        # Keep bytes unchanged so the recorder does not depend on OpenCV.
        "data": bytes(msg.data),
    }


def _point_field_dtype(datatype: int) -> Optional[Any]:
    mapping = {
        PointField.INT8: np.int8,
        PointField.UINT8: np.uint8,
        PointField.INT16: np.int16,
        PointField.UINT16: np.uint16,
        PointField.INT32: np.int32,
        PointField.UINT32: np.uint32,
        PointField.FLOAT32: np.float32,
        PointField.FLOAT64: np.float64,
    }
    return mapping.get(int(datatype))


def pointcloud2_to_dict(msg: PointCloud2) -> Dict[str, Any]:
    point_count = int(msg.width) * int(msg.height)
    fields_meta = [
        {
            "name": str(f.name),
            "offset": int(f.offset),
            "datatype": int(f.datatype),
            "count": int(f.count),
        }
        for f in msg.fields
    ]

    arrays: Dict[str, np.ndarray] = {}
    if point_count > 0 and msg.point_step > 0 and msg.data:
        dtype_fields = []
        for f in msg.fields:
            base_dtype = _point_field_dtype(f.datatype)
            if base_dtype is None:
                continue
            if int(f.count) == 1:
                dtype_fields.append((str(f.name), base_dtype, int(f.offset)))
            else:
                dtype_fields.append((str(f.name), base_dtype, (int(f.count),), int(f.offset)))

        if dtype_fields:
            dtype = np.dtype({
                "names": [x[0] for x in dtype_fields],
                "formats": [x[1] if len(x) == 3 else (x[1], x[2]) for x in dtype_fields],
                "offsets": [x[-1] for x in dtype_fields],
                "itemsize": int(msg.point_step),
            })
            cloud = np.frombuffer(bytes(msg.data), dtype=dtype, count=point_count)
            for name in dtype.names or []:
                arrays[name] = np.asarray(cloud[name]).copy()

    return {
        "header": header_dict(msg),
        "height": int(msg.height),
        "width": int(msg.width),
        "point_step": int(msg.point_step),
        "row_step": int(msg.row_step),
        "is_bigendian": bool(msg.is_bigendian),
        "is_dense": bool(msg.is_dense),
        "fields": fields_meta,
        "points": arrays,
        "raw_data": bytes(msg.data),
    }


class PklTeleopRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("pkl_teleop_recorder")
        self.args = args
        self.latest_lock = threading.Lock()
        self.state_lock = threading.RLock()
        self.latest: Dict[str, Dict[str, Any]] = {}
        self.frames: List[Dict[str, Any]] = []
        self.recording = False
        self.stop_requested = False
        self.pending_start_time: Optional[float] = None
        self.episode_start_wall_time = 0.0
        self.episode_start_ros_time = 0.0
        self.last_warn_time = 0.0
        self.episode_counter = self._count_existing_episodes()
        self.speech = SpeechNotifier(
            self.get_logger(),
            enabled=not bool(getattr(args, "disable_speech", False)),
            command=str(getattr(args, "speech_command", "") or ""),
        )

        self.specs = build_topic_specs(args)
        for spec in self.specs:
            self.create_subscription(
                spec.msg_type,
                spec.topic,
                self._make_callback(spec),
                int(args.qos_depth),
            )

        self.timer = self.create_timer(1.0 / float(args.rate), self.sample_once)
        self.get_logger().info(f"Recorder ready: rate={args.rate} Hz, topics={len(self.specs)}")
        self.get_logger().info(f"Output dir: {args.output_dir}")
        self.get_logger().info("Keyboard: press 5 to start after delay, 6 to save episode, 7 to discard episode, q to save and quit")

    def _make_callback(self, spec: TopicSpec):
        def callback(msg: Any) -> None:
            try:
                data = spec.converter(msg)
            except Exception as exc:
                self.get_logger().warn(f"Failed to convert {spec.topic}: {exc}")
                return
            with self.latest_lock:
                self.latest[spec.key] = {
                    "topic": spec.topic,
                    "type": spec.msg_type.__name__,
                    "received_time": time.time(),
                    "data": data,
                }
        return callback

    def request_start(self) -> bool:
        with self.state_lock:
            if self.recording:
                self.get_logger().warn("Already recording. Press 6 to save or 7 to discard current episode first.")
                return False
            if self.pending_start_time is not None:
                self.get_logger().warn("Start already scheduled.")
                return False
            delay = max(0.0, float(self.args.start_delay))
            self.pending_start_time = time.time() + delay
            self.get_logger().info(f"Start requested. Recording will begin in {delay:.2f} s")
            return True

    def start_episode_now(self) -> None:
        with self.state_lock:
            self.frames = []
            self.recording = True
            self.pending_start_time = None
            self.episode_start_wall_time = time.time()
            self.episode_start_ros_time = stamp_to_float(self.get_clock().now().to_msg())
            self.last_warn_time = 0.0
            self.get_logger().info(f"Episode {self.episode_counter:04d} recording started")

    def finish_episode(self, reason: str = "keyboard") -> Optional[str]:
        with self.state_lock:
            self.pending_start_time = None
            if not self.recording and not self.frames:
                self.get_logger().warn("No active episode to save.")
                return None
            self.recording = False
            output_path = self._save_current_episode(reason=reason)
            self.frames = []
            self.episode_counter += 1
            return output_path

    def discard_episode(self, reason: str = "keyboard") -> Optional[str]:
        with self.state_lock:
            if self.pending_start_time is not None and not self.recording and not self.frames:
                self.pending_start_time = None
                self.get_logger().info("Canceled scheduled episode start.")
                return "canceled"
            self.pending_start_time = None
            if not self.recording and not self.frames:
                self.get_logger().warn("No active episode to discard.")
                return None

            episode_index = self.episode_counter
            frame_count = len(self.frames)
            self.recording = False
            self.frames = []
            self.get_logger().info(
                f"Discarded episode {episode_index:04d}: {frame_count} frames (reason={reason}); no file saved"
            )
            return "discarded"

    def speak(self, text: str) -> None:
        self.speech.say(text)

    def sample_once(self) -> None:
        with self.state_lock:
            now = time.time()
            if self.pending_start_time is not None and now >= self.pending_start_time:
                self.start_episode_now()

            if not self.recording:
                return

            if self.args.duration > 0 and (now - self.episode_start_wall_time) >= self.args.duration:
                self.get_logger().info("Reached duration for current episode")
                self.finish_episode(reason="duration")
                if self.args.oneshot:
                    self.stop_requested = True
                return

            with self.latest_lock:
                snapshot = {key: value.copy() for key, value in self.latest.items()}

            frame = {
                "sample_index": len(self.frames),
                "sample_time": now,
                "ros_time": stamp_to_float(self.get_clock().now().to_msg()),
                "left": {
                    "joint_state": snapshot.get("left_joint_state"),
                    "action_joint_control": snapshot.get("left_joint_control"),
                    "gripper_action": snapshot.get("left_gripper_ctrl"),
                    "gripper_state": snapshot.get("left_gripper_state"),
                    "end_pose": snapshot.get("left_end_pose"),
                    "end_force": snapshot.get("left_end_force"),
                },
                "right": {
                    "joint_state": snapshot.get("right_joint_state"),
                    "action_joint_control": snapshot.get("right_joint_control"),
                    "gripper_action": snapshot.get("right_gripper_ctrl"),
                    "gripper_state": snapshot.get("right_gripper_state"),
                    "end_pose": snapshot.get("right_end_pose"),
                    "end_force": snapshot.get("right_end_force"),
                },
                "cameras": {k: v for k, v in snapshot.items() if k.startswith("camera:")},
                "tactile_images": {k: v for k, v in snapshot.items() if k.startswith("tactile_image:")},
                "tactile_marker_offsets": {k: v for k, v in snapshot.items() if k.startswith("tactile_marker:")},
            }
            self.frames.append(frame)

            if now - self.last_warn_time > 5.0:
                missing = [spec.topic for spec in self.specs if spec.key not in snapshot]
                if missing:
                    self.get_logger().warn(
                        f"Waiting for {len(missing)} topics. Examples: {missing[:6]}"
                    )
                self.get_logger().info(f"Recording episode {self.episode_counter:04d}, frames={len(self.frames)}")
                self.last_warn_time = now

    def _save_current_episode(self, reason: str) -> str:
        os.makedirs(self.args.output_dir, exist_ok=True)
        output_path = self._output_path_for_current_episode()
        end_wall_time = time.time()
        payload = {
            "metadata": {
                "format": "marvin_dual_arm_teleop_record_v1",
                "created_time": end_wall_time,
                "sample_rate_hz": float(self.args.rate),
                "episode_index": int(self.episode_counter),
                "save_reason": reason,
                "start_time": self.episode_start_wall_time,
                "end_time": end_wall_time,
                "duration_s": end_wall_time - self.episode_start_wall_time,
                "start_ros_time": self.episode_start_ros_time,
                "frame_count": len(self.frames),
                "topics": [
                    {"key": spec.key, "topic": spec.topic, "type": spec.msg_type.__name__}
                    for spec in self.specs
                ],
                "image_note": "Image.data is saved as raw bytes. Project camera publishers usually store JPEG bytes in Image.data.",
            },
            "frames": self.frames,
        }
        with open(output_path, "wb") as f:
            pickle.dump(payload, f, protocol=pickle.HIGHEST_PROTOCOL)
        self.get_logger().info(f"Saved episode {self.episode_counter:04d}: {len(self.frames)} frames -> {output_path}")
        return output_path

    def _output_path_for_current_episode(self) -> str:
        if self.args.output_file:
            if not self.args.oneshot:
                self.get_logger().warn("--output-file is intended for --oneshot. Using --output-dir episode numbering instead.")
            else:
                output_parent = os.path.dirname(os.path.abspath(self.args.output_file))
                if output_parent:
                    os.makedirs(output_parent, exist_ok=True)
                return self.args.output_file
        return os.path.join(self.args.output_dir, f"episode_{self.episode_counter:04d}.pkl")

    def _count_existing_episodes(self) -> int:
        if not os.path.isdir(self.args.output_dir):
            return 0
        return len([
            f for f in os.listdir(self.args.output_dir)
            if f.startswith("episode_") and f.endswith(".pkl")
        ])


class KeyboardController:
    def __init__(self, node: PklTeleopRecorder) -> None:
        self.node = node
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.old_term_attrs = None

    def start(self) -> None:
        if not sys.stdin.isatty():
            self.node.get_logger().warn("stdin is not a TTY; keyboard 5/6/7 control is disabled.")
            return
        self.old_term_attrs = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.old_term_attrs is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self.old_term_attrs)
            self.old_term_attrs = None

    def _loop(self) -> None:
        while self.running and rclpy.ok() and not self.node.stop_requested:
            readable, _, _ = select.select([sys.stdin], [], [], 0.1)
            if not readable:
                continue
            ch = sys.stdin.read(1)
            if ch == "5":
                if self.node.request_start():
                    self.node.speak(f"recording number {self.node.episode_counter + 1}")
            elif ch == "6":
                if self.node.finish_episode(reason="keyboard"):
                    self.node.speak("saved")
                else:
                    self.node.speak("nothing saved")
            elif ch == "7":
                discard_result = self.node.discard_episode(reason="keyboard")
                if discard_result == "discarded":
                    self.node.speak("deleted")
                elif discard_result == "canceled":
                    self.node.speak("deleted")
                else:
                    self.node.speak("nothing collecting")
            elif ch in {"q", "Q"}:
                if self.node.recording or self.node.frames:
                    self.node.finish_episode(reason="quit")
                self.node.stop_requested = True
                return



def _short_topic_name(topic: str, max_len: int = 34) -> str:
    topic = str(topic or "")
    if len(topic) <= max_len:
        return topic
    return "..." + topic[-(max_len - 3):]


class LiveVisualizer:
    """Optional OpenCV live view for recorder topics.

    It only reads node.latest and does not change the recorded payload.
    """

    def __init__(self, node: PklTeleopRecorder) -> None:
        self.node = node
        self.running = False
        self.thread: Optional[threading.Thread] = None
        self.window_name = "marvin record live view"

    def start(self) -> None:
        try:
            import cv2  # noqa: F401
        except Exception as exc:
            self.node.get_logger().warn(
                f"--visualize requested but OpenCV cannot be imported: {exc}. "
                "Install with: pip install opencv-python"
            )
            return
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        self.node.get_logger().info("Live visualizer started. Focus the image window and press Esc to close only the window.")

    def stop(self) -> None:
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        try:
            import cv2
            cv2.destroyWindow(self.window_name)
        except Exception:
            pass

    def _decode_image(self, entry: Optional[Dict[str, Any]]) -> Optional[np.ndarray]:
        if not entry:
            return None
        data = entry.get("data", {})
        raw = data.get("data", b"")
        if not raw:
            return None

        import cv2

        # Most project camera publishers put JPEG bytes in sensor_msgs/Image.data.
        arr = np.frombuffer(raw, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is not None:
            return img

        # Fallback for standard raw sensor_msgs/Image encodings.
        h = int(data.get("height", 0) or 0)
        w = int(data.get("width", 0) or 0)
        enc = str(data.get("encoding", "")).lower()
        try:
            if h > 0 and w > 0:
                if enc in {"rgb8", "bgr8"} and len(raw) >= h * w * 3:
                    img = np.frombuffer(raw, dtype=np.uint8, count=h * w * 3).reshape(h, w, 3)
                    if enc == "rgb8":
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    return img.copy()
                if enc in {"mono8", "8uc1"} and len(raw) >= h * w:
                    img = np.frombuffer(raw, dtype=np.uint8, count=h * w).reshape(h, w)
                    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        except Exception:
            return None
        return None

    def _make_tile(self, key: str, entry: Optional[Dict[str, Any]], tile_w: int, tile_h: int) -> np.ndarray:
        import cv2

        img = self._decode_image(entry)
        if img is None:
            tile = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
            label = f"{_short_topic_name(key)} | no image"
        else:
            tile = cv2.resize(img, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
            age = time.time() - float(entry.get("received_time", time.time()))
            label = f"{_short_topic_name(entry.get('topic', key))} | {age:.2f}s"
        cv2.rectangle(tile, (0, 0), (tile_w, 24), (0, 0, 0), -1)
        cv2.putText(tile, label, (6, 17), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        return tile

    @staticmethod
    def _fmt_vec(entry: Optional[Dict[str, Any]], field: str = "position", n: int = 8) -> str:
        if not entry:
            return "None"
        data = entry.get("data", {})
        value = data.get(field, None)
        if value is None:
            return "None"
        arr = np.asarray(value).reshape(-1)
        arr = arr[:n]
        return np.array2string(arr, precision=3, suppress_small=True, separator=", ")

    def _make_status_panel(self, snapshot: Dict[str, Dict[str, Any]], width: int, height: int) -> np.ndarray:
        import cv2

        panel = np.zeros((height, width, 3), dtype=np.uint8)
        lines = [
            f"recording: {self.node.recording}    frames: {len(self.node.frames)}",
            f"episode: {self.node.episode_counter:04d}    topics received: {len(snapshot)}/{len(self.node.specs)}",
            "",
            "LEFT",
            f"  joint_state: {self._fmt_vec(snapshot.get('left_joint_state'))}",
            f"  gripper_state: {self._fmt_vec(snapshot.get('left_gripper_state'))}",
            f"  gripper_cmd: {self._fmt_vec(snapshot.get('left_gripper_ctrl'))}",
            "",
            "RIGHT",
            f"  joint_state: {self._fmt_vec(snapshot.get('right_joint_state'))}",
            f"  gripper_state: {self._fmt_vec(snapshot.get('right_gripper_state'))}",
            f"  gripper_cmd: {self._fmt_vec(snapshot.get('right_gripper_ctrl'))}",
            "",
            "Keyboard: terminal 5=start, 6=save, 7=discard, q=save+quit; window Esc=close visualizer",
        ]
        y = 22
        for line in lines:
            cv2.putText(panel, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
            y += 22
            if y > height - 8:
                break
        return panel

    def _loop(self) -> None:
        import cv2

        tile_w = int(self.node.args.vis_tile_width)
        tile_h = int(self.node.args.vis_tile_height)
        max_images = int(self.node.args.vis_max_images)
        vis_dt = 1.0 / max(1e-6, float(self.node.args.vis_rate))
        cols = 3

        while self.running and rclpy.ok() and not self.node.stop_requested:
            with self.node.latest_lock:
                snapshot = {key: value.copy() for key, value in self.node.latest.items()}

            image_items = [
                (k, v) for k, v in snapshot.items()
                if k.startswith("camera:") or k.startswith("tactile_image:")
            ][:max_images]
            tiles = [self._make_tile(k, v, tile_w, tile_h) for k, v in image_items]
            if not tiles:
                tiles = [self._make_tile("waiting for image topics", None, tile_w, tile_h)]

            rows = []
            for i in range(0, len(tiles), cols):
                row_tiles = tiles[i:i + cols]
                while len(row_tiles) < cols:
                    row_tiles.append(np.zeros((tile_h, tile_w, 3), dtype=np.uint8))
                rows.append(np.hstack(row_tiles))

            status = self._make_status_panel(snapshot, tile_w * cols, max(180, tile_h))
            canvas = np.vstack(rows + [status])
            cv2.imshow(self.window_name, canvas)
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # Esc closes only the visualizer window.
                self.running = False
                break
            time.sleep(vis_dt)


def _topic_for_name(name: Any, suffix: str) -> str:
    name = str(name or "").strip().strip("/")
    if not name:
        return ""
    return f"/{name}/{suffix}"


def _dedupe_topics(topics: Iterable[str]) -> List[str]:
    result: List[str] = []
    seen = set()
    for topic in topics:
        topic = str(topic).strip()
        if not topic or topic in seen:
            continue
        seen.add(topic)
        result.append(topic)
    return result


def _cfg_get(container: Any, key: str, default: Any = None) -> Any:
    if container is None:
        return default
    if isinstance(container, dict):
        return container.get(key, default)
    getter = getattr(container, "get", None)
    if getter is None:
        return default
    return getter(key, default)


def _cfg_list(container: Any, key: str) -> List[Any]:
    value = _cfg_get(container, key, [])
    return list(value or [])


def load_sensor_topics_from_config(config_file: str) -> Dict[str, List[str]]:
    defaults = {
        "rgb": DEFAULT_RGB_TOPICS.copy(),
        "tactile_rgb": DEFAULT_TACTILE_RGB_TOPICS.copy(),
        "tactile_marker": DEFAULT_TACTILE_MARKER_TOPICS.copy(),
    }
    if not config_file or not os.path.exists(config_file):
        return defaults

    try:
        try:
            import yaml

            with open(config_file, "r") as f:
                cfg = yaml.safe_load(f) or {}
        except ImportError:
            from omegaconf import OmegaConf

            cfg = OmegaConf.load(config_file)

        publisher_cfg = _cfg_get(cfg, "publisher", {})
        camera_cfgs = _cfg_list(publisher_cfg, "realsense_camera_publisher")
        tactile_cfgs = _cfg_list(publisher_cfg, "tactile_sensor_publisher")

        rgb_topics = [
            _topic_for_name(_cfg_get(cam, "camera_name"), "color/image_raw")
            for cam in camera_cfgs
        ]
        tactile_names = [
            _cfg_get(sensor, "sensor_name", _cfg_get(sensor, "camera_name"))
            for sensor in tactile_cfgs
        ]
        tactile_rgb_topics = [
            _topic_for_name(name, "color/image_raw")
            for name in tactile_names
        ]
        tactile_marker_topics = [
            _topic_for_name(name, "marker_offset/information")
            for name in tactile_names
        ]

        loaded = {
            "rgb": _dedupe_topics(rgb_topics),
            "tactile_rgb": _dedupe_topics(tactile_rgb_topics),
            "tactile_marker": _dedupe_topics(tactile_marker_topics),
        }
        return {key: value or defaults[key] for key, value in loaded.items()}
    except Exception as exc:
        print(f"Warning: failed to load sensor topics from {config_file}: {exc}", file=sys.stderr)
        return defaults


def _extend_topic_specs(specs: List[TopicSpec], prefix: str, topics: Iterable[str], msg_type: Any, converter: Callable[[Any], Dict[str, Any]]) -> None:
    for topic in topics:
        topic = topic.strip()
        if not topic:
            continue
        specs.append(TopicSpec(key=f"{prefix}:{topic}", topic=topic, msg_type=msg_type, converter=converter))


def build_topic_specs(args: argparse.Namespace) -> List[TopicSpec]:
    specs: List[TopicSpec] = [
        TopicSpec("left_joint_state", args.left_joint_state_topic, JointState, joint_state_to_dict),
        TopicSpec("right_joint_state", args.right_joint_state_topic, JointState, joint_state_to_dict),
        TopicSpec("left_joint_control", args.left_joint_control_topic, JointState, joint_state_to_dict),
        TopicSpec("right_joint_control", args.right_joint_control_topic, JointState, joint_state_to_dict),
        TopicSpec("left_gripper_ctrl", args.left_gripper_ctrl_topic, JointState, joint_state_to_dict),
        TopicSpec("right_gripper_ctrl", args.right_gripper_ctrl_topic, JointState, joint_state_to_dict),
        TopicSpec("left_gripper_state", args.left_gripper_state_topic, JointState, joint_state_to_dict),
        TopicSpec("right_gripper_state", args.right_gripper_state_topic, JointState, joint_state_to_dict),
        TopicSpec("left_end_pose", args.left_end_pose_topic, PoseStamped, pose_stamped_to_dict),
        TopicSpec("right_end_pose", args.right_end_pose_topic, PoseStamped, pose_stamped_to_dict),
        TopicSpec("left_end_force", args.left_end_force_topic, WrenchStamped, wrench_stamped_to_dict),
        TopicSpec("right_end_force", args.right_end_force_topic, WrenchStamped, wrench_stamped_to_dict),
    ]
    _extend_topic_specs(specs, "camera", args.rgb_topic, Image, image_to_dict)
    _extend_topic_specs(specs, "tactile_image", args.tactile_rgb_topic, Image, image_to_dict)
    _extend_topic_specs(specs, "tactile_marker", args.tactile_marker_topic, PointCloud2, pointcloud2_to_dict)
    return specs


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Record dual-arm teleop data into numbered pickle episodes.")
    parser.add_argument("--output-dir", default="/tmp/marvin_records", help="Directory for episode_xxxx.pkl files.")
    parser.add_argument("--output-file", default="", help="Explicit output .pkl path for --oneshot mode.")
    parser.add_argument("--config-file", default=DEFAULT_CONFIG_FILE, help="YAML config used to derive default camera and tactile topics.")
    parser.add_argument("--rate", type=float, default=30.0, help="Snapshot rate in Hz while recording.")
    parser.add_argument("--duration", type=float, default=0.0, help="Auto-save each episode after N seconds. 0 disables auto-save.")
    parser.add_argument("--start-delay", type=float, default=0.5, help="Delay after pressing 5 before recording starts.")
    parser.add_argument("--qos-depth", type=int, default=20)
    parser.add_argument("--record-immediately", action="store_true", help="Start recording immediately instead of waiting for key 5.")
    parser.add_argument("--oneshot", action="store_true", help="Exit after one saved episode, useful with --record-immediately and --duration.")
    parser.add_argument("--disable-keyboard", action="store_true", help="Disable terminal 5/6/7/q keyboard control.")
    parser.add_argument("--disable-speech", action="store_true", help="Disable voice announcements for keyboard 5/6/7 actions.")
    parser.add_argument(
        "--speech-command",
        default="",
        help='Custom TTS command for voice announcements. Text is appended unless the command contains "{text}".',
    )
    parser.add_argument("--visualize", action="store_true", help="Open an OpenCV live window for camera/tactile images and robot/gripper state.")
    parser.add_argument("--vis-rate", type=float, default=10.0, help="Live visualization refresh rate in Hz.")
    parser.add_argument("--vis-tile-width", type=int, default=320, help="Live visualization image tile width.")
    parser.add_argument("--vis-tile-height", type=int, default=240, help="Live visualization image tile height.")
    parser.add_argument("--vis-max-images", type=int, default=8, help="Maximum camera/tactile image tiles shown in the live window.")

    parser.add_argument("--left-joint-state-topic", default="/left_joint_state")
    parser.add_argument("--right-joint-state-topic", default="/right_joint_state")
    parser.add_argument("--left-joint-control-topic", default="/left_joint_control")
    parser.add_argument("--right-joint-control-topic", default="/right_joint_control")
    parser.add_argument("--left-gripper-ctrl-topic", default="/left_teleop_gripper/ctrl")
    parser.add_argument("--right-gripper-ctrl-topic", default="/right_teleop_gripper/ctrl")
    parser.add_argument("--left-gripper-state-topic", default="/left_gripper/state")
    parser.add_argument("--right-gripper-state-topic", default="/right_gripper/state")
    parser.add_argument("--left-end-pose-topic", default="/left_end_pose")
    parser.add_argument("--right-end-pose-topic", default="/right_end_pose")
    parser.add_argument("--left-end-force-topic", default="/left_end_force")
    parser.add_argument("--right-end-force-topic", default="/right_end_force")

    parser.add_argument(
        "--rgb-topic",
        action="append",
        default=None,
        help="RGB image topic. Can be repeated. If omitted, derived from --config-file.",
    )
    parser.add_argument(
        "--tactile-rgb-topic",
        action="append",
        default=None,
        help="Tactile RGB image topic. Can be repeated. If omitted, derived from --config-file.",
    )
    parser.add_argument(
        "--tactile-marker-topic",
        action="append",
        default=None,
        help="Tactile marker PointCloud2 topic. Can be repeated. If omitted, derived from --config-file.",
    )
    args = parser.parse_args(argv)
    sensor_topics = load_sensor_topics_from_config(args.config_file)
    if args.rgb_topic is None:
        args.rgb_topic = sensor_topics["rgb"]
    if args.tactile_rgb_topic is None:
        args.tactile_rgb_topic = sensor_topics["tactile_rgb"]
    if args.tactile_marker_topic is None:
        args.tactile_marker_topic = sensor_topics["tactile_marker"]
    return args


def main(argv: Optional[List[str]] = None) -> None:
    args = parse_args(argv)
    if args.rate <= 0:
        raise ValueError("--rate must be > 0")

    rclpy.init(args=None)
    node = PklTeleopRecorder(args)
    visualizer = LiveVisualizer(node)
    if args.visualize:
        visualizer.start()
    keyboard = KeyboardController(node)
    if not args.disable_keyboard:
        keyboard.start()
    if args.record_immediately:
        node.start_episode_now()

    def handle_signal(signum, frame):
        node.get_logger().info(f"Received signal {signum}, stopping recorder")
        if node.recording or node.frames:
            node.finish_episode(reason="signal")
        node.stop_requested = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        visualizer.stop()
        keyboard.stop()
        if node.recording or node.frames:
            node.finish_episode(reason="shutdown")
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main(sys.argv[1:])
