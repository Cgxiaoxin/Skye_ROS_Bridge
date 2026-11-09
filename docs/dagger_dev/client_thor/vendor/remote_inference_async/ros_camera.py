"""ROS 2 RealSense Image bridge for asynchronous remote inference."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

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
    from sensor_msgs.msg import Image
except ImportError as exc:
    raise ImportError(
        "ROS 2 image packages are unavailable. Run "
        "'source /opt/kernelmind/apex/install/setup.bash' first."
    ) from exc


@dataclass(frozen=True, slots=True)
class JpegFrame:
    data: bytes
    width: int
    height: int
    source_timestamp_ns: int
    received_timestamp_ns: int


def _raw_image_to_bgr(msg: Image) -> np.ndarray:
    """Convert the raw encodings used by the RealSense bridge to BGR8."""

    width = int(msg.width)
    height = int(msg.height)
    if width <= 0 or height <= 0:
        raise ValueError(f"invalid raw image dimensions: {width}x{height}")

    encoding = str(msg.encoding).strip().lower()
    formats: dict[str, tuple[int, int | None]] = {
        "bgr8": (3, None),
        "rgb8": (3, cv2.COLOR_RGB2BGR),
        "bgra8": (4, cv2.COLOR_BGRA2BGR),
        "rgba8": (4, cv2.COLOR_RGBA2BGR),
        "mono8": (1, cv2.COLOR_GRAY2BGR),
        "yuv422_yuy2": (2, cv2.COLOR_YUV2BGR_YUY2),
        "yuy2": (2, cv2.COLOR_YUV2BGR_YUY2),
        "yuyv": (2, cv2.COLOR_YUV2BGR_YUY2),
    }
    if encoding not in formats:
        raise ValueError(
            f"unsupported raw ROS image encoding {msg.encoding!r}; expected one of "
            f"{sorted(formats)}"
        )

    channels, conversion = formats[encoding]
    packed_row_bytes = width * channels
    step = int(msg.step) if int(msg.step) > 0 else packed_row_bytes
    if step < packed_row_bytes:
        raise ValueError(
            f"raw image step {step} is smaller than packed row size {packed_row_bytes}"
        )
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    required_bytes = step * height
    if raw.size < required_bytes:
        raise ValueError(
            f"raw image payload is truncated: got {raw.size} bytes, need {required_bytes}"
        )
    packed = raw[:required_bytes].reshape(height, step)[:, :packed_row_bytes]
    if channels == 1:
        image = packed.reshape(height, width)
    else:
        image = packed.reshape(height, width, channels)
    if conversion is None:
        return image
    return cv2.cvtColor(image, conversion)


def _camera_qos(reliability: str, depth: int) -> QoSProfile:
    if depth <= 0:
        raise ValueError(f"ROS camera QoS depth must be positive, got {depth}")
    reliability_policy = {
        "best_effort": ReliabilityPolicy.BEST_EFFORT,
        "reliable": ReliabilityPolicy.RELIABLE,
    }.get(reliability)
    if reliability_policy is None:
        raise ValueError(
            "ROS camera reliability must be best_effort or reliable, "
            f"got {reliability!r}"
        )
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=reliability_policy,
        durability=DurabilityPolicy.VOLATILE,
    )


class _RosCameraNode(Node):
    def __init__(
        self,
        topics: dict[str, str],
        *,
        reliability: str,
        qos_depth: int,
        output_width: int | None,
        output_height: int | None,
        jpeg_quality: int,
    ) -> None:
        super().__init__("gento_remote_inference_camera_bridge")
        if (output_width is None) != (output_height is None):
            raise ValueError("output_width and output_height must both be set or both be None")
        if output_width is not None and (output_width <= 0 or output_height <= 0):
            raise ValueError(
                f"output dimensions must be positive, got {output_width}x{output_height}"
            )
        if not 1 <= jpeg_quality <= 100:
            raise ValueError(f"jpeg_quality must be in [1, 100], got {jpeg_quality}")
        self._output_width = None if output_width is None else int(output_width)
        self._output_height = None if output_height is None else int(output_height)
        self._jpeg_quality = int(jpeg_quality)
        self._condition = threading.Condition()
        self._frames: dict[str, JpegFrame] = {}
        self._errors: dict[str, str] = {}
        qos = _camera_qos(reliability, qos_depth)
        for name, topic in topics.items():
            # Node.create_subscription() already retains the subscription in
            # Node._subscriptions. Appending it again makes destroy_node() try
            # to destroy the same rclpy subscription twice.
            self.create_subscription(
                Image,
                topic,
                lambda msg, camera_name=name: self._receive(camera_name, msg),
                qos,
            )

    @staticmethod
    def _source_timestamp_ns(msg: Image) -> int:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def _store_frame(
        self,
        name: str,
        *,
        data: bytes,
        width: int,
        height: int,
        source_timestamp_ns: int,
        received_timestamp_ns: int,
    ) -> None:
        frame = JpegFrame(
            data=data,
            width=width,
            height=height,
            source_timestamp_ns=source_timestamp_ns,
            received_timestamp_ns=received_timestamp_ns,
        )
        with self._condition:
            self._frames[name] = frame
            self._errors.pop(name, None)
            self._condition.notify_all()

    def _record_error(self, name: str, exc: Exception) -> None:
        with self._condition:
            self._errors[name] = str(exc)
            self._condition.notify_all()

    def _resize_and_encode(self, image: np.ndarray) -> tuple[bytes, int, int]:
        height, width = image.shape[:2]
        if self._output_width is not None and (
            width != self._output_width or height != self._output_height
        ):
            interpolation = (
                cv2.INTER_AREA
                if self._output_width <= width and self._output_height <= height
                else cv2.INTER_LINEAR
            )
            image = cv2.resize(
                image,
                (self._output_width, self._output_height),
                interpolation=interpolation,
            )
            width = self._output_width
            height = self._output_height
        encoded_ok, encoded = cv2.imencode(
            ".jpg",
            image,
            [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality],
        )
        if not encoded_ok:
            raise ValueError("OpenCV failed to encode ROS image as JPEG")
        return encoded.tobytes(), width, height

    def _receive(self, name: str, msg: Image) -> None:
        received_ns = time.monotonic_ns()
        try:
            image = _raw_image_to_bgr(msg)
            data, width, height = self._resize_and_encode(image)
            self._store_frame(
                name,
                data=data,
                width=width,
                height=height,
                source_timestamp_ns=self._source_timestamp_ns(msg),
                received_timestamp_ns=received_ns,
            )
        except Exception as exc:
            self._record_error(name, exc)

    def wait_for_frames(
        self,
        names: list[str],
        *,
        timeout_s: float,
        max_age_s: float,
        sync_tolerance_s: float,
    ) -> list[tuple[str, bytes, int, int]]:
        deadline = time.monotonic() + timeout_s
        last_problem = "waiting for first frames"
        with self._condition:
            while True:
                if self._errors:
                    errors = ", ".join(f"{name}: {message}" for name, message in self._errors.items())
                    raise RuntimeError(f"ROS camera decode error: {errors}")
                missing = [name for name in names if name not in self._frames]
                if not missing:
                    selected = [self._frames[name] for name in names]
                    now_ns = time.monotonic_ns()
                    ages = [(now_ns - frame.received_timestamp_ns) / 1e9 for frame in selected]
                    if max(ages) <= max_age_s:
                        # D435 and GMSL headers can come from different hardware
                        # clock domains with a stable offset. Compare arrival
                        # times measured by this process's monotonic clock,
                        # while max_age_s above still rejects stalled streams.
                        receive_stamps = [frame.received_timestamp_ns for frame in selected]
                        sync_span_s = (max(receive_stamps) - min(receive_stamps)) / 1e9
                        if sync_span_s <= sync_tolerance_s:
                            return [
                                (name, frame.data, frame.width, frame.height)
                                for name, frame in zip(names, selected, strict=True)
                            ]
                        last_problem = (
                            f"camera receive-time span {sync_span_s:.3f}s exceeds "
                            f"{sync_tolerance_s:.3f}s"
                        )
                    else:
                        last_problem = f"stale camera frames, ages={dict(zip(names, ages, strict=True))}"
                else:
                    last_problem = f"missing camera frames: {missing}"

                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"ROS camera timeout: {last_problem}")
                self._condition.wait(timeout=min(remaining, 0.05))


class RosCameraBridge:
    """Cache raw RealSense frames and expose synchronized JPEG payloads."""

    def __init__(
        self,
        topics: dict[str, str],
        *,
        reliability: str = "reliable",
        qos_depth: int = 10,
        output_width: int | None = None,
        output_height: int | None = None,
        jpeg_quality: int = 85,
    ) -> None:
        if not topics:
            raise ValueError("at least one ROS camera topic is required")
        if len(topics) != len(set(topics.values())):
            raise ValueError(f"ROS camera topics must be unique: {topics}")
        if not rclpy.ok():
            rclpy.init(args=None)
        self.names = list(topics)
        self.node = _RosCameraNode(
            topics,
            reliability=reliability,
            qos_depth=qos_depth,
            output_width=output_width,
            output_height=output_height,
            jpeg_quality=jpeg_quality,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self.executor.spin, name="ros-cameras", daemon=True)
        self.thread.start()
        self._closed = False

    def read_jpegs(
        self,
        *,
        timeout_s: float,
        max_age_s: float,
        sync_tolerance_s: float,
    ) -> list[tuple[str, bytes, int, int]]:
        return self.node.wait_for_frames(
            self.names,
            timeout_s=timeout_s,
            max_age_s=max_age_s,
            sync_tolerance_s=sync_tolerance_s,
        )

    def close(self) -> None:
        if self._closed:
            return
        self._closed = True
        self.executor.shutdown()
        self.thread.join(timeout=2.0)
        self.node.destroy_node()
        # RosGripperBridge owns the shared rclpy process lifetime and shuts it down.
