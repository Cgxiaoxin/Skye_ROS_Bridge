"""Tianji protocol-v5 helpers for history and RTC action-prefix transport.

The binary state/image/action layout is unchanged. Protocol v5 adds an optional
committed-action prefix frame used only when RTC is enabled.
"""

from __future__ import annotations

import json
from dataclasses import dataclass
from typing import Any

import numpy as np

PROTOCOL_VERSION = 5
STATE_NAMES = (
    [f"left_arm_joint_{i}" for i in range(7)]
    + [f"right_arm_joint_{i}" for i in range(7)]
    + ["left_gripper", "right_gripper"]
)
ACTION_NAMES = list(STATE_NAMES)
STATE_DIM = len(STATE_NAMES)
ACTION_DIM = len(ACTION_NAMES)


@dataclass(slots=True)
class ObservationPacket:
    route: list[bytes]
    request_id: str
    timestep: int
    timestamp_ns: int
    must_go: bool
    observation: dict[str, np.ndarray | str]


@dataclass(slots=True)
class ActionPacket:
    route: list[bytes]
    request_id: str
    observation_timestep: int
    observation_timestamp_ns: int
    actions: np.ndarray | None = None
    inference_time_ms: float = 0.0
    error: str | None = None
    dropped: bool = False


def json_bytes(value: dict[str, Any]) -> bytes:
    return json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")


def parse_json(data: bytes) -> dict[str, Any]:
    value = json.loads(data.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON frame must contain an object")
    return value


def float32_bytes(value: Any, *, expected_last_dim: int) -> bytes:
    array = np.asarray(value, dtype="<f4")
    if array.ndim == 0 or array.shape[-1] != expected_last_dim:
        raise ValueError(f"expected last dimension {expected_last_dim}, got {array.shape}")
    if not np.isfinite(array).all():
        raise ValueError("array contains NaN or infinity")
    return array.tobytes(order="C")


def parse_float32(
    data: bytes,
    shape: list[int] | tuple[int, ...],
    *,
    expected_last_dim: int,
) -> np.ndarray:
    dims = tuple(int(v) for v in shape)
    if not dims or dims[-1] != expected_last_dim or any(v <= 0 for v in dims):
        raise ValueError(f"invalid array shape: {dims}")
    array = np.frombuffer(data, dtype="<f4")
    expected_size = int(np.prod(dims))
    if array.size != expected_size:
        raise ValueError(f"payload has {array.size} floats, shape requires {expected_size}")
    return array.reshape(dims).copy()


def split_action(action: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    action = np.asarray(action, dtype=np.float32)
    if action.shape != (ACTION_DIM,):
        raise ValueError(f"action must have shape ({ACTION_DIM},), got {action.shape}")
    return action[:7], action[7:14], action[14:16]


def jpeg_dimensions(data: bytes) -> tuple[int, int]:
    """Read JPEG dimensions from a SOF marker without decoding the image."""
    if len(data) < 4 or data[:2] != b"\xff\xd8":
        raise ValueError("image payload is not a JPEG")
    offset = 2
    sof_markers = {
        0xC0,
        0xC1,
        0xC2,
        0xC3,
        0xC5,
        0xC6,
        0xC7,
        0xC9,
        0xCA,
        0xCB,
        0xCD,
        0xCE,
        0xCF,
    }
    while offset + 3 < len(data):
        if data[offset] != 0xFF:
            offset += 1
            continue
        while offset < len(data) and data[offset] == 0xFF:
            offset += 1
        if offset >= len(data):
            break
        marker = data[offset]
        offset += 1
        if marker in {0x01, *range(0xD0, 0xDA)}:
            continue
        if offset + 2 > len(data):
            break
        segment_length = int.from_bytes(data[offset : offset + 2], "big")
        if segment_length < 2 or offset + segment_length > len(data):
            break
        if marker in sof_markers:
            if segment_length < 7:
                break
            height = int.from_bytes(data[offset + 3 : offset + 5], "big")
            width = int.from_bytes(data[offset + 5 : offset + 7], "big")
            if width <= 0 or height <= 0:
                break
            return width, height
        offset += segment_length
    raise ValueError("cannot find JPEG dimensions in image payload")
