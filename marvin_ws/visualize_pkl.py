#!/usr/bin/env python3
"""Visualize Marvin recorder pkl files.

Outputs:
- one video per image topic
- one tiled video containing all image topics
- PNG time-series plots for robot states and tactile marker summaries

Run with the conda environment that can load the recorder pickle, for example:
  /home/wsj/miniconda3/envs/vitacsdk/bin/python data_convert/visualize_pkl.py \
      --input /home/wsj/marvin_records/episode_0000.pkl
"""
from __future__ import annotations

import argparse
import json
import math
import pickle
import re
from collections import OrderedDict, defaultdict
from pathlib import Path
from typing import Any, Dict, Iterable, List, Mapping, Optional, Tuple

import cv2
import numpy as np

IMAGE_GROUPS = ("cameras", "tactile_images")
ROBOT_SIDES = ("left", "right")
ROBOT_FIELDS = (
    "joint_state",
    "action_joint_control",
    "gripper_action",
    "gripper_state",
    "end_pose",
    "end_force",
)
PLOT_COLORS = [
    (31, 119, 180),
    (255, 127, 14),
    (44, 160, 44),
    (214, 39, 40),
    (148, 103, 189),
    (140, 86, 75),
    (227, 119, 194),
    (127, 127, 127),
    (188, 189, 34),
    (23, 190, 207),
]


def load_pkl(path: Path) -> Dict[str, Any]:
    with path.open("rb") as f:
        return pickle.load(f)


def safe_name(text: str, max_len: int = 120) -> str:
    text = str(text).strip()
    text = re.sub(r"^[a-zA-Z_]+:", "", text)
    text = text.strip("/") or "root"
    text = re.sub(r"[^0-9A-Za-z._-]+", "_", text.replace("/", "_"))
    return text[:max_len].strip("._-") or "unnamed"


def finite_or_nan(value: Any) -> float:
    try:
        out = float(value)
    except Exception:
        return float("nan")
    return out if np.isfinite(out) else float("nan")


def relative_times(frames: List[Dict[str, Any]]) -> np.ndarray:
    if not frames:
        return np.zeros((0,), dtype=np.float64)
    sample_times = np.asarray([finite_or_nan(fr.get("sample_time")) for fr in frames], dtype=np.float64)
    if np.isfinite(sample_times).any():
        start = sample_times[np.isfinite(sample_times)][0]
        return sample_times - start
    return np.arange(len(frames), dtype=np.float64)


def iter_frame_indices(total: int, stride: int, max_frames: int) -> Iterable[int]:
    emitted = 0
    for idx in range(0, total, max(1, stride)):
        if max_frames > 0 and emitted >= max_frames:
            break
        emitted += 1
        yield idx


def image_streams(frames: List[Dict[str, Any]]) -> "OrderedDict[Tuple[str, str], str]":
    streams: "OrderedDict[Tuple[str, str], str]" = OrderedDict()
    for frame in frames:
        for group in IMAGE_GROUPS:
            for key, item in (frame.get(group) or {}).items():
                if item is None:
                    continue
                topic = str(item.get("topic") or key)
                streams.setdefault((group, key), topic)
    return streams


def image_record_for(frame: Mapping[str, Any], group: str, key: str) -> Optional[Dict[str, Any]]:
    item = (frame.get(group) or {}).get(key)
    return item if isinstance(item, dict) else None


def bytes_to_uint8(data: Any) -> np.ndarray:
    if data is None:
        return np.zeros((0,), dtype=np.uint8)
    if isinstance(data, np.ndarray):
        return data.astype(np.uint8, copy=False).reshape(-1)
    return np.frombuffer(bytes(data), dtype=np.uint8)


def decode_image(record: Optional[Mapping[str, Any]]) -> Optional[np.ndarray]:
    if not record:
        return None
    data = record.get("data") or {}
    raw = bytes_to_uint8(data.get("data"))
    if raw.size == 0:
        return None

    image = cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if image is not None:
        return image

    height = int(data.get("height") or 0)
    width = int(data.get("width") or 0)
    encoding = str(data.get("encoding") or "").lower()
    if height <= 0 or width <= 0:
        return None

    channels = 1 if encoding in {"mono8", "8uc1"} else 3
    expected = height * width * channels
    if raw.size < expected:
        return None
    image = raw[:expected].reshape(height, width, channels)
    if channels == 1:
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    elif encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    return image


def draw_label(image: np.ndarray, label: str, frame_index: int) -> np.ndarray:
    out = image.copy()
    text = f"{label} | frame {frame_index}"
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.55
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    cv2.rectangle(out, (8, 8), (min(out.shape[1] - 1, tw + 22), th + baseline + 18), (0, 0, 0), -1)
    cv2.putText(out, text, (16, th + 12), font, scale, (255, 255, 255), thickness, cv2.LINE_AA)
    return out


def make_writer(path: Path, fps: float, size: Tuple[int, int], codec: str) -> cv2.VideoWriter:
    path.parent.mkdir(parents=True, exist_ok=True)
    fourcc = cv2.VideoWriter_fourcc(*codec)
    writer = cv2.VideoWriter(str(path), fourcc, float(fps), size)
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open VideoWriter for {path} with codec {codec}")
    return writer


def write_topic_video(
    frames: List[Dict[str, Any]],
    group: str,
    key: str,
    topic: str,
    output_dir: Path,
    fps: float,
    codec: str,
    stride: int,
    max_frames: int,
    overlay: bool,
) -> Optional[Path]:
    writer: Optional[cv2.VideoWriter] = None
    output_path = output_dir / f"{safe_name(topic)}.mp4"
    frame_size: Optional[Tuple[int, int]] = None
    written = 0

    for frame_index in iter_frame_indices(len(frames), stride, max_frames):
        image = decode_image(image_record_for(frames[frame_index], group, key))
        if image is None:
            continue
        if overlay:
            image = draw_label(image, topic, frame_index)
        if frame_size is None:
            frame_size = (int(image.shape[1]), int(image.shape[0]))
            writer = make_writer(output_path, fps, frame_size, codec)
        elif (image.shape[1], image.shape[0]) != frame_size:
            image = cv2.resize(image, frame_size, interpolation=cv2.INTER_AREA)
        writer.write(image)
        written += 1

    if writer is not None:
        writer.release()
    return output_path if written else None


def resize_to_tile(image: Optional[np.ndarray], tile_size: Tuple[int, int], label: str, frame_index: int) -> np.ndarray:
    tile_w, tile_h = tile_size
    if image is None:
        tile = np.zeros((tile_h, tile_w, 3), dtype=np.uint8)
    else:
        tile = cv2.resize(image, (tile_w, tile_h), interpolation=cv2.INTER_AREA)
    return draw_label(tile, label, frame_index)


def write_tiled_video(
    frames: List[Dict[str, Any]],
    streams: "OrderedDict[Tuple[str, str], str]",
    output_path: Path,
    fps: float,
    codec: str,
    stride: int,
    max_frames: int,
    tile_width: int,
    tile_height: int,
) -> Optional[Path]:
    if not streams:
        return None
    stream_items = list(streams.items())
    cols = int(math.ceil(math.sqrt(len(stream_items))))
    rows = int(math.ceil(len(stream_items) / cols))
    tile_size = (int(tile_width), int(tile_height))
    canvas_size = (cols * tile_size[0], rows * tile_size[1])
    writer = make_writer(output_path, fps, canvas_size, codec)
    written = 0

    for frame_index in iter_frame_indices(len(frames), stride, max_frames):
        tiles: List[np.ndarray] = []
        for (group, key), topic in stream_items:
            image = decode_image(image_record_for(frames[frame_index], group, key))
            tiles.append(resize_to_tile(image, tile_size, topic, frame_index))
        while len(tiles) < rows * cols:
            tiles.append(np.zeros((tile_size[1], tile_size[0], 3), dtype=np.uint8))
        row_images = [np.hstack(tiles[r * cols:(r + 1) * cols]) for r in range(rows)]
        writer.write(np.vstack(row_images))
        written += 1

    writer.release()
    return output_path if written else None


def flatten_vector(value: Any) -> Optional[np.ndarray]:
    if value is None:
        return None
    try:
        arr = np.asarray(value, dtype=np.float64).reshape(-1)
    except Exception:
        return None
    return arr if arr.size else None


def vector_components(item: Mapping[str, Any]) -> List[Tuple[str, np.ndarray, List[str]]]:
    data = item.get("data") or {}
    msg_type = str(item.get("type") or "")
    out: List[Tuple[str, np.ndarray, List[str]]] = []

    is_joint_state = "JointState" in msg_type or "name" in data
    if is_joint_state:
        joint_names = [str(x) for x in data.get("name", [])]
        for name in ("position", "velocity", "effort"):
            arr = flatten_vector(data.get(name))
            if arr is not None:
                labels = joint_names if len(joint_names) == arr.size else [f"{name}_{i}" for i in range(arr.size)]
                out.append((name, arr, labels))

    pose_parts = {}
    if "PoseStamped" in msg_type or "orientation_xyzw" in data:
        pose_parts.update({
            "position_xyz": data.get("position"),
            "orientation_xyzw": data.get("orientation_xyzw"),
        })
    if "WrenchStamped" in msg_type or "force" in data or "torque" in data:
        pose_parts.update({
            "force_xyz": data.get("force"),
            "torque_xyz": data.get("torque"),
        })
    fixed_labels = {
        "position_xyz": ["x", "y", "z"],
        "orientation_xyzw": ["qx", "qy", "qz", "qw"],
        "force_xyz": ["fx", "fy", "fz"],
        "torque_xyz": ["tx", "ty", "tz"],
    }
    for name, value in pose_parts.items():
        arr = flatten_vector(value)
        if arr is not None:
            labels = fixed_labels.get(name, [f"{name}_{i}" for i in range(arr.size)])
            if len(labels) != arr.size:
                labels = [f"{name}_{i}" for i in range(arr.size)]
            out.append((name, arr, labels))
    return out


def collect_robot_series(frames: List[Dict[str, Any]]) -> Dict[str, Tuple[np.ndarray, List[str]]]:
    values: Dict[str, Dict[int, np.ndarray]] = defaultdict(dict)
    labels: Dict[str, List[str]] = {}
    for frame_index, frame in enumerate(frames):
        for side in ROBOT_SIDES:
            side_data = frame.get(side) or {}
            for field in ROBOT_FIELDS:
                item = side_data.get(field)
                if not isinstance(item, dict):
                    continue
                for component, arr, component_labels in vector_components(item):
                    key = f"{side}.{field}.{component}"
                    values[key][frame_index] = arr
                    labels.setdefault(key, component_labels)

    series: Dict[str, Tuple[np.ndarray, List[str]]] = OrderedDict()
    for key in sorted(values):
        dim = max(arr.size for arr in values[key].values())
        mat = np.full((len(frames), dim), np.nan, dtype=np.float64)
        for idx, arr in values[key].items():
            mat[idx, :min(dim, arr.size)] = arr[:dim]
        if np.isfinite(mat).any():
            label_list = labels.get(key, [])
            if len(label_list) != dim:
                label_list = [f"dim_{i}" for i in range(dim)]
            series[key] = (mat, label_list)
    return series


def collect_marker_summaries(frames: List[Dict[str, Any]]) -> Dict[str, Tuple[np.ndarray, List[str]]]:
    values: Dict[str, Dict[int, np.ndarray]] = defaultdict(dict)
    labels = ["mean_dx", "mean_dy", "mean_dz", "std_dx", "std_dy", "std_dz", "mean_norm", "max_norm"]

    for frame_index, frame in enumerate(frames):
        for key, item in (frame.get("tactile_marker_offsets") or {}).items():
            if not isinstance(item, dict):
                continue
            topic = str(item.get("topic") or key)
            points = (item.get("data") or {}).get("points") or {}
            offsets = []
            for name in ("marker_offset_x", "marker_offset_y", "marker_offset_z"):
                arr = flatten_vector(points.get(name))
                if arr is None:
                    arr = np.full((0,), np.nan, dtype=np.float64)
                offsets.append(arr)
            if not offsets or max(arr.size for arr in offsets) == 0:
                continue
            dim = max(arr.size for arr in offsets)
            padded = np.full((dim, 3), np.nan, dtype=np.float64)
            for col, arr in enumerate(offsets):
                padded[:arr.size, col] = arr
            norm = np.linalg.norm(np.nan_to_num(padded, nan=0.0), axis=1)
            summary = np.asarray([
                np.nanmean(padded[:, 0]),
                np.nanmean(padded[:, 1]),
                np.nanmean(padded[:, 2]),
                np.nanstd(padded[:, 0]),
                np.nanstd(padded[:, 1]),
                np.nanstd(padded[:, 2]),
                np.nanmean(norm),
                np.nanmax(norm),
            ], dtype=np.float64)
            values[f"tactile_marker.{safe_name(topic)}.summary"][frame_index] = summary

    series: Dict[str, Tuple[np.ndarray, List[str]]] = OrderedDict()
    for key in sorted(values):
        mat = np.full((len(frames), len(labels)), np.nan, dtype=np.float64)
        for idx, arr in values[key].items():
            mat[idx, :arr.size] = arr
        if np.isfinite(mat).any():
            series[key] = (mat, labels)
    return series


def draw_timeseries_png(
    output_path: Path,
    times: np.ndarray,
    values: np.ndarray,
    labels: List[str],
    title: str,
    width: int = 1600,
    height: int = 900,
) -> bool:
    if values.size == 0 or not np.isfinite(values).any():
        return False

    canvas = np.full((height, width, 3), 255, dtype=np.uint8)
    left, right, top, bottom = 110, 270, 85, 95
    plot_x0, plot_y0 = left, top
    plot_x1, plot_y1 = width - right, height - bottom

    finite_y = values[np.isfinite(values)]
    y_min, y_max = float(np.min(finite_y)), float(np.max(finite_y))
    if y_min == y_max:
        pad = max(1.0, abs(y_min) * 0.1)
        y_min -= pad
        y_max += pad
    else:
        pad = (y_max - y_min) * 0.08
        y_min -= pad
        y_max += pad

    finite_t = times[np.isfinite(times)]
    if finite_t.size == 0:
        finite_t = np.arange(values.shape[0], dtype=np.float64)
    x_min, x_max = float(finite_t[0]), float(finite_t[-1])
    if x_min == x_max:
        x_max = x_min + 1.0

    def xy(t: float, y: float) -> Tuple[int, int]:
        x_pix = plot_x0 + int(round((t - x_min) / (x_max - x_min) * (plot_x1 - plot_x0)))
        y_pix = plot_y1 - int(round((y - y_min) / (y_max - y_min) * (plot_y1 - plot_y0)))
        return x_pix, y_pix

    # Grid and axes.
    for i in range(6):
        alpha = i / 5.0
        x = int(plot_x0 + alpha * (plot_x1 - plot_x0))
        y = int(plot_y1 - alpha * (plot_y1 - plot_y0))
        cv2.line(canvas, (x, plot_y0), (x, plot_y1), (230, 230, 230), 1)
        cv2.line(canvas, (plot_x0, y), (plot_x1, y), (230, 230, 230), 1)
        cv2.putText(canvas, f"{x_min + alpha * (x_max - x_min):.1f}", (x - 25, plot_y1 + 32), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1, cv2.LINE_AA)
        cv2.putText(canvas, f"{y_min + alpha * (y_max - y_min):.3g}", (10, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (80, 80, 80), 1, cv2.LINE_AA)
    cv2.rectangle(canvas, (plot_x0, plot_y0), (plot_x1, plot_y1), (20, 20, 20), 1)
    cv2.putText(canvas, title[:120], (left, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (20, 20, 20), 2, cv2.LINE_AA)
    cv2.putText(canvas, "time (s)", ((plot_x0 + plot_x1) // 2 - 45, height - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (50, 50, 50), 1, cv2.LINE_AA)

    draw_step = max(1, values.shape[0] // 2500)
    for dim in range(values.shape[1]):
        color = PLOT_COLORS[dim % len(PLOT_COLORS)]
        last: Optional[Tuple[int, int]] = None
        for idx in range(0, values.shape[0], draw_step):
            t = times[idx] if idx < times.size else float(idx)
            y = values[idx, dim]
            if not np.isfinite(t) or not np.isfinite(y):
                last = None
                continue
            point = xy(float(t), float(y))
            if last is not None:
                cv2.line(canvas, last, point, color, 2, cv2.LINE_AA)
            last = point

    legend_x = plot_x1 + 25
    legend_y = plot_y0 + 5
    for dim, label in enumerate(labels[:32]):
        y = legend_y + dim * 24
        color = PLOT_COLORS[dim % len(PLOT_COLORS)]
        cv2.line(canvas, (legend_x, y), (legend_x + 24, y), color, 3, cv2.LINE_AA)
        cv2.putText(canvas, str(label)[:30], (legend_x + 34, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (35, 35, 35), 1, cv2.LINE_AA)
    if len(labels) > 32:
        cv2.putText(canvas, f"... +{len(labels) - 32} more", (legend_x, legend_y + 32 * 24 + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (90, 90, 90), 1, cv2.LINE_AA)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    return bool(cv2.imwrite(str(output_path), canvas))


def write_state_plots(frames: List[Dict[str, Any]], output_dir: Path) -> List[Path]:
    times = relative_times(frames)
    series = OrderedDict()
    series.update(collect_robot_series(frames))
    series.update(collect_marker_summaries(frames))

    written: List[Path] = []
    for key, (values, labels) in series.items():
        out = output_dir / f"{safe_name(key)}.png"
        if draw_timeseries_png(out, times, values, labels, key):
            written.append(out)
    return written


def collect_all_state_series(frames: List[Dict[str, Any]]) -> Dict[str, Tuple[np.ndarray, List[str]]]:
    series = OrderedDict()
    series.update(collect_robot_series(frames))
    series.update(collect_marker_summaries(frames))
    return series


def select_dashboard_series(
    all_series: Dict[str, Tuple[np.ndarray, List[str]]],
    max_panels: int,
) -> "OrderedDict[str, Tuple[np.ndarray, List[str]]]":
    selected: "OrderedDict[str, Tuple[np.ndarray, List[str]]]" = OrderedDict()

    def add_exact(key: str) -> None:
        if key in all_series and key not in selected and len(selected) < max_panels:
            selected[key] = all_series[key]

    def add_prefix(prefix: str) -> None:
        for key in all_series:
            if key.startswith(prefix) and key not in selected and len(selected) < max_panels:
                selected[key] = all_series[key]

    for key in (
        "left.joint_state.position",
        "right.joint_state.position",
        "left.end_pose.position_xyz",
        "right.end_pose.position_xyz",
        "left.gripper_state.position",
        "right.gripper_state.position",
    ):
        add_exact(key)

    add_prefix("tactile_marker.right_gripper_sensor_1")
    add_prefix("tactile_marker.right_gripper_sensor_2")

    for key in all_series:
        if len(selected) >= max_panels:
            break
        if key not in selected:
            selected[key] = all_series[key]
    return selected


def reduce_dashboard_dims(
    key: str,
    values: np.ndarray,
    labels: List[str],
    max_dims: int,
) -> Tuple[np.ndarray, List[str]]:
    if values.ndim != 2 or values.shape[1] == 0:
        return values, labels

    if key.endswith(".summary"):
        preferred = ["mean_norm", "max_norm", "mean_dx"]
        indices = [labels.index(label) for label in preferred if label in labels]
        if indices:
            return values[:, indices], [labels[i] for i in indices]

    if "joint_state.position" in key and values.shape[1] > 4:
        indices = [0, 1, 2, 3]
        return values[:, indices], [labels[i] for i in indices]

    if values.shape[1] <= max_dims:
        return values, labels
    return values[:, :max_dims], labels[:max_dims]


def compact_dashboard_label(label: str) -> str:
    mapping = {
        "mean_norm": "avg",
        "max_norm": "max",
        "mean_dx": "dx",
        "mean_dy": "dy",
        "mean_dz": "dz",
        "std_dx": "sdx",
        "std_dy": "sdy",
        "std_dz": "sdz",
        "position": "pos",
        "velocity": "vel",
        "effort": "eff",
    }
    if label in mapping:
        return mapping[label]
    cleaned = label.replace("left_", "").replace("right_", "")
    cleaned = cleaned.replace("L_j", "j").replace("R_j", "j")
    cleaned = cleaned.replace("l_j", "j").replace("r_j", "j")
    return cleaned[:6]


def compact_dashboard_value(value: float) -> str:
    if not np.isfinite(value):
        return "nan"
    abs_value = abs(float(value))
    if abs_value >= 100:
        return f"{value:.0f}"
    if abs_value >= 10:
        return f"{value:.1f}"
    if abs_value >= 1:
        return f"{value:.2f}"
    return f"{value:.3f}"


def stream_for_topic(
    streams: "OrderedDict[Tuple[str, str], str]",
    patterns: Iterable[str],
) -> Optional[Tuple[str, str, str]]:
    lowered_patterns = [p.lower() for p in patterns]
    for (group, key), topic in streams.items():
        topic_l = str(topic).lower()
        if any(pattern in topic_l for pattern in lowered_patterns):
            return group, key, topic
    return None


def fit_image_to_box(
    image: Optional[np.ndarray],
    width: int,
    height: int,
    bg: Tuple[int, int, int],
    mode: str = "contain",
) -> np.ndarray:
    canvas = np.full((height, width, 3), bg, dtype=np.uint8)
    if image is None or image.size == 0:
        return canvas
    src_h, src_w = image.shape[:2]
    if src_h <= 0 or src_w <= 0:
        return canvas
    if mode == "cover":
        scale = max(width / src_w, height / src_h)
    else:
        scale = min(width / src_w, height / src_h)
    new_w = max(1, int(round(src_w * scale)))
    new_h = max(1, int(round(src_h * scale)))
    resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
    if mode == "cover":
        src_x0 = max(0, (new_w - width) // 2)
        src_y0 = max(0, (new_h - height) // 2)
        canvas[:, :] = resized[src_y0:src_y0 + height, src_x0:src_x0 + width]
    else:
        x0 = (width - new_w) // 2
        y0 = (height - new_h) // 2
        canvas[y0:y0 + new_h, x0:x0 + new_w] = resized
    return canvas


def draw_dashboard_background(width: int, height: int) -> np.ndarray:
    canvas = np.zeros((height, width, 3), dtype=np.uint8)
    top = np.array([24, 18, 8], dtype=np.float32)
    bottom = np.array([42, 28, 10], dtype=np.float32)
    for y in range(height):
        alpha = y / max(1, height - 1)
        color = (1 - alpha) * top + alpha * bottom
        canvas[y, :, :] = color.astype(np.uint8)

    # Subtle grid, like an operations dashboard rather than a plain video wall.
    for x in range(0, width, 64):
        cv2.line(canvas, (x, 0), (x, height), (38, 31, 18), 1)
    for y in range(0, height, 64):
        cv2.line(canvas, (0, y), (width, y), (38, 31, 18), 1)
    return canvas


def draw_card(
    canvas: np.ndarray,
    x: int,
    y: int,
    w: int,
    h: int,
    title: str,
    accent: Tuple[int, int, int],
    header_h: int = 34,
) -> Tuple[int, int, int, int]:
    # Shadow.
    cv2.rectangle(canvas, (x + 5, y + 5), (x + w + 5, y + h + 5), (4, 6, 8), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (18, 25, 31), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (70, 83, 94), 1)
    cv2.rectangle(canvas, (x + 1, y + 1), (x + w - 1, y + header_h), (29, 39, 48), -1)
    cv2.rectangle(canvas, (x + 1, y + 1), (x + 8, y + h - 1), accent, -1)
    cv2.putText(canvas, title[:54], (x + 18, y + 23), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (225, 235, 238), 1, cv2.LINE_AA)
    return x + 10, y + header_h + 8, w - 20, h - header_h - 18


def draw_camera_card(
    canvas: np.ndarray,
    frame: Dict[str, Any],
    frame_index: int,
    stream: Optional[Tuple[str, str, str]],
    rect: Tuple[int, int, int, int],
    title: str,
    accent: Tuple[int, int, int],
    current_time: float,
    main: bool = False,
    fit_mode: str = "contain",
) -> None:
    x, y, w, h = rect
    content = draw_card(canvas, x, y, w, h, title, accent, header_h=42 if main else 34)
    cx, cy, cw, ch = content
    image = None
    topic = ""
    if stream is not None:
        group, key, topic = stream
        image = decode_image(image_record_for(frame, group, key))
    canvas[cy:cy + ch, cx:cx + cw] = fit_image_to_box(image, cw, ch, bg=(12, 16, 20), mode=fit_mode)

    if main:
        cv2.putText(canvas, f"frame {frame_index:04d} | t={current_time:06.2f}s", (x + w - 270, y + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (178, 232, 255), 1, cv2.LINE_AA)
    if not topic:
        cv2.putText(canvas, "NO SIGNAL", (cx + 20, cy + ch // 2), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (70, 90, 105), 2, cv2.LINE_AA)


def select_dashboard_side_series(
    all_series: Dict[str, Tuple[np.ndarray, List[str]]],
    side: str,
    max_panels: int,
) -> "OrderedDict[str, Tuple[np.ndarray, List[str]]]":
    selected: "OrderedDict[str, Tuple[np.ndarray, List[str]]]" = OrderedDict()
    if side == "left":
        preferred = (
            "left.joint_state.position",
            "left.end_pose.position_xyz",
            "left.gripper_state.position",
            "tactile_marker.right_gripper_sensor_1_marker_offset_information.summary",
        )
        tactile_prefix = "tactile_marker.right_gripper_sensor_1"
        fallback_prefix = "left."
    else:
        preferred = (
            "right.joint_state.position",
            "right.end_pose.position_xyz",
            "right.gripper_state.position",
            "tactile_marker.right_gripper_sensor_2_marker_offset_information.summary",
        )
        tactile_prefix = "tactile_marker.right_gripper_sensor_2"
        fallback_prefix = "right."

    for key in preferred:
        if key in all_series and len(selected) < max_panels:
            selected[key] = all_series[key]
    for key in all_series:
        if len(selected) >= max_panels:
            break
        if key.startswith(tactile_prefix) and key not in selected:
            selected[key] = all_series[key]
    for key in all_series:
        if len(selected) >= max_panels:
            break
        if key.startswith(fallback_prefix) and key not in selected:
            selected[key] = all_series[key]
    return selected


def short_dashboard_title(key: str) -> str:
    mapping = {
        "left.joint_state.position": "Joint Position",
        "right.joint_state.position": "Joint Position",
        "left.end_pose.position_xyz": "End-Effector XYZ",
        "right.end_pose.position_xyz": "End-Effector XYZ",
        "left.gripper_state.position": "Gripper Position",
        "right.gripper_state.position": "Gripper Position",
        "tactile_marker.right_gripper_sensor_1_marker_offset_information.summary": "Tactile 1 Marker Motion",
        "tactile_marker.right_gripper_sensor_2_marker_offset_information.summary": "Tactile 2 Marker Motion",
    }
    return mapping.get(key, key.replace("_", " "))


def draw_dashboard_timeseries_panel(
    times: np.ndarray,
    values: np.ndarray,
    labels: List[str],
    title: str,
    current_index: int,
    width: int,
    height: int,
) -> np.ndarray:
    panel = np.full((height, width, 3), (18, 25, 31), dtype=np.uint8)
    if values.size == 0 or not np.isfinite(values).any():
        cv2.putText(panel, title[:70], (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (225, 235, 238), 1, cv2.LINE_AA)
        cv2.putText(panel, "no finite data", (12, 62), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 130, 145), 1, cv2.LINE_AA)
        return panel

    left, right, top, bottom = 40, 10, 58, 28
    plot_x0, plot_y0 = left, top
    plot_x1, plot_y1 = width - right, height - bottom

    finite_y = values[np.isfinite(values)]
    y_min, y_max = float(np.min(finite_y)), float(np.max(finite_y))
    if y_min == y_max:
        pad = max(1.0, abs(y_min) * 0.1)
        y_min -= pad
        y_max += pad
    else:
        pad = (y_max - y_min) * 0.08
        y_min -= pad
        y_max += pad

    finite_t = times[np.isfinite(times)]
    if finite_t.size == 0:
        finite_t = np.arange(values.shape[0], dtype=np.float64)
    x_min, x_max = float(finite_t[0]), float(finite_t[-1])
    if x_min == x_max:
        x_max = x_min + 1.0

    current_index = int(np.clip(current_index, 0, values.shape[0] - 1))
    current_time = times[current_index] if current_index < times.size and np.isfinite(times[current_index]) else float(current_index)

    def xy(t: float, y: float) -> Tuple[int, int]:
        x_pix = plot_x0 + int(round((t - x_min) / (x_max - x_min) * (plot_x1 - plot_x0)))
        y_pix = plot_y1 - int(round((y - y_min) / (y_max - y_min) * (plot_y1 - plot_y0)))
        return x_pix, y_pix

    cv2.putText(panel, title[:48], (12, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (225, 235, 238), 1, cv2.LINE_AA)
    legend_count = min(values.shape[1], 4)
    slot_w = max(48, (width - 24) // max(1, legend_count))
    for dim in range(legend_count):
        color = PLOT_COLORS[dim % len(PLOT_COLORS)]
        x0 = 12 + dim * slot_w
        y0 = 42
        label = compact_dashboard_label(labels[dim] if dim < len(labels) else f"d{dim}")
        value = compact_dashboard_value(float(values[current_index, dim]))
        text = f"{label} {value}"
        cv2.line(panel, (x0, y0), (x0 + 10, y0), color, 2, cv2.LINE_AA)
        cv2.putText(panel, text[:10], (x0 + 13, y0 + 4), cv2.FONT_HERSHEY_SIMPLEX, 0.30, (205, 218, 224), 1, cv2.LINE_AA)

    for i in range(4):
        alpha = i / 3.0
        x = int(plot_x0 + alpha * (plot_x1 - plot_x0))
        y = int(plot_y1 - alpha * (plot_y1 - plot_y0))
        cv2.line(panel, (x, plot_y0), (x, plot_y1), (39, 50, 58), 1)
        cv2.line(panel, (plot_x0, y), (plot_x1, y), (39, 50, 58), 1)
    cv2.rectangle(panel, (plot_x0, plot_y0), (plot_x1, plot_y1), (77, 93, 105), 1)
    cv2.putText(panel, f"{y_max:.3g}", (8, plot_y0 + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 166, 175), 1, cv2.LINE_AA)
    cv2.putText(panel, f"{y_min:.3g}", (8, plot_y1), cv2.FONT_HERSHEY_SIMPLEX, 0.34, (150, 166, 175), 1, cv2.LINE_AA)

    draw_step = max(1, values.shape[0] // 900)
    for dim in range(values.shape[1]):
        color = PLOT_COLORS[dim % len(PLOT_COLORS)]
        last_full: Optional[Tuple[int, int]] = None
        last_hist: Optional[Tuple[int, int]] = None
        for idx in range(0, values.shape[0], draw_step):
            t = times[idx] if idx < times.size else float(idx)
            y = values[idx, dim]
            if not np.isfinite(t) or not np.isfinite(y):
                last_full = None
                last_hist = None
                continue
            point = xy(float(t), float(y))
            if last_full is not None:
                cv2.line(panel, last_full, point, (58, 67, 73), 1, cv2.LINE_AA)
            if idx <= current_index:
                if last_hist is not None:
                    cv2.line(panel, last_hist, point, color, 2, cv2.LINE_AA)
                last_hist = point
            last_full = point

        y_now = values[current_index, dim]
        if np.isfinite(y_now):
            cv2.circle(panel, xy(float(current_time), float(y_now)), 3, color, -1, cv2.LINE_AA)

    cursor_x = xy(float(current_time), y_min)[0]
    cv2.line(panel, (cursor_x, plot_y0), (cursor_x, plot_y1), (80, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(panel, f"t={current_time:.2f}s", (plot_x0, height - 9), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (185, 205, 214), 1, cv2.LINE_AA)
    return panel


def draw_state_column(
    canvas: np.ndarray,
    times: np.ndarray,
    selected_series: "OrderedDict[str, Tuple[np.ndarray, List[str]]]",
    frame_index: int,
    x: int,
    y: int,
    w: int,
    h: int,
    title: str,
    accent: Tuple[int, int, int],
    max_dims: int,
) -> None:
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (13, 20, 26), -1)
    cv2.rectangle(canvas, (x, y), (x + w, y + h), (65, 82, 94), 1)
    cv2.rectangle(canvas, (x, y), (x + w, y + 42), (26, 36, 45), -1)
    cv2.rectangle(canvas, (x, y), (x + 8, y + h), accent, -1)
    cv2.putText(canvas, title, (x + 20, y + 27), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (232, 240, 242), 1, cv2.LINE_AA)

    if not selected_series:
        cv2.putText(canvas, "No state series available", (x + 25, y + 85), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (120, 150, 164), 1, cv2.LINE_AA)
        return

    gap = 10
    content_y = y + 56
    panel_w = w - 24
    panel_h = int((h - 70 - gap * (len(selected_series) - 1)) / max(1, len(selected_series)))
    panel_h = max(105, panel_h)
    for i, (key, (values, labels)) in enumerate(selected_series.items()):
        panel_values, panel_labels = reduce_dashboard_dims(key, values, labels, max_dims=max_dims)
        panel = draw_dashboard_timeseries_panel(times, panel_values, panel_labels, short_dashboard_title(key), frame_index, panel_w, panel_h)
        py = content_y + i * (panel_h + gap)
        if py + panel_h <= y + h - 10:
            canvas[py:py + panel_h, x + 12:x + 12 + panel_w] = panel


def draw_dashboard_header(
    canvas: np.ndarray,
    input_name: str,
    frame_index: int,
    frame_count: int,
    current_time: float,
    fps: float,
) -> None:
    h, w = canvas.shape[:2]
    cv2.rectangle(canvas, (0, 0), (w, 74), (13, 20, 27), -1)
    cv2.line(canvas, (0, 74), (w, 74), (68, 88, 102), 1)
    cv2.putText(canvas, "MARVIN DUAL-ARM TELEOP REPLAY", (38, 45), cv2.FONT_HERSHEY_SIMPLEX, 1.05, (232, 241, 244), 2, cv2.LINE_AA)
    cv2.putText(canvas, input_name[:60], (650, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (156, 184, 196), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"frame {frame_index:04d}/{frame_count - 1:04d}", (650, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (156, 184, 196), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"t={current_time:06.2f}s", (1510, 31), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (80, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, f"{fps:.1f} FPS", (1510, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (156, 184, 196), 1, cv2.LINE_AA)


def build_professional_dashboard_frame(
    frames: List[Dict[str, Any]],
    frame_index: int,
    streams: "OrderedDict[Tuple[str, str], str]",
    times: np.ndarray,
    left_series: "OrderedDict[str, Tuple[np.ndarray, List[str]]]",
    right_series: "OrderedDict[str, Tuple[np.ndarray, List[str]]]",
    input_name: str,
    fps: float,
    max_dims: int,
) -> np.ndarray:
    width, height = 1920, 1080
    canvas = draw_dashboard_background(width, height)
    current_time = times[frame_index] if frame_index < times.size and np.isfinite(times[frame_index]) else float(frame_index)
    draw_dashboard_header(canvas, input_name, frame_index, len(frames), float(current_time), fps)

    left_x, side_y, side_w, side_h = 16, 92, 270, 950
    right_x = width - side_w - 16
    center_x = left_x + side_w + 18
    center_w = right_x - center_x - 18

    draw_state_column(canvas, times, left_series, frame_index, left_x, side_y, side_w, side_h, "LEFT ARM STATE", (255, 176, 30), max_dims)
    draw_state_column(canvas, times, right_series, frame_index, right_x, side_y, side_w, side_h, "RIGHT ARM STATE", (80, 220, 255), max_dims)

    frame = frames[frame_index]
    external_stream = stream_for_topic(streams, ("external_camera", "external"))
    left_wrist_stream = stream_for_topic(streams, ("left_wrist_camera", "left_wrist"))
    right_wrist_stream = stream_for_topic(streams, ("right_wrist_camera", "right_wrist"))
    tactile_1_stream = stream_for_topic(streams, ("right_gripper_sensor_1", "gripper_sensor_1", "sensor_1"))
    tactile_2_stream = stream_for_topic(streams, ("right_gripper_sensor_2", "gripper_sensor_2", "sensor_2"))

    # Every video content region is close to 4:3. The card itself is slightly
    # taller because it includes a title bar and padding.
    main_w, main_h = 936, 704
    main_x = center_x + (center_w - main_w) // 2
    draw_camera_card(canvas, frame, frame_index, external_stream, (main_x, 94, main_w, main_h), "EXTERNAL CAMERA | MAIN VIEW", (80, 255, 255), float(current_time), main=True, fit_mode="contain")

    gap = 22
    tile_w, tile_h = 306, 266
    small_total_w = tile_w * 4 + gap * 3
    small_x = center_x + (center_w - small_total_w) // 2
    small_y = 804
    draw_camera_card(canvas, frame, frame_index, left_wrist_stream, (small_x, small_y, tile_w, tile_h), "LEFT WRIST", (255, 176, 30), float(current_time), fit_mode="contain")
    draw_camera_card(canvas, frame, frame_index, right_wrist_stream, (small_x + (tile_w + gap), small_y, tile_w, tile_h), "RIGHT WRIST", (80, 220, 255), float(current_time), fit_mode="contain")
    draw_camera_card(canvas, frame, frame_index, tactile_1_stream, (small_x + 2 * (tile_w + gap), small_y, tile_w, tile_h), "TACTILE 1", (90, 255, 120), float(current_time), fit_mode="contain")
    draw_camera_card(canvas, frame, frame_index, tactile_2_stream, (small_x + 3 * (tile_w + gap), small_y, tile_w, tile_h), "TACTILE 2", (190, 120, 255), float(current_time), fit_mode="contain")
    return canvas


def write_dashboard_video(
    frames: List[Dict[str, Any]],
    streams: "OrderedDict[Tuple[str, str], str]",
    output_path: Path,
    fps: float,
    codec: str,
    stride: int,
    max_frames: int,
    image_tile_width: int,
    image_tile_height: int,
    state_panel_width: int,
    state_panel_height: int,
    state_cols: int,
    max_panels: int,
    max_dims: int,
) -> Optional[Path]:
    if not frames:
        return None

    times = relative_times(frames)
    all_series = collect_all_state_series(frames)
    side_panels = max(1, max_panels // 2)
    left_series = select_dashboard_side_series(all_series, "left", max_panels=side_panels)
    right_series = select_dashboard_side_series(all_series, "right", max_panels=side_panels)
    if not streams and not left_series and not right_series:
        return None

    writer: Optional[cv2.VideoWriter] = None
    written = 0
    for frame_index in iter_frame_indices(len(frames), stride, max_frames):
        canvas = build_professional_dashboard_frame(
            frames=frames,
            frame_index=frame_index,
            streams=streams,
            times=times,
            left_series=left_series,
            right_series=right_series,
            input_name=output_path.parent.parent.name,
            fps=fps,
            max_dims=max_dims,
        )
        canvas = np.ascontiguousarray(canvas)
        size = (int(canvas.shape[1]), int(canvas.shape[0]))
        if writer is None:
            writer = make_writer(output_path, fps, size, codec)
        writer.write(canvas)
        written += 1

    if writer is not None:
        writer.release()
    return output_path if written else None


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Convert a Marvin recorder pkl into videos and state plot images.")
    parser.add_argument("--input", required=True, type=Path, help="Input episode_XXXX.pkl file.")
    parser.add_argument("--output-dir", type=Path, default=None, help="Output directory. Default: data_convert/visualizations/<pkl_stem>.")
    parser.add_argument("--fps", type=float, default=0.0, help="Output video FPS. Default: metadata sample_rate_hz or 20.")
    parser.add_argument("--stride", type=int, default=1, help="Use every Nth frame.")
    parser.add_argument("--max-frames", type=int, default=0, help="Maximum frames to export after stride. 0 means all.")
    parser.add_argument("--codec", default="mp4v", help="FourCC codec for mp4 output, default mp4v.")
    parser.add_argument("--tile-width", type=int, default=480)
    parser.add_argument("--tile-height", type=int, default=360)
    parser.add_argument("--no-videos", action="store_true")
    parser.add_argument("--no-topic-videos", action="store_true", help="Do not generate one video per image topic.")
    parser.add_argument("--no-combined-video", action="store_true")
    parser.add_argument("--no-state-plots", action="store_true")
    parser.add_argument("--no-dashboard-video", action="store_true", help="Do not generate the image+state dashboard video.")
    parser.add_argument("--dashboard-max-panels", type=int, default=8, help="Maximum representative state panels in the dashboard video.")
    parser.add_argument("--dashboard-max-dims", type=int, default=7, help="Maximum plotted dimensions per dashboard state panel.")
    parser.add_argument("--dashboard-image-tile-width", type=int, default=320)
    parser.add_argument("--dashboard-image-tile-height", type=int, default=240)
    parser.add_argument("--dashboard-state-panel-width", type=int, default=640)
    parser.add_argument("--dashboard-state-panel-height", type=int, default=180)
    parser.add_argument("--dashboard-state-cols", type=int, default=2)
    parser.add_argument("--no-overlay", action="store_true", help="Do not draw topic/frame labels on videos.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    data = load_pkl(args.input)
    frames = list(data.get("frames") or [])
    metadata = dict(data.get("metadata") or {})
    fps = float(args.fps or metadata.get("sample_rate_hz") or 20.0)
    output_dir = args.output_dir or (Path(__file__).resolve().parent / "visualizations" / args.input.stem)
    videos_dir = output_dir / "videos"
    plots_dir = output_dir / "state_plots"
    output_dir.mkdir(parents=True, exist_ok=True)

    produced: Dict[str, Any] = {
        "input": str(args.input),
        "output_dir": str(output_dir),
        "frame_count": len(frames),
        "fps": fps,
        "videos": [],
        "state_plots": [],
    }

    streams = image_streams(frames)
    if not args.no_videos:
        if not args.no_topic_videos:
            for (group, key), topic in streams.items():
                path = write_topic_video(
                    frames=frames,
                    group=group,
                    key=key,
                    topic=topic,
                    output_dir=videos_dir,
                    fps=fps,
                    codec=args.codec,
                    stride=args.stride,
                    max_frames=args.max_frames,
                    overlay=not args.no_overlay,
                )
                if path:
                    produced["videos"].append(str(path))
        if not args.no_combined_video:
            combined = write_tiled_video(
                frames=frames,
                streams=streams,
                output_path=videos_dir / "all_image_topics_grid.mp4",
                fps=fps,
                codec=args.codec,
                stride=args.stride,
                max_frames=args.max_frames,
                tile_width=args.tile_width,
                tile_height=args.tile_height,
            )
            if combined:
                produced["videos"].append(str(combined))
        if not args.no_dashboard_video:
            dashboard = write_dashboard_video(
                frames=frames,
                streams=streams,
                output_path=videos_dir / "dashboard_with_state.mp4",
                fps=fps,
                codec=args.codec,
                stride=args.stride,
                max_frames=args.max_frames,
                image_tile_width=args.dashboard_image_tile_width,
                image_tile_height=args.dashboard_image_tile_height,
                state_panel_width=args.dashboard_state_panel_width,
                state_panel_height=args.dashboard_state_panel_height,
                state_cols=args.dashboard_state_cols,
                max_panels=args.dashboard_max_panels,
                max_dims=args.dashboard_max_dims,
            )
            if dashboard:
                produced["videos"].append(str(dashboard))

    if not args.no_state_plots:
        produced["state_plots"] = [str(x) for x in write_state_plots(frames, plots_dir)]

    summary_path = output_dir / "summary.json"
    summary_path.write_text(json.dumps(produced, indent=2), encoding="utf-8")
    print(json.dumps(produced, indent=2))


if __name__ == "__main__":
    main()
