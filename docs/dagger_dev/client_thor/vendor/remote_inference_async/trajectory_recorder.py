"""Non-blocking, operator-segmented trajectory recording for Tianji inference.

The robot control loop only enqueues already-available values.  JSON encoding,
disk flushes, fsync and segment summaries are handled by a dedicated writer
thread.  A local Unix-domain control socket lets an operator start and stop
segments without sharing stdin with the inference process.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass, field
from datetime import datetime, timezone
import json
import logging
import math
import os
from pathlib import Path
import queue
import re
import socket
import stat
import threading
import time
from typing import Any, TextIO
import uuid


LOGGER = logging.getLogger("tianji_trajectory_recorder")

TRAJECTORY_SCHEMA = "tianji_right_arm_trajectory/v1"
SUMMARY_SCHEMA = "tianji_right_arm_trajectory_summary/v1"
CONTROL_SCHEMA = "tianji_right_arm_trajectory_control/v1"

DEFAULT_OUTPUT_DIR = Path.home() / "Desktop" / "机器人推理轨迹"
DEFAULT_CONTROL_SOCKET = Path("/tmp/tianji_right_arm_trajectory.sock")
DEFAULT_STATUS_PATH = Path("/tmp/tianji_right_arm_trajectory_status.json")

_MAX_CONTROL_BYTES = 64 * 1024
_RESERVED_RECORD_KEYS = {
    "schema",
    "segment_id",
    "sequence",
    "event",
    "wall_time_ns",
    "wall_time_utc",
    "monotonic_ns",
}


class RecorderControlError(RuntimeError):
    """Raised when the recorder control endpoint rejects a command."""


def _utc_iso(wall_time_ns: int) -> str:
    return datetime.fromtimestamp(
        wall_time_ns / 1_000_000_000,
        tz=timezone.utc,
    ).isoformat(timespec="microseconds").replace("+00:00", "Z")


def _json_safe(value: Any) -> Any:
    """Return a strict-JSON-compatible copy without importing NumPy."""

    if value is None or isinstance(value, (str, bool, int)):
        return value
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    item_method = getattr(value, "item", None)
    if callable(item_method):
        try:
            return _json_safe(item_method())
        except (TypeError, ValueError):
            pass
    return str(value)


def _json_line(payload: dict[str, Any]) -> str:
    return json.dumps(
        _json_safe(payload),
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    ) + "\n"


def _atomic_write_json(path: Path, payload: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.{uuid.uuid4().hex}.tmp")
    try:
        with temporary.open("x", encoding="utf-8") as handle:
            json.dump(
                _json_safe(payload),
                handle,
                ensure_ascii=False,
                allow_nan=False,
                indent=2,
                sort_keys=True,
            )
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def _label_slug(label: str) -> str:
    cleaned = re.sub(r"[^\w.-]+", "_", label.strip(), flags=re.UNICODE)
    return cleaned.strip("._")[:48]


@dataclass
class _Segment:
    segment_id: str
    label: str
    trajectory_path: Path
    summary_path: Path
    camera_dir: Path
    handle: TextIO
    started_wall_ns: int
    started_monotonic_ns: int
    next_sequence: int = 1
    events_queued: int = 0
    events_written: int = 0
    events_dropped: int = 0
    serialization_errors: int = 0
    camera_frames_queued: int = 0
    camera_frames_written: int = 0
    camera_frames_dropped: int = 0
    camera_write_errors: int = 0
    camera_bytes_written: int = 0
    camera_frames_by_name: Counter[str] = field(default_factory=Counter)
    counts_by_event: Counter[str] = field(default_factory=Counter)
    stopped_wall_ns: int | None = None
    stopped_monotonic_ns: int | None = None
    stop_reason: str | None = None
    summary: dict[str, Any] | None = None
    closed: threading.Event = field(default_factory=threading.Event)


class RealtimeTrajectoryRecorder:
    """Record operator-defined trajectory segments without blocking control."""

    def __init__(
        self,
        *,
        output_dir: str | Path = DEFAULT_OUTPUT_DIR,
        control_socket: str | Path = DEFAULT_CONTROL_SOCKET,
        status_path: str | Path = DEFAULT_STATUS_PATH,
        runtime_metadata: dict[str, Any] | None = None,
        queue_size: int = 50_000,
        flush_interval_s: float = 0.25,
    ) -> None:
        if queue_size <= 0 or flush_interval_s <= 0:
            raise ValueError("queue_size and flush_interval_s must be positive")
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.control_socket = Path(control_socket)
        self.status_path = Path(status_path)
        self.runtime_metadata = _json_safe(runtime_metadata or {})
        self.flush_interval_s = float(flush_interval_s)

        self._queue: queue.Queue[tuple[str, _Segment | None, Any]] = queue.Queue(
            maxsize=queue_size
        )
        self._state_lock = threading.Lock()
        self._lifecycle_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._active: _Segment | None = None
        self._last_summary: dict[str, Any] | None = None
        self._writer_error: str | None = None
        self._started = False
        self._closed = False
        self._stop = threading.Event()
        self._server: socket.socket | None = None
        self._writer_thread = threading.Thread(
            target=self._writer_loop,
            name="trajectory-jsonl-writer",
            daemon=True,
        )
        self._control_thread = threading.Thread(
            target=self._control_loop,
            name="trajectory-control-socket",
            daemon=True,
        )

    @property
    def is_recording(self) -> bool:
        with self._state_lock:
            return self._active is not None and self._writer_error is None

    def start(self) -> None:
        with self._lifecycle_lock:
            if self._started:
                return
            if self._closed:
                raise RuntimeError("trajectory recorder has already been closed")
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self._prepare_control_socket()
            self._writer_thread.start()
            self._control_thread.start()
            self._started = True
        self._write_status()
        LOGGER.info(
            "Trajectory recorder ready: output=%s, control=%s",
            self.output_dir,
            self.control_socket,
        )

    def _prepare_control_socket(self) -> None:
        self.control_socket.parent.mkdir(parents=True, exist_ok=True)
        try:
            existing_mode = self.control_socket.lstat().st_mode
        except FileNotFoundError:
            existing_mode = None
        if existing_mode is not None:
            if not stat.S_ISSOCK(existing_mode):
                raise RuntimeError(
                    f"trajectory control path exists and is not a socket: "
                    f"{self.control_socket}"
                )
            probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            probe.settimeout(0.2)
            try:
                probe.connect(str(self.control_socket))
            except OSError:
                self.control_socket.unlink()
            else:
                raise RuntimeError(
                    f"another trajectory recorder is already listening at "
                    f"{self.control_socket}"
                )
            finally:
                probe.close()

        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        try:
            server.bind(str(self.control_socket))
            os.chmod(self.control_socket, 0o600)
            server.listen(4)
            server.settimeout(0.2)
        except BaseException:
            server.close()
            try:
                self.control_socket.unlink()
            except FileNotFoundError:
                pass
            raise
        self._server = server

    def start_segment(self, label: str = "") -> dict[str, Any]:
        if not self._started or self._closed:
            raise RuntimeError("trajectory recorder is not running")
        wall_ns = time.time_ns()
        monotonic_ns = time.monotonic_ns()
        stamp = datetime.fromtimestamp(
            wall_ns / 1_000_000_000,
            tz=timezone.utc,
        ).strftime("%Y%m%dT%H%M%S.%fZ")
        slug = _label_slug(label)
        suffix = f"_{slug}" if slug else ""
        segment_id = f"trajectory_{stamp}_{uuid.uuid4().hex[:8]}{suffix}"
        trajectory_path = self.output_dir / f"{segment_id}.jsonl"
        summary_path = self.output_dir / f"{segment_id}.summary.json"
        camera_dir = self.output_dir / f"{segment_id}.cameras"

        with self._state_lock:
            if self._active is not None:
                raise RecorderControlError(
                    f"a segment is already recording: {self._active.trajectory_path}"
                )
            if self._writer_error is not None:
                raise RecorderControlError(
                    f"trajectory writer is unavailable: {self._writer_error}"
                )
            handle = trajectory_path.open("x", encoding="utf-8", buffering=1)
            segment = _Segment(
                segment_id=segment_id,
                label=label.strip(),
                trajectory_path=trajectory_path,
                summary_path=summary_path,
                camera_dir=camera_dir,
                handle=handle,
                started_wall_ns=wall_ns,
                started_monotonic_ns=monotonic_ns,
            )
            start_record = {
                "schema": TRAJECTORY_SCHEMA,
                "segment_id": segment_id,
                "sequence": 0,
                "event": "segment_start",
                "wall_time_ns": wall_ns,
                "wall_time_utc": _utc_iso(wall_ns),
                "monotonic_ns": monotonic_ns,
                "label": segment.label,
                "runtime": self.runtime_metadata,
                "units": {
                    "right_arm_joint_position": "degree",
                    "right_arm_joint_velocity": "degree_per_second",
                    "right_arm_torque": "newton_meter",
                    "left_gripper_position": "radian_and_calibrated_0_open_1_closed",
                    "left_gripper_velocity": "radian_per_second",
                    "left_gripper_torque": "newton_meter",
                    "right_gripper_position": "radian_and_calibrated_0_open_1_closed",
                    "right_gripper_velocity": "radian_per_second",
                    "right_gripper_torque": "newton_meter",
                },
            }
            try:
                handle.write(_json_line(start_record))
                handle.flush()
            except BaseException:
                handle.close()
                try:
                    trajectory_path.unlink()
                except FileNotFoundError:
                    pass
                raise
            segment.events_written = 1
            segment.counts_by_event["segment_start"] = 1
            self._active = segment

        self._write_status()
        LOGGER.info("Trajectory segment started: %s", trajectory_path)
        return self.status()

    def stop_segment(
        self,
        *,
        reason: str = "operator",
        timeout_s: float = 10.0,
    ) -> dict[str, Any]:
        with self._state_lock:
            segment = self._active
            if segment is None:
                raise RecorderControlError("no trajectory segment is recording")
            self._active = None
            segment.stop_reason = str(reason)
            segment.stopped_wall_ns = time.time_ns()
            segment.stopped_monotonic_ns = time.monotonic_ns()
            end_sequence = segment.next_sequence
            segment.next_sequence += 1
            end_record = {
                "schema": TRAJECTORY_SCHEMA,
                "segment_id": segment.segment_id,
                "sequence": end_sequence,
                "event": "segment_end",
                "wall_time_ns": segment.stopped_wall_ns,
                "wall_time_utc": _utc_iso(segment.stopped_wall_ns),
                "monotonic_ns": segment.stopped_monotonic_ns,
                "reason": segment.stop_reason,
                "events_queued": segment.events_queued,
                "events_dropped": segment.events_dropped,
            }
        try:
            self._queue.put(("close", segment, end_record), timeout=timeout_s)
        except queue.Full as exc:
            raise RuntimeError("trajectory writer queue did not drain during stop") from exc
        if not segment.closed.wait(timeout_s):
            raise RuntimeError(
                f"trajectory segment did not finish within {timeout_s:.1f}s: "
                f"{segment.trajectory_path}"
            )
        self._write_status()
        if segment.summary is None:
            raise RuntimeError(
                f"trajectory segment closed without a summary: {segment.trajectory_path}"
            )
        LOGGER.info("Trajectory segment stopped: %s", segment.trajectory_path)
        return dict(segment.summary)

    def record(self, event: str, **payload: Any) -> bool:
        """Enqueue one event; return immediately and never wait for disk I/O."""

        if not event or not isinstance(event, str):
            raise ValueError("trajectory event must be a non-empty string")
        conflicting = _RESERVED_RECORD_KEYS.intersection(payload)
        if conflicting:
            raise ValueError(f"reserved trajectory keys in payload: {sorted(conflicting)}")
        with self._state_lock:
            segment = self._active
            if segment is None or self._writer_error is not None:
                return False
            # Timestamp after taking the same lock that allocates sequence IDs,
            # so records from the 20 Hz arm loop and 100 Hz gripper sampler have
            # a deterministic temporal order even when they arrive together.
            wall_ns = time.time_ns()
            monotonic_ns = time.monotonic_ns()
            sequence = segment.next_sequence
            segment.next_sequence += 1
            record = {
                "schema": TRAJECTORY_SCHEMA,
                "segment_id": segment.segment_id,
                "sequence": sequence,
                "event": event,
                "wall_time_ns": wall_ns,
                "wall_time_utc": _utc_iso(wall_ns),
                "monotonic_ns": monotonic_ns,
                **payload,
            }
            try:
                self._queue.put_nowait(("record", segment, record))
            except queue.Full:
                segment.events_dropped += 1
                return False
            segment.events_queued += 1
            return True

    def record_camera_frames(
        self,
        *,
        observation_timestep: int,
        request_id: str,
        images: list[tuple[str, bytes, int, int]],
    ) -> list[dict[str, Any]]:
        """Queue the exact JPEG policy inputs without blocking control.

        The returned paths can be embedded in the matching ``observation_sent``
        record. Camera I/O is diagnostic only: a write failure is counted in
        the segment summary and never stops inference.
        """

        if observation_timestep < 0:
            raise ValueError("observation_timestep must be non-negative")
        request_token = re.sub(
            r"[^A-Za-z0-9_.-]+", "_", str(request_id)
        ).strip("._")
        if not request_token:
            raise ValueError(
                "request_id must contain at least one safe filename character"
            )

        prepared: list[tuple[str, str, bytes]] = []
        metadata: list[dict[str, Any]] = []
        for camera_name, jpeg, width, height in images:
            camera_token = re.sub(
                r"[^A-Za-z0-9_.-]+", "_", str(camera_name)
            ).strip("._")
            if not camera_token:
                raise ValueError(
                    "camera name must contain at least one safe filename character"
                )
            relative_path = (
                f"{camera_token}/{observation_timestep:08d}_{request_token}.jpg"
            )
            frame = bytes(jpeg)
            prepared.append((camera_token, relative_path, frame))
            metadata.append(
                {
                    "name": str(camera_name),
                    "width": int(width),
                    "height": int(height),
                    "jpeg_bytes": len(frame),
                    "recording_path": relative_path,
                }
            )

        if not prepared:
            return []
        with self._state_lock:
            segment = self._active
            if segment is None or self._writer_error is not None:
                return []
            try:
                self._queue.put_nowait(("camera_frames", segment, prepared))
            except queue.Full:
                segment.camera_frames_dropped += len(prepared)
                return []
            segment.camera_frames_queued += len(prepared)
        return metadata

    def status(self) -> dict[str, Any]:
        with self._state_lock:
            segment = self._active
            result: dict[str, Any] = {
                "schema": CONTROL_SCHEMA,
                "recorder_online": self._started and not self._closed,
                "recording": segment is not None and self._writer_error is None,
                "segment_open": segment is not None,
                "output_dir": str(self.output_dir),
                "control_socket": str(self.control_socket),
                "writer_error": self._writer_error,
            }
            if segment is not None:
                result["active_segment"] = {
                    "segment_id": segment.segment_id,
                    "label": segment.label,
                    "trajectory_path": str(segment.trajectory_path),
                    "summary_path": str(segment.summary_path),
                    "camera_dir": str(segment.camera_dir),
                    "started_wall_time_ns": segment.started_wall_ns,
                    "started_wall_time_utc": _utc_iso(segment.started_wall_ns),
                    "events_queued": segment.events_queued,
                    "events_written": segment.events_written,
                    "events_dropped": segment.events_dropped,
                    "camera_frames_queued": segment.camera_frames_queued,
                    "camera_frames_written": segment.camera_frames_written,
                    "camera_frames_dropped": segment.camera_frames_dropped,
                    "camera_write_errors": segment.camera_write_errors,
                    "queue_depth": self._queue.qsize(),
                }
            if self._last_summary is not None:
                result["last_segment"] = dict(self._last_summary)
            return result

    def _write_status(self) -> None:
        with self._status_lock:
            try:
                _atomic_write_json(self.status_path, self.status())
            except OSError as exc:
                LOGGER.warning("Could not update trajectory status file: %s", exc)

    def _writer_loop(self) -> None:
        last_flush_at: dict[str, float] = {}
        while True:
            kind, segment, payload = self._queue.get()
            if kind == "shutdown":
                return
            if segment is None:
                continue
            if kind == "record":
                try:
                    segment.handle.write(_json_line(payload))
                    now = time.monotonic()
                    previous_flush = last_flush_at.get(segment.segment_id, 0.0)
                    if now - previous_flush >= self.flush_interval_s:
                        segment.handle.flush()
                        last_flush_at[segment.segment_id] = now
                except (OSError, TypeError, ValueError) as exc:
                    with self._state_lock:
                        segment.serialization_errors += 1
                        self._writer_error = f"{type(exc).__name__}: {exc}"
                    LOGGER.exception("Trajectory writer failed for %s", segment.trajectory_path)
                else:
                    with self._state_lock:
                        segment.events_written += 1
                        segment.counts_by_event[str(payload.get("event", "unknown"))] += 1
                continue
            if kind == "camera_frames":
                for camera_name, relative_path, jpeg in payload:
                    output_path = segment.camera_dir / relative_path
                    try:
                        output_path.parent.mkdir(parents=True, exist_ok=True)
                        with output_path.open("xb") as image_handle:
                            image_handle.write(jpeg)
                    except OSError:
                        with self._state_lock:
                            segment.camera_write_errors += 1
                        LOGGER.exception(
                            "Could not record policy camera frame %s", output_path
                        )
                    else:
                        with self._state_lock:
                            segment.camera_frames_written += 1
                            segment.camera_bytes_written += len(jpeg)
                            segment.camera_frames_by_name[camera_name] += 1
                continue
            if kind == "close":
                try:
                    segment.handle.write(_json_line(payload))
                    with self._state_lock:
                        segment.events_written += 1
                        segment.counts_by_event["segment_end"] += 1
                    segment.handle.flush()
                    os.fsync(segment.handle.fileno())
                    segment.handle.close()
                    summary = self._segment_summary(segment)
                    _atomic_write_json(segment.summary_path, summary)
                    _atomic_write_json(
                        self.output_dir / "latest_segment.json",
                        {
                            "schema": SUMMARY_SCHEMA,
                            "trajectory_path": str(segment.trajectory_path),
                            "summary_path": str(segment.summary_path),
                            "segment_id": segment.segment_id,
                            "stopped_wall_time_ns": segment.stopped_wall_ns,
                        },
                    )
                    segment.summary = summary
                    with self._state_lock:
                        self._last_summary = summary
                except BaseException as exc:
                    with self._state_lock:
                        self._writer_error = f"{type(exc).__name__}: {exc}"
                    LOGGER.exception("Could not finalize trajectory %s", segment.trajectory_path)
                    try:
                        segment.handle.close()
                    except OSError:
                        pass
                finally:
                    last_flush_at.pop(segment.segment_id, None)
                    segment.closed.set()

    def _segment_summary(self, segment: _Segment) -> dict[str, Any]:
        stopped_wall_ns = segment.stopped_wall_ns or time.time_ns()
        stopped_monotonic_ns = segment.stopped_monotonic_ns or time.monotonic_ns()
        return {
            "schema": SUMMARY_SCHEMA,
            "segment_id": segment.segment_id,
            "label": segment.label,
            "trajectory_path": str(segment.trajectory_path),
            "summary_path": str(segment.summary_path),
            "camera_dir": str(segment.camera_dir),
            "started_wall_time_ns": segment.started_wall_ns,
            "started_wall_time_utc": _utc_iso(segment.started_wall_ns),
            "stopped_wall_time_ns": stopped_wall_ns,
            "stopped_wall_time_utc": _utc_iso(stopped_wall_ns),
            "duration_seconds": (
                stopped_monotonic_ns - segment.started_monotonic_ns
            )
            / 1_000_000_000,
            "stop_reason": segment.stop_reason,
            "events_queued": segment.events_queued,
            "events_written": segment.events_written,
            "events_dropped": segment.events_dropped,
            "serialization_errors": segment.serialization_errors,
            "camera_frames_queued": segment.camera_frames_queued,
            "camera_frames_written": segment.camera_frames_written,
            "camera_frames_dropped": segment.camera_frames_dropped,
            "camera_write_errors": segment.camera_write_errors,
            "camera_bytes_written": segment.camera_bytes_written,
            "camera_frames_by_name": dict(
                sorted(segment.camera_frames_by_name.items())
            ),
            "counts_by_event": dict(sorted(segment.counts_by_event.items())),
            "runtime": self.runtime_metadata,
        }

    def _control_loop(self) -> None:
        server = self._server
        if server is None:
            return
        while not self._stop.is_set():
            try:
                connection, _address = server.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                LOGGER.exception("Trajectory control socket accept failed")
                continue
            with connection:
                connection.settimeout(2.0)
                response: dict[str, Any]
                try:
                    request = self._read_control_request(connection)
                    response = self._execute_control_request(request)
                except BaseException as exc:
                    response = {
                        "schema": CONTROL_SCHEMA,
                        "ok": False,
                        "error": f"{type(exc).__name__}: {exc}",
                    }
                try:
                    connection.sendall(_json_line(response).encode("utf-8"))
                except OSError:
                    LOGGER.debug("Trajectory control client disconnected before reply")

    @staticmethod
    def _read_control_request(connection: socket.socket) -> dict[str, Any]:
        chunks = bytearray()
        while len(chunks) <= _MAX_CONTROL_BYTES:
            data = connection.recv(4096)
            if not data:
                break
            chunks.extend(data)
            if b"\n" in data:
                break
        if len(chunks) > _MAX_CONTROL_BYTES:
            raise RecorderControlError("trajectory control request is too large")
        line = bytes(chunks).split(b"\n", 1)[0]
        if not line:
            raise RecorderControlError("empty trajectory control request")
        request = json.loads(line.decode("utf-8"))
        if not isinstance(request, dict):
            raise RecorderControlError("trajectory control request must be a JSON object")
        return request

    def _execute_control_request(self, request: dict[str, Any]) -> dict[str, Any]:
        command = str(request.get("command", "")).strip().lower()
        if command == "status":
            return {"ok": True, **self.status()}
        if command == "start":
            result = self.start_segment(str(request.get("label", "")))
            return {"ok": True, "command": "start", **result}
        if command == "stop":
            result = self.stop_segment(reason="operator")
            return {"ok": True, "command": "stop", "segment": result, **self.status()}
        raise RecorderControlError(f"unknown trajectory command: {command!r}")

    def close(self, *, reason: str = "client_shutdown") -> None:
        with self._lifecycle_lock:
            if self._closed:
                return
            self._closed = True
        # A writer error makes ``is_recording`` false so producers stop adding
        # data, but an already-open segment must still be finalized (or at
        # least have its file handle closed) during process shutdown.
        with self._state_lock:
            has_active_segment = self._active is not None
        if has_active_segment:
            try:
                self.stop_segment(reason=reason)
            except BaseException:
                LOGGER.exception("Could not close active trajectory segment")
        self._stop.set()
        server = self._server
        if server is not None:
            try:
                server.close()
            except OSError:
                pass
        if self._control_thread.is_alive():
            self._control_thread.join(timeout=2.0)
        if self._writer_thread.is_alive():
            try:
                self._queue.put(("shutdown", None, None), timeout=2.0)
            except queue.Full:
                LOGGER.error("Trajectory writer queue remained full during shutdown")
            self._writer_thread.join(timeout=5.0)
        try:
            if stat.S_ISSOCK(self.control_socket.lstat().st_mode):
                self.control_socket.unlink()
        except FileNotFoundError:
            pass
        self._write_status()
        LOGGER.info("Trajectory recorder stopped")


def send_control_command(
    command: str,
    *,
    control_socket: str | Path = DEFAULT_CONTROL_SOCKET,
    label: str = "",
    timeout_s: float = 5.0,
) -> dict[str, Any]:
    """Send one control command and return the recorder's JSON reply."""

    request: dict[str, Any] = {"command": command}
    if label:
        request["label"] = label
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout_s)
    try:
        client.connect(str(control_socket))
        client.sendall(_json_line(request).encode("utf-8"))
        client.shutdown(socket.SHUT_WR)
        chunks = bytearray()
        while len(chunks) <= _MAX_CONTROL_BYTES:
            data = client.recv(4096)
            if not data:
                break
            chunks.extend(data)
            if b"\n" in data:
                break
    except FileNotFoundError as exc:
        raise RecorderControlError(
            "trajectory recorder is not running; start run_robot.sh first"
        ) from exc
    except (ConnectionRefusedError, socket.timeout, OSError) as exc:
        raise RecorderControlError(f"cannot reach trajectory recorder: {exc}") from exc
    finally:
        client.close()
    line = bytes(chunks).split(b"\n", 1)[0]
    if not line:
        raise RecorderControlError("trajectory recorder returned an empty response")
    response = json.loads(line.decode("utf-8"))
    if not isinstance(response, dict):
        raise RecorderControlError("trajectory recorder returned invalid JSON")
    if not response.get("ok", False):
        raise RecorderControlError(str(response.get("error", "unknown recorder error")))
    return response
