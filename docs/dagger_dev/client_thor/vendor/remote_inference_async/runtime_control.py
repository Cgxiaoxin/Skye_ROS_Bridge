"""Local control channel for the persistent robot inference runtime."""

from __future__ import annotations

import json
import os
import queue
import socket
import stat
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEFAULT_RUNTIME_CONTROL_SOCKET = Path("/tmp/gento_robot_runtime.sock")
_MAX_REQUEST_BYTES = 64 * 1024
_VALID_COMMANDS = {"start_policy", "switch_policy", "pause", "shutdown"}


class RuntimeControlError(RuntimeError):
    """Raised when a runtime control request cannot be delivered."""


@dataclass(frozen=True, slots=True)
class RuntimeCommand:
    name: str
    task: str | None = None


class RuntimeControlServer:
    """Receive small JSON commands without touching the real-time control loop."""

    def __init__(self, control_socket: str | Path) -> None:
        self.control_socket = Path(control_socket)
        self._commands: queue.Queue[RuntimeCommand] = queue.Queue(maxsize=32)
        self._status_lock = threading.Lock()
        self._status: dict[str, Any] = {
            "state": "starting",
            "ready": False,
            "task": "",
            "generation": 0,
        }
        self._stop = threading.Event()
        self._server: socket.socket | None = None
        self._thread = threading.Thread(
            target=self._control_loop,
            name="runtime-control-socket",
            daemon=True,
        )
        self._started = False

    def start(self) -> None:
        if self._started:
            return
        self._prepare_socket()
        self._thread.start()
        self._started = True

    def _prepare_socket(self) -> None:
        self.control_socket.parent.mkdir(parents=True, exist_ok=True)
        try:
            existing_mode = self.control_socket.lstat().st_mode
        except FileNotFoundError:
            existing_mode = None
        if existing_mode is not None:
            if not stat.S_ISSOCK(existing_mode):
                raise RuntimeError(
                    f"runtime control path exists and is not a socket: "
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
                    f"another robot runtime is listening at {self.control_socket}"
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

    def update_status(self, **values: Any) -> None:
        with self._status_lock:
            self._status.update(values)

    def status(self) -> dict[str, Any]:
        with self._status_lock:
            return dict(self._status)

    def drain_commands(self) -> list[RuntimeCommand]:
        commands: list[RuntimeCommand] = []
        while True:
            try:
                commands.append(self._commands.get_nowait())
            except queue.Empty:
                return commands

    def _handle_request(self, request: dict[str, Any]) -> dict[str, Any]:
        command = request.get("command")
        if command == "status":
            return {"ok": True, **self.status()}
        if command not in _VALID_COMMANDS:
            return {
                "ok": False,
                "error": "command must be status, start_policy, switch_policy, pause, or shutdown",
            }
        task_value = request.get("task")
        task = None if task_value is None else str(task_value).strip()
        if command == "switch_policy" and not task:
            return {"ok": False, "error": "switch_policy requires a non-empty task"}
        try:
            self._commands.put_nowait(RuntimeCommand(name=command, task=task))
        except queue.Full:
            return {"ok": False, "error": "runtime command queue is full"}
        return {"ok": True, "accepted": command, **self.status()}

    def _control_loop(self) -> None:
        assert self._server is not None
        while not self._stop.is_set():
            try:
                connection, _address = self._server.accept()
            except socket.timeout:
                continue
            except OSError:
                if self._stop.is_set():
                    return
                continue
            with connection:
                connection.settimeout(1.0)
                try:
                    payload = connection.recv(_MAX_REQUEST_BYTES)
                    request = json.loads(payload.decode("utf-8"))
                    if not isinstance(request, dict):
                        raise ValueError("request must be a JSON object")
                    response = self._handle_request(request)
                except Exception as exc:
                    response = {"ok": False, "error": str(exc)}
                try:
                    connection.sendall(
                        json.dumps(response, ensure_ascii=False).encode("utf-8") + b"\n"
                    )
                except OSError:
                    pass

    def close(self) -> None:
        self._stop.set()
        if self._server is not None:
            self._server.close()
        if self._started:
            self._thread.join(timeout=2.0)
        try:
            if stat.S_ISSOCK(self.control_socket.lstat().st_mode):
                self.control_socket.unlink()
        except FileNotFoundError:
            pass


def send_runtime_command(
    command: str,
    *,
    control_socket: str | Path = DEFAULT_RUNTIME_CONTROL_SOCKET,
    task: str | None = None,
    timeout_s: float = 2.0,
) -> dict[str, Any]:
    request: dict[str, Any] = {"command": command}
    if task is not None:
        request["task"] = task
    client = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    client.settimeout(timeout_s)
    try:
        client.connect(str(control_socket))
        client.sendall(json.dumps(request, ensure_ascii=False).encode("utf-8") + b"\n")
        payload = client.recv(_MAX_REQUEST_BYTES)
    except OSError as exc:
        raise RuntimeControlError(
            f"cannot contact robot runtime at {control_socket}: {exc}"
        ) from exc
    finally:
        client.close()
    try:
        response = json.loads(payload.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise RuntimeControlError("robot runtime returned invalid JSON") from exc
    if not isinstance(response, dict):
        raise RuntimeControlError("robot runtime returned a non-object response")
    if not response.get("ok"):
        raise RuntimeControlError(str(response.get("error", "runtime command failed")))
    return response
