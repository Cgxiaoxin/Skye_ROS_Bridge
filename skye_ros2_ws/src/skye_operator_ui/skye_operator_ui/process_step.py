"""Single playbook step: subprocess group + bounded log ring."""

from __future__ import annotations

import os
import signal
import subprocess
import threading
import time
from collections import deque
from collections.abc import Callable

class LogRing:
    """Thread-safe bounded FIFO log buffer."""

    def __init__(self, maxlen: int = 2000) -> None:
        self._maxlen = maxlen
        self._lines: deque[str] = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def append(self, line: str) -> None:
        with self._lock:
            self._lines.append(line)

    def tail(self, n: int) -> list[str]:
        with self._lock:
            if n <= 0:
                return []
            return list(self._lines)[-n:]


class ProcessStep:
    """Run one external command as a new session/process group."""

    def __init__(
        self,
        step_id: str,
        argv: list[str],
        env: dict[str, str],
        health_check: Callable[[], bool],
        timeout_s: float,
        log_ring_size: int = 2000,
    ) -> None:
        self.step_id = step_id
        self.argv = argv
        self.env = env
        self.health_check = health_check
        self.timeout_s = timeout_s
        self.logs = LogRing(maxlen=log_ring_size)
        self._proc: subprocess.Popen[str] | None = None
        self._reader: threading.Thread | None = None

    def start(self) -> None:
        if self._proc is not None and self._proc.poll() is None:
            return
        merged_env = os.environ.copy()
        merged_env.update(self.env)
        self._proc = subprocess.Popen(
            self.argv,
            env=merged_env,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        self._reader = threading.Thread(
            target=self._read_stdout,
            name=f"process-step-{self.step_id}",
            daemon=True,
        )
        self._reader.start()

    def poll_health(self) -> bool:
        return self.health_check()

    def terminate(self, grace_s: float = 5.0) -> None:
        proc = self._proc
        if proc is None:
            return
        if proc.poll() is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                self._join_reader()
                return
            deadline = time.monotonic() + grace_s
            while time.monotonic() < deadline:
                if proc.poll() is not None:
                    break
                time.sleep(0.05)
            if proc.poll() is None:
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                else:
                    proc.wait(timeout=1.0)
        self._join_reader()

    def _join_reader(self) -> None:
        if self._reader is not None:
            self._reader.join(timeout=1.0)

    def _read_stdout(self) -> None:
        proc = self._proc
        if proc is None or proc.stdout is None:
            return
        for line in proc.stdout:
            self.logs.append(line.rstrip("\n"))
        proc.stdout.close()
