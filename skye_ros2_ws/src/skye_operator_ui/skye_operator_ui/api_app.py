"""FastAPI application and WebSocket state stream."""

from __future__ import annotations

import asyncio
import logging
import os
import signal
import time
from contextlib import asynccontextmanager
from pathlib import Path
from typing import Any

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from skye_operator_ui.commands import command_allowed, validate_op
from skye_operator_ui.session_state import UiMode


logger = logging.getLogger(__name__)

PENDING_TIMEOUT_S = 8.0

_ALIGN_DONE = frozenset({"ALIGNED", "TIMEOUT_WARN"})

# Ops whose effect is observable in the state mailbox: pending clears as soon as
# the expected state lands, otherwise on PENDING_TIMEOUT_S so buttons unlock.
_PENDING_CONFIRMERS: dict[str, Any] = {
    "switch_sync": lambda m: m.get("teleop_state") in ("TELEOP_SYNCING", "SYNCED"),
    "switch_teleop": lambda m: m.get("teleop_state") == "TELEOP",
    "switch_stop": lambda m: m.get("teleop_state") not in ("TELEOP", "TELEOP_SYNCING"),
    "align_start": lambda m: m.get("align_status") == "ALIGNING"
    or m.get("align_status") in _ALIGN_DONE,
    "align_cancel": lambda m: m.get("align_status") != "ALIGNING",
    "takeover": lambda m: m.get("hitl_mode") == "HUMAN",
    "return": lambda m: m.get("hitl_mode") == "AUTONOMOUS",
    "recorder_start": lambda m: bool(m.get("recording_active")),
    "recorder_stop": lambda m: not m.get("recording_active"),
}


class PendingTracker:
    """Track the in-flight op, clearing it on state confirmation or timeout."""

    def __init__(self, timeout_s: float = PENDING_TIMEOUT_S) -> None:
        self.timeout_s = timeout_s
        self._op: str | None = None
        self._deadline = 0.0

    def set(self, op: str) -> None:
        # Ops without observable state feedback resolve on the dispatch ack alone.
        if op not in _PENDING_CONFIRMERS:
            self._op = None
            return
        self._op = op
        self._deadline = time.monotonic() + self.timeout_s

    def clear(self) -> None:
        self._op = None

    def resolve(self, mailbox: dict[str, Any] | None = None) -> str | None:
        op = self._op
        if op is None:
            return None
        confirmer = _PENDING_CONFIRMERS.get(op)
        if mailbox is not None and confirmer is not None and confirmer(mailbox):
            self._op = None
            return None
        if time.monotonic() >= self._deadline:
            self._op = None
            return None
        return op


class CommandBody(BaseModel):
    op: str


class SessionStartBody(BaseModel):
    profile: str
    mode: str


def _parse_mode(mode: str) -> UiMode | None:
    try:
        return UiMode[mode]
    except KeyError:
        return None


def _bridge_available(bridge: Any) -> bool:
    available = getattr(bridge, "available", None)
    if callable(available):
        return bool(available())
    return True


def _static_web_dir() -> Path | None:
    pkg_root = Path(__file__).resolve().parent.parent
    source_dist = pkg_root / "web" / "dist"
    if source_dist.is_dir():
        return source_dist

    try:
        from ament_index_python.packages import get_package_share_directory

        share_web = Path(get_package_share_directory("skye_operator_ui")) / "web"
        if share_web.is_dir():
            return share_web
    except Exception:
        pass
    return None


def create_app(
    supervisor,
    bridge,
    snapshot_builder,
    *,
    pending_timeout_s: float = PENDING_TIMEOUT_S,
) -> FastAPI:
    pending = PendingTracker(pending_timeout_s)

    def build_snapshot() -> dict[str, Any]:
        pending_op = pending.resolve(bridge.mailbox())
        return snapshot_builder.build(supervisor, bridge, pending_op)

    async def tick_loop() -> None:
        while True:
            try:
                supervisor.tick()
            except Exception:  # noqa: BLE001 - a bad tick must not kill the loop
                logger.exception("supervisor.tick failed")
            try:
                pending.resolve(bridge.mailbox())
            except Exception:  # noqa: BLE001
                logger.exception("pending resolve failed")
            await asyncio.sleep(0.1)

    @asynccontextmanager
    async def lifespan(app: FastAPI):
        task = asyncio.create_task(tick_loop())
        original_sigint = signal.getsignal(signal.SIGINT)

        def handle_shutdown(signum, frame) -> None:
            supervisor.stop()
            if callable(original_sigint) and original_sigint not in (
                signal.SIG_DFL,
                signal.SIG_IGN,
            ):
                original_sigint(signum, frame)

        # Only the main thread may install handlers; under a test client or an
        # embedded server thread we simply skip the orderly-shutdown hook.
        try:
            signal.signal(signal.SIGINT, handle_shutdown)
            installed = True
        except ValueError:
            installed = False

        try:
            yield
        finally:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            if installed:
                signal.signal(signal.SIGINT, original_sigint)

    app = FastAPI(lifespan=lifespan)

    @app.get("/api/snapshot")
    def get_snapshot():
        return build_snapshot()

    @app.post("/api/session/start")
    def session_start(body: SessionStartBody):
        mode = _parse_mode(body.mode)
        if mode is None:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "reason": f"无效模式: {body.mode}"},
            )

        ok, reason = supervisor.start(body.profile, mode)
        if not ok:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "reason": reason},
            )

        bridge.set_session_mode(mode)
        return {"ok": True}

    @app.post("/api/session/stop")
    def session_stop():
        ok, reason = supervisor.stop()
        if not ok:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "reason": reason},
            )
        bridge.set_session_mode(None)
        pending.clear()
        return {"ok": True}

    @app.post("/api/session/retry_step")
    def session_retry_step():
        ok, reason = supervisor.retry_step()
        if not ok:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "reason": reason},
            )
        return {"ok": True}

    @app.post("/api/session/cleanup_stale")
    def session_cleanup_stale():
        ok, reason = supervisor.run_cleanup_stale()
        if not ok:
            return JSONResponse(
                status_code=400,
                content={"ok": False, "reason": reason},
            )
        return {"ok": True, "message": reason}

    @app.post("/api/command")
    def post_command(body: CommandBody):
        op = body.op
        if not validate_op(op):
            return JSONResponse(
                status_code=400,
                content={"ok": False, "reason": "未知命令，已拒绝"},
            )

        mailbox = bridge.mailbox()
        if op == "emergency_stop":
            if not _bridge_available(bridge):
                return JSONResponse(
                    status_code=400,
                    content={"ok": False, "reason": "机器人连接不可用，无法急停"},
                )
        else:
            allowed, reason = command_allowed(
                op=op,
                session=supervisor.logic,
                teleop_state=mailbox.get("teleop_state"),
                hitl_mode=mailbox.get("hitl_mode"),
                align_status=mailbox.get("align_status"),
            )
            if not allowed:
                return JSONResponse(
                    status_code=400,
                    content={"ok": False, "reason": reason},
                )

        ok, reason = bridge.dispatch(op)
        if not ok:
            pending.clear()
            return JSONResponse(
                status_code=400,
                content={"ok": False, "reason": reason or "命令派发失败"},
            )
        pending.set(op)
        return {"ok": True}

    @app.get("/api/logs/{step_id}")
    def get_logs(step_id: str, n: int = 200):
        return {"lines": supervisor.step_logs(step_id, n=n)}

    @app.websocket("/ws/state")
    async def ws_state(websocket: WebSocket):
        await websocket.accept()
        try:
            while True:
                await websocket.send_json(build_snapshot())
                await asyncio.sleep(0.1)
        except WebSocketDisconnect:
            pass

    static_dir = _static_web_dir()
    if static_dir is not None:
        app.mount("/", StaticFiles(directory=str(static_dir), html=True), name="web")

    return app
