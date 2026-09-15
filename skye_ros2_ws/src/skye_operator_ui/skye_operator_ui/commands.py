"""Command whitelist and session-aware permission checks."""

from __future__ import annotations

from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode

ALLOWED_OPS: frozenset[str] = frozenset(
    {
        "switch_sync",
        "switch_teleop",
        "switch_stop",
        "align_start",
        "align_cancel",
        "recorder_start",
        "recorder_stop",
        "takeover",
        "return",
        "emergency_stop",
        "hold_current",
        "stop_motion",
    }
)

_TELEOP_OPS = frozenset(
    {
        "switch_sync",
        "switch_teleop",
        "switch_stop",
        "align_start",
        "align_cancel",
    }
)

_DAGGER_OPS = frozenset({"takeover", "return"})

_COMMON_OPS = frozenset(
    {
        "recorder_start",
        "recorder_stop",
        "hold_current",
        "stop_motion",
    }
)

_ALIGN_DONE = frozenset({"ALIGNED", "TIMEOUT_WARN"})

_OPERATIONAL_STATES = frozenset(
    {SessionState.READY, SessionState.RUNNING, SessionState.DEGRADED}
)


def validate_op(op: str) -> bool:
    return op in ALLOWED_OPS


def _operational(session: SessionLogic) -> bool:
    return session.state() in _OPERATIONAL_STATES


def command_allowed(
    *,
    op: str,
    session: SessionLogic,
    teleop_state: str | None,
    hitl_mode: str | None,
    align_status: str | None,
) -> tuple[bool, str]:
    if not validate_op(op):
        return False, "未知命令，已拒绝"

    state = session.state()

    if op == "emergency_stop":
        return True, ""

    if not _operational(session):
        if state == SessionState.STOPPING:
            return False, "会话正在结束，请稍候"
        if state == SessionState.FAILED:
            return False, "会话启动失败，请重试或结束会话"
        return False, "会话尚未就绪，请等待启动完成"

    mode = session.mode()
    if mode is None:
        return False, "会话模式未知"

    if op in _TELEOP_OPS:
        if mode != UiMode.teleop_record:
            return False, "当前为 DAgger 模式，无法执行遥操命令"
    elif op in _DAGGER_OPS:
        if mode != UiMode.dagger:
            return False, "当前为遥操数采模式，无法执行接管/交还"

    if op == "return":
        if hitl_mode != "HUMAN":
            return False, "仅在人工控制（HUMAN）时可交还"
        return True, ""

    if op == "takeover":
        if hitl_mode != "AUTONOMOUS":
            return False, "仅在自主模式（AUTONOMOUS）时可接管"
        return True, ""

    if op == "switch_teleop":
        synced = teleop_state == "SYNCED"
        align_done = align_status in _ALIGN_DONE
        if not synced and not align_done:
            return False, "请先完成小臂同步或 follower 对齐"
        return True, ""

    if op == "align_start":
        if align_status == "ALIGNING":
            return False, "对齐正在进行中"
        if teleop_state != "SYNCED":
            return False, "请先完成小臂同步（SYNCED）"
        return True, ""

    if op == "align_cancel":
        if align_status != "ALIGNING":
            return False, "当前未在对齐中"
        return True, ""

    if op == "switch_sync":
        if teleop_state in ("TELEOP_SYNCING", "SYNCED", "TELEOP"):
            return False, "遥操状态不允许再次同步"
        return True, ""

    if op == "recorder_start" and state == SessionState.DEGRADED:
        return False, "系统降级，禁止新开录制"

    if op in _COMMON_OPS or op in _TELEOP_OPS or op in _DAGGER_OPS:
        return True, ""

    return False, "命令当前不可用"
