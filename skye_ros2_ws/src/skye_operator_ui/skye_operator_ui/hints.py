"""Chinese next-step hints for the operator UI."""

from __future__ import annotations

from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode


def next_hint(
    *,
    session: SessionLogic,
    teleop_state: str | None,
    hitl_mode: str | None,
    align_status: str | None,
) -> str:
    state = session.state()
    mode = session.mode()

    if state == SessionState.IDLE:
        return "请选择机台（THOR / ORIN）和模式，然后启动会话"

    if state == SessionState.PRECHECK:
        return "正在进行预检，请稍候…"

    if state == SessionState.STARTING:
        return "正在启动子系统，请等待全部步骤就绪"

    if state == SessionState.FAILED:
        return "启动失败：可重试当前步骤或结束会话"

    if state == SessionState.STOPPING:
        return "正在有序结束会话，请勿操作"

    if state == SessionState.DEGRADED:
        return "系统降级：禁止新开录制，请检查健康状态后结束会话"

    if mode == UiMode.dagger:
        if hitl_mode == "AUTONOMOUS":
            return "策略自主运行中，需要干预时请先点「同步」"
        if hitl_mode == "HUMAN":
            return "人工控制中，可交还控制权或开始/停止录制"
        if hitl_mode == "HANDOVER_SYNC":
            if teleop_state == "SYNCED":
                return "同步已完成，确认姿态安全后点击「进入遥操」"
            return "接管同步中：小臂正在跟大臂，请等待 SYNCED"
        return "等待控制模式就绪，观察 /skye/control_mode"

    if mode == UiMode.teleop_record:
        if teleop_state in (None, "IDLE"):
            return "请点击「同步」将小臂对齐到大臂"
        if teleop_state == "TELEOP_SYNCING":
            return "小臂同步中，请保持静止"
        if teleop_state == "SYNCED":
            if align_status in (None, "IDLE"):
                return "同步完成，可开始 follower 对齐"
            if align_status == "ALIGNING":
                return "follower 对齐中，请稍候"
            if align_status == "TIMEOUT_WARN":
                return "对齐超时警告，仍可尝试开启遥操"
        if teleop_state == "SYNCED" and align_status in ("ALIGNED", "TIMEOUT_WARN"):
            return "对齐完成，可开启遥操"
        if teleop_state == "TELEOP":
            return "遥操已开启，可开始/停止数采录制"
        return "等待遥操状态更新"

    return "会话就绪，请按左侧步骤操作"
