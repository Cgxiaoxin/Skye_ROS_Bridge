"""Aggregate supervisor + ROS mailbox into UI snapshots."""

from __future__ import annotations

from typing import Any, Protocol

from skye_operator_ui.hints import next_hint
from skye_operator_ui.session_state import SessionState


class _SupervisorLike(Protocol):
    logic: Any

    def snapshot_fields(self) -> dict[str, Any]:
        ...


class _BridgeLike(Protocol):
    def mailbox(self) -> dict[str, Any]:
        ...


class SnapshotBuilder:
    """Build spec §5.4 snapshots for WebSocket / HTTP."""

    def build(
        self,
        supervisor: _SupervisorLike,
        bridge: _BridgeLike,
        pending_op: str | None,
    ) -> dict[str, Any]:
        session = supervisor.snapshot_fields()
        mailbox = bridge.mailbox()
        logic = supervisor.logic

        teleop_state = mailbox.get("teleop_state")
        align_status = mailbox.get("align_status")
        hitl_mode = mailbox.get("hitl_mode")
        hitl_source = mailbox.get("hitl_source")
        health_map = mailbox.get("health") or {}

        hint = next_hint(
            session=logic,
            teleop_state=teleop_state,
            hitl_mode=hitl_mode,
            align_status=align_status,
        )

        return {
            "session": session,
            "teleop": {"state": teleop_state},
            "align": {"status": align_status},
            "hitl": {"mode": hitl_mode, "source": hitl_source},
            "recording": {"active": bool(mailbox.get("recording_active"))},
            "arms": {
                "left": list(mailbox.get("left_joints") or []),
                "right": list(mailbox.get("right_joints") or []),
            },
            "grippers": {
                "left": mailbox.get("left_gripper"),
                "right": mailbox.get("right_gripper"),
            },
            "health": [
                {"key": key, "ok": bool(ok)} for key, ok in sorted(health_map.items())
            ],
            "banner": self._banner(
                session_state=session.get("state"),
                pending_op=pending_op,
            ),
            "next_hint": hint,
            "pending_op": pending_op,
        }

    def _banner(
        self,
        *,
        session_state: str | None,
        pending_op: str | None,
    ) -> dict[str, Any] | None:
        if pending_op:
            return {
                "level": "amber",
                "text": f"等待命令确认：{pending_op}",
            }
        if session_state == SessionState.DEGRADED.name:
            return {
                "level": "red",
                "text": "系统降级：禁止新开录制，请检查健康状态",
            }
        if session_state == SessionState.FAILED.name:
            return {
                "level": "red",
                "text": "会话启动失败，可重试当前步骤或结束会话",
            }
        return None
