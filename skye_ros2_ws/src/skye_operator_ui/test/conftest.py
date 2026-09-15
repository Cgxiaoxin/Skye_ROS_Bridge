import pytest

from skye_operator_ui.session_state import SessionLogic, SessionState
from skye_operator_ui.snapshot import SnapshotBuilder


@pytest.fixture
def fake_stack():
    logic = SessionLogic()

    class FakeSupervisor:
        def __init__(self) -> None:
            self.logic = logic

        def start(self, profile, mode):
            if not self.logic.begin_start(profile, mode):
                return False, "无法启动：当前状态不允许开始会话"
            if not self.logic.precheck_ok():
                return False, "预检状态异常"
            return True, ""

        def stop(self):
            if self.logic.state() == SessionState.IDLE:
                return False, "当前无活跃会话"
            if not self.logic.begin_stop():
                return False, "无法结束会话"
            if not self.logic.mark_idle():
                return False, "结束会话状态异常"
            return True, ""

        def retry_step(self):
            if not self.logic.resume_starting():
                return False, "无法恢复启动状态"
            return True, ""

        def tick(self) -> None:
            pass

        def snapshot_fields(self):
            mode = self.logic.mode()
            return {
                "state": self.logic.state().name,
                "profile": self.logic.profile(),
                "mode": mode.name if mode else None,
                "step": None,
            }

        def run_cleanup_stale(self):
            return True, "清理完成"

        def step_logs(self, step_id, n=200):
            return [f"log:{step_id}"]

    class FakeBridge:
        def __init__(self) -> None:
            self._mode = None
            self.dispatched: list[str] = []

        def available(self) -> bool:
            return True

        def set_session_mode(self, mode) -> None:
            self._mode = mode

        def mailbox(self):
            return {
                "teleop_state": "SYNCED",
                "align_status": "IDLE",
                "hitl_mode": None,
                "hitl_source": None,
                "left_joints": [0.0] * 7,
                "right_joints": [0.0] * 7,
                "left_gripper": 0.0,
                "right_gripper": 0.0,
                "health": {"driver": True},
                "recording_active": False,
            }

        def dispatch(self, op: str):
            self.dispatched.append(op)
            return True, ""

    return FakeSupervisor(), FakeBridge(), SnapshotBuilder()
