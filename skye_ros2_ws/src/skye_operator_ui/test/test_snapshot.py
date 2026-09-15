class FakeBridge:
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


def test_snapshot_contains_hint_and_session(monkeypatch):
    from skye_operator_ui.snapshot import SnapshotBuilder
    from skye_operator_ui.session_state import SessionLogic, UiMode

    logic = SessionLogic()
    logic.begin_start("thor", UiMode.teleop_record)
    logic.precheck_ok()
    logic.mark_ready()

    class FakeSup:
        def snapshot_fields(self):
            return {
                "state": logic.state().name,
                "profile": "thor",
                "mode": "teleop_record",
                "step": None,
            }

    FakeSup.logic = logic

    snap = SnapshotBuilder().build(FakeSup(), FakeBridge(), pending_op=None)
    assert snap["session"]["profile"] == "thor"
    assert "next_hint" in snap
    assert snap["teleop"]["state"] == "SYNCED"


def test_dispatch_plan_mode_and_recorder_routing():
    from skye_operator_ui.ros_bridge import dispatch_plan
    from skye_operator_ui.session_state import UiMode

    assert dispatch_plan("switch_sync", UiMode.teleop_record) == (
        "string",
        "/mode/switch_sync",
        "switch_sync",
    )
    assert dispatch_plan("recorder_start", UiMode.teleop_record) == (
        "trigger",
        "/skye/data_recorder/start",
        "",
    )
    assert dispatch_plan("recorder_start", UiMode.dagger) == (
        "trigger",
        "/skye/recorder/start",
        "",
    )
    assert dispatch_plan("joint_control", UiMode.teleop_record) is None
