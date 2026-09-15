from skye_operator_ui.commands import ALLOWED_OPS, command_allowed, validate_op
from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode


def test_allowed_ops_exact_set():
    assert ALLOWED_OPS == frozenset(
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


def test_unknown_op_rejected():
    assert not validate_op("joint_control")
    assert validate_op("emergency_stop")


def test_return_only_in_human():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    s.precheck_ok()
    s.mark_ready()
    ok, _ = command_allowed(
        op="return",
        session=s,
        teleop_state=None,
        hitl_mode="AUTONOMOUS",
        align_status=None,
    )
    assert not ok
    ok, _ = command_allowed(
        op="return",
        session=s,
        teleop_state=None,
        hitl_mode="HUMAN",
        align_status=None,
    )
    assert ok


def test_takeover_only_autonomous():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    s.precheck_ok()
    s.mark_ready()
    ok, _ = command_allowed(
        op="takeover",
        session=s,
        teleop_state=None,
        hitl_mode="HUMAN",
        align_status=None,
    )
    assert not ok
    ok, _ = command_allowed(
        op="takeover",
        session=s,
        teleop_state=None,
        hitl_mode="AUTONOMOUS",
        align_status=None,
    )
    assert ok


def test_emergency_stop_only_when_session_active():
    s = SessionLogic()
    ok, _ = command_allowed(
        op="emergency_stop",
        session=s,
        teleop_state=None,
        hitl_mode=None,
        align_status=None,
    )
    assert not ok
    s.begin_start("thor", UiMode.teleop_record)
    ok, _ = command_allowed(
        op="emergency_stop",
        session=s,
        teleop_state=None,
        hitl_mode=None,
        align_status=None,
    )
    assert ok


def test_switch_teleop_requires_synced_or_align_done():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_ok()
    s.mark_ready()
    ok, _ = command_allowed(
        op="switch_teleop",
        session=s,
        teleop_state="IDLE",
        hitl_mode=None,
        align_status="IDLE",
    )
    assert not ok
    ok, _ = command_allowed(
        op="switch_teleop",
        session=s,
        teleop_state="SYNCED",
        hitl_mode=None,
        align_status="IDLE",
    )
    assert ok
    ok, _ = command_allowed(
        op="switch_teleop",
        session=s,
        teleop_state="IDLE",
        hitl_mode=None,
        align_status="ALIGNED",
    )
    assert ok


def test_teleop_ops_rejected_in_dagger_mode():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    s.precheck_ok()
    s.mark_ready()
    ok, _ = command_allowed(
        op="switch_sync",
        session=s,
        teleop_state="SYNCED",
        hitl_mode="AUTONOMOUS",
        align_status=None,
    )
    assert not ok


def test_commands_rejected_before_ready():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    ok, _ = command_allowed(
        op="switch_sync",
        session=s,
        teleop_state="SYNCED",
        hitl_mode=None,
        align_status=None,
    )
    assert not ok
    assert s.state() == SessionState.PRECHECK


def test_recorder_start_rejected_in_degraded_recorder_stop_allowed():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_ok()
    s.mark_ready()
    s.mark_degraded()
    assert s.state() == SessionState.DEGRADED

    ok, reason = command_allowed(
        op="recorder_start",
        session=s,
        teleop_state="TELEOP",
        hitl_mode=None,
        align_status="ALIGNED",
    )
    assert not ok
    assert reason == "系统降级，禁止新开录制"

    ok, reason = command_allowed(
        op="recorder_stop",
        session=s,
        teleop_state="TELEOP",
        hitl_mode=None,
        align_status="ALIGNED",
    )
    assert ok
    assert reason == ""
