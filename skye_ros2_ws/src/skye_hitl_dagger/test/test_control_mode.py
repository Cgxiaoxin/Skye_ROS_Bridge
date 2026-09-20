from skye_hitl_dagger.control_mode import ControlArbiterLogic, ControlModeState


def test_starts_autonomous():
    logic = ControlArbiterLogic()
    assert logic.mode() == ControlModeState.AUTONOMOUS
    assert logic.active_source() == "policy"
    assert logic.teleop_requested() is False


def test_takeover_stays_in_handover_until_enter_teleop():
    logic = ControlArbiterLogic()
    assert logic.request_takeover() is True
    assert logic.mode() == ControlModeState.HANDOVER_SYNC
    assert logic.active_source() == "hold"
    assert logic.teleop_requested() is False
    # Must not complete to HUMAN without enter_teleop.
    assert logic.sync_completed() is False
    assert logic.mode() == ControlModeState.HANDOVER_SYNC


def test_enter_teleop_then_sync_completed_reaches_human():
    logic = ControlArbiterLogic()
    logic.request_takeover()
    assert logic.request_enter_teleop() is True
    assert logic.teleop_requested() is True
    assert logic.sync_completed() is True
    assert logic.mode() == ControlModeState.HUMAN
    assert logic.active_source() == "teleop"
    assert logic.teleop_requested() is False


def test_enter_teleop_only_from_handover():
    logic = ControlArbiterLogic()
    assert logic.request_enter_teleop() is False
    logic.request_takeover()
    assert logic.request_enter_teleop() is True
    assert logic.request_enter_teleop() is False  # already requested


def test_return_only_from_human():
    logic = ControlArbiterLogic()
    assert logic.request_return() is False
    logic.request_takeover()
    logic.request_enter_teleop()
    logic.sync_completed()
    assert logic.request_return() is True
    assert logic.mode() == ControlModeState.AUTONOMOUS


def test_return_aborts_handover_sync():
    logic = ControlArbiterLogic()
    logic.request_takeover()
    assert logic.request_return() is True
    assert logic.mode() == ControlModeState.AUTONOMOUS
    assert logic.teleop_requested() is False


def test_takeover_ignored_when_not_autonomous():
    logic = ControlArbiterLogic()
    logic.request_takeover()
    assert logic.request_takeover() is False
