"""RosBridge unit tests that do not need a live ROS graph."""

import time

from skye_operator_ui.ros_bridge import RosBridge


class FakeFuture:
    def __init__(self, done_after: float) -> None:
        self._ready_at = time.monotonic() + done_after
        self.cancelled = False

    def done(self) -> bool:
        return time.monotonic() >= self._ready_at

    def cancel(self) -> None:
        self.cancelled = True


def _bare_bridge() -> RosBridge:
    bridge = object.__new__(RosBridge)
    bridge._ui_mode = None
    bridge._teleop_state = "TELEOP"
    bridge._align_status = "ALIGNED"
    bridge._hitl_mode = "HUMAN"
    bridge._hitl_source = "operator"
    bridge._align_stamp = time.monotonic()
    bridge._control_mode_stamp = time.monotonic()
    bridge._recording_active = True
    return bridge


def test_await_future_returns_true_when_future_completes():
    future = FakeFuture(done_after=0.02)
    assert RosBridge._await_future(future, 1.0) is True


def test_await_future_times_out():
    future = FakeFuture(done_after=10.0)
    start = time.monotonic()
    assert RosBridge._await_future(future, 0.05) is False
    assert time.monotonic() - start < 1.0


def test_clear_session_cache_resets_latched_state():
    bridge = _bare_bridge()
    bridge.clear_session_cache()

    assert bridge._teleop_state is None
    assert bridge._align_status is None
    assert bridge._hitl_mode is None
    assert bridge._hitl_source is None
    assert bridge._align_stamp is None
    assert bridge._control_mode_stamp is None
    assert bridge._recording_active is False


def test_set_session_mode_none_clears_cache():
    bridge = _bare_bridge()
    bridge.set_session_mode(None)
    assert bridge._teleop_state is None
    assert bridge._recording_active is False


def test_stop_recorder_if_active_noop_when_not_recording():
    bridge = _bare_bridge()
    bridge._recording_active = False
    calls = []
    bridge.dispatch = lambda op: calls.append(op) or (True, "")

    assert bridge.stop_recorder_if_active() == (True, "")
    assert calls == []


def test_stop_recorder_if_active_dispatches_stop():
    bridge = _bare_bridge()
    calls = []
    bridge.dispatch = lambda op: (calls.append(op), (True, ""))[1]

    ok, _ = bridge.stop_recorder_if_active()
    assert ok is True
    assert calls == ["recorder_stop"]


def test_stop_recorder_if_active_swallows_errors():
    bridge = _bare_bridge()

    def boom(op):
        raise RuntimeError("no service")

    bridge.dispatch = boom
    ok, reason = bridge.stop_recorder_if_active()
    assert ok is False
    assert "no service" in reason
