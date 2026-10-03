"""RosBridge unit tests that do not need a live ROS graph."""

import subprocess
import threading
import time
from types import SimpleNamespace

from skye_operator_ui.leader_arms import LeaderArmGate
from skye_operator_ui.ros_bridge import RosBridge, run_leader_arm_gate
from skye_operator_ui.session_state import UiMode


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
    bridge._leader_gate = LeaderArmGate()
    bridge._repo_root = "/repo"
    bridge._leader_gate_container = "skye_marvin_m6"
    bridge._leader_gate_image = "marvin-m6-ros2:e5a9d8fd"
    bridge._leader_gate_timeout_s = 15.0
    bridge._script_runner = lambda *args, **kwargs: (True, "")
    bridge._dispatch_lock = threading.Lock()
    return bridge


def test_run_leader_arm_gate_invokes_script_with_container_env(monkeypatch):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append((argv, kwargs))
        return subprocess.CompletedProcess(argv, 0, stdout="", stderr="")

    monkeypatch.setattr(subprocess, "run", fake_run)

    ok, reason = run_leader_arm_gate(
        "/repo",
        "left",
        "off",
        container="marvin",
        timeout_s=3.0,
        image="marvin-m6-ros2:test",
    )

    assert ok is True
    assert reason == ""
    argv, kwargs = calls[0]
    assert argv == ["/repo/scripts/leader_arm_gate.sh", "left", "off"]
    assert kwargs["timeout"] == 3.0
    assert kwargs["env"]["MARVIN_CONTAINER_NAME"] == "marvin"
    assert kwargs["env"]["MARVIN_IMAGE"] == "marvin-m6-ros2:test"


def test_run_leader_arm_gate_returns_error_message(monkeypatch):
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0], 2, stdout="out", stderr="bad"
        ),
    )

    ok, reason = run_leader_arm_gate(
        "/repo",
        "right",
        "on",
        container="marvin",
    )

    assert ok is False
    assert reason == "bad"


def test_run_leader_arm_gate_translates_missing_container(monkeypatch):
    monkeypatch.setattr(
        subprocess,
        "run",
        lambda *args, **kwargs: subprocess.CompletedProcess(
            args[0],
            1,
            stdout="",
            stderr="Error response from daemon: No such container: skye_marvin_m6",
        ),
    )

    ok, reason = run_leader_arm_gate(
        "/repo",
        "left",
        "off",
        container="skye_marvin_m6",
    )

    assert ok is False
    assert "未运行" in reason


def test_run_leader_arm_gate_timeout_returns_false(monkeypatch):
    def fake_run(*args, **kwargs):
        raise subprocess.TimeoutExpired(cmd=args[0], timeout=kwargs["timeout"])

    monkeypatch.setattr(subprocess, "run", fake_run)

    ok, reason = run_leader_arm_gate(
        "/repo",
        "left",
        "on",
        container="marvin",
        timeout_s=1.5,
    )

    assert ok is False
    assert "超时" in reason


def test_run_leader_arm_gate_missing_script_returns_false(monkeypatch):
    def fake_run(*args, **kwargs):
        raise FileNotFoundError(2, "No such file or directory", args[0][0])

    monkeypatch.setattr(subprocess, "run", fake_run)

    ok, reason = run_leader_arm_gate(
        "/repo",
        "right",
        "off",
        container="marvin",
    )

    assert ok is False
    assert "找不到" in reason


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


def test_dispatch_leader_script_flips_gate_only_on_success():
    bridge = _bare_bridge()
    bridge._ui_mode = UiMode.teleop_record
    calls = []

    def runner(repo_root, side, action, *, container, timeout_s, image=None):
        calls.append((repo_root, side, action, container, timeout_s, image))
        return True, ""

    bridge._script_runner = runner

    ok, reason = bridge.dispatch("leader_left_off")

    assert ok is True
    assert reason == ""
    assert calls == [
        ("/repo", "left", "off", "skye_marvin_m6", 15.0, "marvin-m6-ros2:e5a9d8fd")
    ]
    assert bridge._leader_gate.snapshot()["left_enabled"] is False


def test_dispatch_leader_script_failure_does_not_flip_gate():
    bridge = _bare_bridge()
    bridge._ui_mode = UiMode.teleop_record
    bridge._script_runner = lambda *args, **kwargs: (False, "script failed")

    ok, reason = bridge.dispatch("leader_right_off")

    assert ok is False
    assert reason == "script failed"
    assert bridge._leader_gate.snapshot()["right_enabled"] is True


def test_dispatch_rejects_nested_command_while_script_in_flight():
    bridge = _bare_bridge()
    bridge._ui_mode = UiMode.teleop_record
    nested_result = None
    bridge._publish_string = lambda topic, data: (True, "")

    def runner(*args, **kwargs):
        nonlocal nested_result
        nested_result = bridge.dispatch("switch_stop")
        return True, ""

    bridge._script_runner = runner

    ok, reason = bridge.dispatch("leader_left_off")

    assert ok is True
    assert reason == ""
    assert nested_result == (False, "命令正在执行，请稍候")


def test_dispatch_align_start_uses_gate_payload_and_rejects_none():
    bridge = _bare_bridge()
    bridge._ui_mode = UiMode.teleop_record
    published = []
    bridge._publish_string = lambda topic, data: published.append((topic, data)) or (True, "")

    bridge._leader_gate.set_enabled("right", False)
    assert bridge.dispatch("align_start") == (True, "")
    assert published == [("/mode/align_follower", "align_follower_left")]

    bridge._leader_gate.set_enabled("left", False)
    ok, reason = bridge.dispatch("align_start")
    assert ok is False
    assert "两侧小臂已关闭" in reason
    assert published == [("/mode/align_follower", "align_follower_left")]


def test_dispatch_switch_sync_and_teleop_callback_lock_gate():
    bridge = _bare_bridge()
    bridge._ui_mode = UiMode.teleop_record
    bridge._publish_string = lambda topic, data: (True, "")

    assert bridge.dispatch("switch_sync") == (True, "")
    assert bridge._leader_gate.snapshot()["locked"] is True

    bridge._leader_gate.reset()
    bridge._teleop_state_callback(SimpleNamespace(data="SYNCED"))
    assert bridge._leader_gate.snapshot()["locked"] is True
