from skye_operator_ui.session_state import SessionState, UiMode
from skye_operator_ui.supervisor import SessionSupervisor


def _mock_cfg(step_id: str = "a", health_key: str = "a", timeout_s: float = 3.0):
    return {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": [],
        "precheck": {"skip_ping": True},
        "playbook_override": {
            "teleop_record": [
                {
                    "id": step_id,
                    "argv": ["/bin/sleep", "60"],
                    "health_key": health_key,
                    "timeout_s": timeout_s,
                    "optional": False,
                },
            ]
        },
    }


def _make_supervisor(tmp_path, cfg, healthy: dict):
    def health(key):
        return healthy.get(key, False)

    return SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=health,
        precheck_fn=lambda: (True, "ok"),
    )


def test_supervisor_mock_playbook(tmp_path, monkeypatch):
    healthy = {"a": False}
    sup = _make_supervisor(tmp_path, _mock_cfg(), healthy)
    ok, _ = sup.start("thor", UiMode.teleop_record)
    assert ok
    # still starting
    for _ in range(5):
        sup.tick()
    healthy["a"] = True
    for _ in range(20):
        sup.tick()
        if sup.logic.state() == SessionState.READY:
            break
    assert sup.logic.state() == SessionState.READY
    ok, _ = sup.stop()
    assert ok
    assert sup.logic.state() == SessionState.IDLE


def test_step_timeout_marks_failed(tmp_path, monkeypatch):
    healthy = {"a": False}
    sup = _make_supervisor(tmp_path, _mock_cfg(timeout_s=3.0), healthy)
    ok, _ = sup.start("thor", UiMode.teleop_record)
    assert ok

    t = 0.0

    def fake_monotonic():
        return t

    monkeypatch.setattr("skye_operator_ui.supervisor.time.monotonic", fake_monotonic)

    sup.tick()
    assert sup.logic.state() == SessionState.STARTING

    t = 3.1
    sup.tick()
    assert sup.logic.state() == SessionState.FAILED


def test_ready_unhealthy_driver_degraded(tmp_path):
    healthy = {"driver": True}
    sup = _make_supervisor(
        tmp_path,
        _mock_cfg(step_id="driver", health_key="driver"),
        healthy,
    )
    ok, _ = sup.start("thor", UiMode.teleop_record)
    assert ok

    for _ in range(20):
        sup.tick()
        if sup.logic.state() == SessionState.READY:
            break

    assert sup.logic.state() == SessionState.READY

    healthy["driver"] = False
    sup.tick()
    assert sup.logic.state() == SessionState.DEGRADED


def test_retry_step_recovers_after_failed(tmp_path, monkeypatch):
    healthy = {"a": False}
    sup = _make_supervisor(tmp_path, _mock_cfg(timeout_s=3.0), healthy)
    ok, _ = sup.start("thor", UiMode.teleop_record)
    assert ok

    t = 0.0

    def fake_monotonic():
        return t

    monkeypatch.setattr("skye_operator_ui.supervisor.time.monotonic", fake_monotonic)

    sup.tick()
    t = 3.1
    sup.tick()
    assert sup.logic.state() == SessionState.FAILED

    ok, _ = sup.retry_step()
    assert ok
    assert sup.logic.state() == SessionState.STARTING
    assert sup._step_deadline is None

    healthy["a"] = True
    sup.tick()
    sup.tick()
    assert sup.logic.state() == SessionState.READY
