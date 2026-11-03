import subprocess

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


def _make_supervisor(tmp_path, cfg, healthy: dict, **kwargs):
    def health(key):
        return healthy.get(key, False)

    return SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=health,
        precheck_fn=lambda: (True, "ok"),
        **kwargs,
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


def test_degrade_invokes_callback_once(tmp_path):
    healthy = {"driver": True}
    calls = []
    sup = _make_supervisor(
        tmp_path,
        _mock_cfg(step_id="driver", health_key="driver"),
        healthy,
        on_degraded=lambda: calls.append("stop"),
    )
    sup.start("thor", UiMode.teleop_record)
    for _ in range(20):
        sup.tick()
        if sup.logic.state() == SessionState.READY:
            break

    healthy["driver"] = False
    sup.tick()
    sup.tick()
    assert sup.logic.state() == SessionState.DEGRADED
    assert calls == ["stop"]


def test_on_before_stop_failure_does_not_block_stop(tmp_path):
    healthy = {"driver": True}

    def boom():
        raise RuntimeError("recorder stop failed")

    sup = _make_supervisor(
        tmp_path,
        _mock_cfg(step_id="driver", health_key="driver"),
        healthy,
        on_before_stop=boom,
    )
    sup.start("thor", UiMode.teleop_record)
    for _ in range(20):
        sup.tick()
        if sup.logic.state() == SessionState.READY:
            break

    ok, _ = sup.stop()
    assert ok
    assert sup.logic.state() == SessionState.IDLE


def test_cleanup_stale_reports_empty_config(tmp_path):
    sup = _make_supervisor(tmp_path, _mock_cfg(), {})
    ok, reason = sup.run_cleanup_stale()
    assert ok
    assert "cleanup_stale_commands" in reason


def test_cleanup_stale_pkill_exit_1_is_ok(tmp_path, monkeypatch):
    def fake_run(argv, **kwargs):
        class R:
            returncode = 1
            stdout = ""
            stderr = ""

        return R()

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)
    sup = _make_supervisor(
        tmp_path,
        {
            **_mock_cfg(),
            "cleanup_stale_commands": [["pkill", "-f", "skye_robot_driver"]],
        },
        {},
    )
    ok, reason = sup.run_cleanup_stale()
    assert ok, reason


def _precheck_cfg(tmp_path, *, mode_key: str = "dagger"):
    xml = tmp_path / "marvin_ws" / "fastrtps_no_shm.xml"
    xml.parent.mkdir(parents=True)
    xml.write_text("<xml/>")
    return {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": [["pkill", "-f", "skye_robot_driver"]],
        "precheck": {"skip_ping": True},
        "playbook_override": {
            mode_key: [
                {
                    "id": "driver",
                    "argv": ["/bin/true"],
                    "health_key": "driver",
                    "timeout_s": 3.0,
                    "optional": False,
                }
            ]
        },
    }


def _teleop_record_precheck_cfg(tmp_path, cleanup_commands):
    return {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": cleanup_commands,
        "precheck": {"skip_ping": True},
    }


def test_precheck_auto_cleans_residual_driver(tmp_path, monkeypatch):
    calls = {"pgrep": 0, "cleanup": 0}

    def fake_run(argv, **kwargs):
        class R:
            returncode = 0
            stdout = "1"
            stderr = ""

        if argv[:2] == ["pgrep", "-fc"]:
            calls["pgrep"] += 1
            r = R()
            # First count finds residual; after cleanup, gone.
            r.stdout = "1" if calls["pgrep"] == 1 else "0"
            r.returncode = 0 if calls["pgrep"] == 1 else 1
            return r
        if argv[:2] == ["pkill", "-f"]:
            calls["cleanup"] += 1
            return R()
        raise AssertionError(f"unexpected argv: {argv}")

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)
    monkeypatch.setattr("skye_operator_ui.supervisor.time.sleep", lambda _s: None)

    cfg = _precheck_cfg(tmp_path)
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=lambda _k: False,
    )
    ok, reason = sup.start("thor", UiMode.dagger)
    assert ok, reason
    assert calls["cleanup"] == 1
    assert calls["pgrep"] >= 2


def test_precheck_fails_if_residual_remains_after_cleanup(tmp_path, monkeypatch):
    def fake_run(argv, **kwargs):
        class R:
            returncode = 0
            stdout = "1"
            stderr = ""

        if argv[:2] == ["pgrep", "-fc"]:
            return R()  # always residual
        if argv[:2] == ["pkill", "-f"]:
            return R()
        raise AssertionError(f"unexpected argv: {argv}")

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)
    monkeypatch.setattr("skye_operator_ui.supervisor.time.sleep", lambda _s: None)

    cfg = _precheck_cfg(tmp_path)
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=lambda _k: False,
    )
    ok, reason = sup.start("thor", UiMode.dagger)
    assert not ok
    assert "残留" in reason


def test_teleop_record_precheck_runs_teleop_check(tmp_path, monkeypatch):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append(list(argv))
        if len(argv) >= 2 and argv[0] == "bash" and "teleop.sh" in str(argv[1]):
            return subprocess.CompletedProcess(argv, 0, "", "")
        if argv[:2] == ["pkill", "-f"]:
            return subprocess.CompletedProcess(argv, 0, "", "")
        raise AssertionError(f"unexpected argv: {argv}")

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)
    monkeypatch.setattr("skye_operator_ui.supervisor.time.sleep", lambda _s: None)

    cfg = _teleop_record_precheck_cfg(
        tmp_path,
        [["pkill", "-f", "skye_robot_driver"]],
    )
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=lambda _k: False,
    )
    ok, reason = sup.start("orin", UiMode.teleop_record)
    assert ok, reason
    teleop_checks = [
        c
        for c in calls
        if len(c) >= 6
        and c[0] == "bash"
        and "teleop.sh" in c[1]
        and c[2:] == ["check", "--robot", "orin", "--fix-latency"]
    ]
    assert teleop_checks, calls
    assert not any("ping" == c[0] for c in calls)


def test_teleop_record_precheck_skips_docker_rm(tmp_path, monkeypatch):
    calls = []

    def fake_run(argv, **kwargs):
        calls.append(list(argv))
        if len(argv) >= 3 and argv[0] == "bash" and "teleop.sh" in argv[1]:
            return subprocess.CompletedProcess(argv, 0, "", "")
        if argv[:2] == ["pkill", "-f"]:
            return subprocess.CompletedProcess(argv, 0, "", "")
        raise AssertionError(f"unexpected argv: {argv}")

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)
    monkeypatch.setattr("skye_operator_ui.supervisor.time.sleep", lambda _s: None)

    cfg = _teleop_record_precheck_cfg(
        tmp_path,
        [
            ["pkill", "-f", "skye_robot_driver"],
            ["docker", "rm", "-f", "skye_marvin_m6"],
        ],
    )
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=lambda _k: False,
    )
    ok, reason = sup.start("thor", UiMode.teleop_record)
    assert ok, reason
    assert not any(
        c[:3] == ["docker", "rm", "-f"] for c in calls if isinstance(c, list)
    )


def test_teleop_record_precheck_no_fastrtps_xml_required(tmp_path, monkeypatch):
    def fake_run(argv, **kwargs):
        if len(argv) >= 3 and argv[0] == "bash" and "teleop.sh" in argv[1]:
            return subprocess.CompletedProcess(argv, 0, "", "")
        if argv[:2] == ["pkill", "-f"]:
            return subprocess.CompletedProcess(argv, 0, "", "")
        raise AssertionError(f"unexpected argv: {argv}")

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)
    monkeypatch.setattr("skye_operator_ui.supervisor.time.sleep", lambda _s: None)

    cfg = _teleop_record_precheck_cfg(tmp_path, [])
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=lambda _k: False,
    )
    ok, reason = sup.start("thor", UiMode.teleop_record)
    assert ok, reason


def test_controller_ip_by_profile():
    cfg = {
        "controller_ips": {"thor": "6.6.7.191", "orin": "6.6.7.190"},
        "precheck": {"skip_ping": True},
        "cleanup_stale_commands": [],
        "playbook_override": {"teleop_record": []},
    }
    # repo_root unused for this helper
    from pathlib import Path

    sup = SessionSupervisor(
        repo_root=str(Path("/tmp")),
        cfg=cfg,
        health_fn=lambda _k: False,
        precheck_fn=lambda: (True, "ok"),
    )
    assert sup._controller_ip_for_profile("thor") == "6.6.7.191"
    assert sup._controller_ip_for_profile("orin") == "6.6.7.190"
    assert SessionSupervisor(
        repo_root=str(Path("/tmp")),
        cfg={},
        health_fn=lambda _k: False,
        precheck_fn=lambda: (True, "ok"),
    )._controller_ip_for_profile("thor") == "6.6.7.191"


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
