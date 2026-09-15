import time
from unittest.mock import patch

import pytest

from skye_operator_ui.playbooks import playbook_for
from skye_operator_ui.process_step import LogRing, ProcessStep
from skye_operator_ui.session_state import UiMode


def test_log_ring_caps():
    r = LogRing(maxlen=3)
    for i in range(5):
        r.append(str(i))
    assert r.tail(10) == ["2", "3", "4"]


def test_log_ring_default_maxlen():
    r = LogRing()
    for i in range(2005):
        r.append(str(i))
    assert len(r.tail(2005)) == 2000
    assert r.tail(1) == ["2004"]


def test_process_step_start_and_terminate():
    step = ProcessStep(
        step_id="sleep",
        argv=["/bin/sleep", "30"],
        env={},
        health_check=lambda: True,
        timeout_s=2.0,
    )
    step.start()
    assert step.poll_health()
    step.terminate(grace_s=1.0)
    if step._proc.poll() is None:  # noqa: SLF001
        pytest.skip("process signals unavailable in this environment")
    assert step._proc.poll() is not None  # noqa: SLF001


def test_process_step_captures_stdout():
    step = ProcessStep(
        step_id="echo",
        argv=["/bin/sh", "-c", "echo hello-step"],
        env={},
        health_check=lambda: True,
        timeout_s=2.0,
    )
    step.start()
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if any("hello-step" in line for line in step.logs.tail(20)):
            break
        time.sleep(0.05)
    step.terminate(grace_s=1.0)
    assert any("hello-step" in line for line in step.logs.tail(20))


def test_playbook_for_teleop_record():
    repo = "/data/repo"
    cfg = {"step_timeout_s": 90.0}
    steps = playbook_for(UiMode.teleop_record, repo, "thor", cfg)
    assert steps[0]["id"] == "driver"
    assert steps[0]["argv"] == [f"{repo}/scripts/start_skye_for_factr.sh"]
    assert steps[0]["env"]["ROBOT_PROFILE"] == "thor"
    assert steps[0]["health_key"] == "driver"
    assert steps[1]["id"] == "marvin"
    assert steps[1]["argv"] == [f"{repo}/scripts/run_marvin_m6_impedance.sh"]
    optional = [s for s in steps if s.get("optional")]
    assert any(s["id"] == "align" for s in optional)


def test_playbook_for_dagger():
    repo = "/data/repo"
    cfg = {"step_timeout_s": 90.0}
    steps = playbook_for(UiMode.dagger, repo, "orin", cfg)
    assert steps[0]["health_key"] == "driver"
    assert steps[2]["id"] == "arbiter"
    assert steps[2]["argv"] == [f"{repo}/scripts/start_hitl_host.sh", "--arbiter-only"]
    assert steps[2]["env"]["ROBOT_PROFILE"] == "orin"
    assert "ENABLE_RECORDER" not in steps[2]["env"]
    hitl_hosts = [
        s["argv"]
        for s in steps
        if s["argv"] and "start_hitl_host.sh" in s["argv"][0]
    ]
    assert len(hitl_hosts) == 1


def test_playbook_for_dagger_enable_hitl_recorder():
    repo = "/data/repo"
    cfg = {"step_timeout_s": 90.0, "enable_hitl_recorder": True}
    steps = playbook_for(UiMode.dagger, repo, "orin", cfg)
    arbiter = next(s for s in steps if s["id"] == "arbiter")
    assert arbiter["env"]["ENABLE_RECORDER"] == "true"
    assert not any(s["id"] == "recorder" for s in steps)


def test_process_step_terminate_killpg_permission_error():
    step = ProcessStep(
        step_id="sleep",
        argv=["/bin/sleep", "30"],
        env={},
        health_check=lambda: True,
        timeout_s=2.0,
    )
    step.start()
    proc = step._proc  # noqa: SLF001
    with (
        patch("skye_operator_ui.process_step.os.killpg", side_effect=PermissionError),
        patch("skye_operator_ui.process_step.os.kill", side_effect=PermissionError),
        patch.object(proc, "terminate") as mock_terminate,
    ):
        step.terminate(grace_s=0.1)
    mock_terminate.assert_called()


def test_playbook_marvin_start_cmd_override():
    repo = "/data/repo"
    cfg = {
        "step_timeout_s": 60.0,
        "playbook": {
            "marvin_start_cmd": ["/custom/marvin_wrapper.sh", "--non-interactive"],
        },
    }
    steps = playbook_for(UiMode.teleop_record, repo, "thor", cfg)
    marvin = next(s for s in steps if s["id"] == "marvin")
    assert marvin["argv"] == ["/custom/marvin_wrapper.sh", "--non-interactive"]


def test_playbook_override_takes_priority():
    repo = "/data/repo"
    cfg = {
        "playbook_override": {
            "teleop_record": [
                {
                    "id": "mock",
                    "argv": ["/bin/true"],
                    "env": {},
                    "health_key": "mock",
                    "timeout_s": 1.0,
                    "optional": False,
                }
            ]
        }
    }
    steps = playbook_for(UiMode.teleop_record, repo, "thor", cfg)
    assert len(steps) == 1
    assert steps[0]["id"] == "mock"
