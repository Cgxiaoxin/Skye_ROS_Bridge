"""Startup playbooks for teleop recording and HITL DAgger sessions."""

from __future__ import annotations

import copy
import os
from typing import Any

from skye_operator_ui.session_state import UiMode

# v1 marvin step: run_marvin_m6_impedance.sh drops into an interactive docker
# shell (`docker run -it`). For operator UI automation, override the wrapper via
# cfg.playbook.marvin_start_cmd (argv list). Pass MARVIN_LAUNCH_CMD in env for
# the non-interactive ros2 launch line inside the container (see Hint doc).

_TELEOP_MARVIN_LAUNCH = (
    "ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py "
    "use_keyboard:=false"
)
_HITL_MARVIN_LAUNCH = (
    "ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py "
    "use_keyboard:=false"
)


def _step_timeout(cfg: dict[str, Any]) -> float:
    return float(cfg.get("step_timeout_s", 120.0))


def _base_env(profile: str, repo_root: str, cfg: dict[str, Any]) -> dict[str, str]:
    env = {
        "ROBOT_PROFILE": profile,
        "ROS_DOMAIN_ID": str(cfg.get("ros_domain_id", 21)),
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
        "FASTRTPS_DEFAULT_PROFILES_FILE": os.path.join(
            repo_root, "marvin_ws", "fastrtps_no_shm.xml"
        ),
    }
    launch_cmd = cfg.get("playbook", {}).get("marvin_launch_cmd")
    if launch_cmd:
        env["MARVIN_LAUNCH_CMD"] = str(launch_cmd)
    elif os.environ.get("MARVIN_LAUNCH_CMD"):
        env["MARVIN_LAUNCH_CMD"] = os.environ["MARVIN_LAUNCH_CMD"]
    return env


def _marvin_argv(repo_root: str, cfg: dict[str, Any]) -> list[str]:
    override = cfg.get("playbook", {}).get("marvin_start_cmd")
    if override:
        return list(override)
    return [os.path.join(repo_root, "scripts", "run_marvin_m6_impedance.sh")]


def _recorder_argv(repo_root: str) -> list[str]:
    setup = os.path.join(repo_root, "skye_ros2_ws", "install", "setup.bash")
    return [
        "bash",
        "-lc",
        f"source {setup} && ros2 launch skye_data_recorder data_recorder.launch.py",
    ]


def _make_step(
    step_id: str,
    argv: list[str],
    env: dict[str, str],
    health_key: str,
    timeout_s: float,
    optional: bool = False,
) -> dict[str, Any]:
    return {
        "id": step_id,
        "argv": argv,
        "env": copy.copy(env),
        "health_key": health_key,
        "timeout_s": timeout_s,
        "optional": optional,
    }


def _teleop_playbook(
    repo_root: str,
    profile: str,
    cfg: dict[str, Any],
) -> list[dict[str, Any]]:
    timeout_s = _step_timeout(cfg)
    env = _base_env(profile, repo_root, cfg)
    marvin_env = copy.copy(env)
    marvin_env.setdefault("MARVIN_LAUNCH_CMD", _TELEOP_MARVIN_LAUNCH)
    return [
        _make_step(
            "driver",
            [os.path.join(repo_root, "scripts", "start_skye_for_factr.sh")],
            env,
            "driver",
            timeout_s,
        ),
        _make_step(
            "marvin",
            _marvin_argv(repo_root, cfg),
            marvin_env,
            "marvin",
            timeout_s,
        ),
        _make_step(
            "align",
            [os.path.join(repo_root, "scripts", "start_follower_align.sh")],
            env,
            "align",
            timeout_s,
            optional=True,
        ),
        _make_step(
            "recorder",
            _recorder_argv(repo_root),
            env,
            "recorder",
            timeout_s,
        ),
    ]


def _dagger_playbook(
    repo_root: str,
    profile: str,
    cfg: dict[str, Any],
) -> list[dict[str, Any]]:
    timeout_s = _step_timeout(cfg)
    env = _base_env(profile, repo_root, cfg)
    marvin_env = copy.copy(env)
    marvin_env["MARVIN_LAUNCH_CMD"] = marvin_env.get(
        "MARVIN_LAUNCH_CMD", _HITL_MARVIN_LAUNCH
    )
    setup = os.path.join(repo_root, "skye_ros2_ws", "install", "setup.bash")
    arbiter_env = copy.copy(env)
    if cfg.get("enable_hitl_recorder", False):
        arbiter_env["ENABLE_RECORDER"] = "true"
    return [
        _make_step(
            "driver",
            [os.path.join(repo_root, "scripts", "start_skye_for_factr.sh")],
            env,
            "driver",
            timeout_s,
        ),
        _make_step(
            "marvin",
            _marvin_argv(repo_root, cfg),
            marvin_env,
            "marvin",
            timeout_s,
        ),
        _make_step(
            "arbiter",
            [
                os.path.join(repo_root, "scripts", "start_hitl_host.sh"),
                "--arbiter-only",
            ],
            arbiter_env,
            "arbiter",
            timeout_s,
        ),
        _make_step(
            "dummy_policy",
            [
                "bash",
                "-lc",
                f"source {setup} && ros2 run skye_hitl_dagger pub_dummy_policy_chunk",
            ],
            env,
            "policy",
            timeout_s,
            optional=True,
        ),
    ]


def playbook_for(
    mode: UiMode,
    repo_root: str,
    profile: str,
    cfg: dict[str, Any],
) -> list[dict[str, Any]]:
    """Return ordered startup steps for the given UI mode."""
    override = cfg.get("playbook_override", {})
    mode_key = mode.name
    if mode_key in override:
        return copy.deepcopy(override[mode_key])
    if mode == UiMode.teleop_record:
        return _teleop_playbook(repo_root, profile, cfg)
    if mode == UiMode.dagger:
        return _dagger_playbook(repo_root, profile, cfg)
    raise ValueError(f"unsupported mode: {mode}")
