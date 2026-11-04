"""Startup playbooks for teleop recording and HITL DAgger sessions."""

from __future__ import annotations

import copy
import os
from typing import Any

from skye_operator_ui.session_state import UiMode

# Keep in sync with scripts/lib/robot_profile.sh robot_controller_ip().
_CONTROLLER_IP = {
    "thor": "6.6.7.191",
    "orin": "6.6.7.190",
}

_DOMAIN = {"thor": "20", "orin": "21"}

# FastDDS profile ships with the driver package (4 MB UDP + SHM).
_FASTDDS_PROFILE = ("skye_ros2_ws", "src", "skye_robot_driver", "config",
                    "fastrtps_teleop.xml")


def fastdds_profile_path(repo_root: str) -> str:
    return os.path.join(repo_root, *_FASTDDS_PROFILE)


def _step_timeout(cfg: dict[str, Any]) -> float:
    return float(cfg.get("step_timeout_s", 120.0))


def _base_env(profile: str, repo_root: str, cfg: dict[str, Any]) -> dict[str, str]:
    key = profile.strip().lower()
    env = {
        "ROBOT_PROFILE": key,
        "ROBOT_IP": _CONTROLLER_IP.get(key, _CONTROLLER_IP["thor"]),
        "ROS_DOMAIN_ID": str(
            cfg.get("ros_domain_id", _DOMAIN.get(key, "21"))
        ),
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    return env


def _dagger_base_env(
    profile: str, repo_root: str, cfg: dict[str, Any]
) -> dict[str, str]:
    env = _base_env(profile, repo_root, cfg)
    env["FASTRTPS_DEFAULT_PROFILES_FILE"] = fastdds_profile_path(repo_root)
    return env


def _hitl_leaders_argv(repo_root: str, profile: str) -> list[str]:
    setup = os.path.join(repo_root, "skye_ros2_ws", "install", "setup.bash")
    key = profile.strip().lower()
    return [
        "bash",
        "-lc",
        (
            f"source {setup} && "
            f"ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py "
            f"robot_profile:={key}"
        ),
    ]


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
    key = profile.strip().lower()
    return [
        _make_step(
            "teleop",
            [
                "bash",
                os.path.join(repo_root, "scripts", "teleop.sh"),
                "start",
                "--robot",
                key,
                "--skip-precheck",
            ],
            env,
            "teleop",
            timeout_s,
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
    env = _dagger_base_env(profile, repo_root, cfg)
    setup = os.path.join(repo_root, "skye_ros2_ws", "install", "setup.bash")
    arbiter_env = copy.copy(env)
    if cfg.get("enable_hitl_recorder", False):
        arbiter_env["ENABLE_RECORDER"] = "true"
    return [
        _make_step(
            "driver",
            [os.path.join(repo_root, "scripts", "start_skye_for_factr.sh"), profile],
            env,
            "driver",
            timeout_s,
        ),
        _make_step(
            "leaders",
            _hitl_leaders_argv(repo_root, profile),
            env,
            "teleop",
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
