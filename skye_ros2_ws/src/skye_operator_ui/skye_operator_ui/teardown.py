"""Best-effort session teardown: remove Marvin Docker + disable leader DXL."""

from __future__ import annotations

import logging
import os
import subprocess
import sys
from pathlib import Path
from typing import Any

logger = logging.getLogger(__name__)

DEFAULT_CONTAINER = "skye_marvin_m6"
DEFAULT_IMAGE = "harbor.amigos-robot.com/tmp/marvin-m6-ros2:humble"


def _teardown_cfg(cfg: dict[str, Any]) -> dict[str, Any]:
    raw = cfg.get("teardown")
    return dict(raw) if isinstance(raw, dict) else {}


def run_safe_teardown(repo_root: str, cfg: dict[str, Any]) -> list[str]:
    """Remove Marvin container then (optionally) disable leader Dynamixel.

    Disable prefers a one-shot Docker run (host often lacks dynamixel_sdk).
    Never raises. Returns human-readable notes for any failures.
    """
    tcfg = _teardown_cfg(cfg)
    notes: list[str] = []
    name = str(tcfg.get("marvin_container_name") or DEFAULT_CONTAINER)
    docker_timeout = float(tcfg.get("docker_rm_timeout_s", 20.0))
    disable_timeout = float(tcfg.get("disable_timeout_s", 60.0))
    disable_leader = bool(tcfg.get("disable_leader", True))
    image = str(
        tcfg.get("marvin_image")
        or os.environ.get("IMAGE")
        or DEFAULT_IMAGE
    )
    # via_docker default True — host Python usually has no dynamixel_sdk.
    via_docker = bool(tcfg.get("disable_via_docker", True))

    note = _docker_rm_f(name, timeout_s=docker_timeout)
    if note:
        notes.append(note)

    if disable_leader:
        note = _disable_leader(
            repo_root,
            timeout_s=disable_timeout,
            via_docker=via_docker,
            image=image,
        )
        if note:
            notes.append(note)

    return notes


def _docker_rm_f(name: str, *, timeout_s: float) -> str | None:
    argv = ["docker", "rm", "-f", name]
    try:
        result = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except FileNotFoundError:
        msg = "docker 不可用，无法强制移除小臂容器"
        logger.warning(msg)
        return msg
    except subprocess.TimeoutExpired:
        msg = f"docker rm -f {name} 超时"
        logger.warning(msg)
        return msg
    except OSError as exc:
        msg = f"docker rm 失败: {exc}"
        logger.warning(msg)
        return msg

    if result.returncode != 0:
        err = (result.stderr or result.stdout or "").strip()
        if "No such container" in err or "No such" in err:
            logger.info("marvin container %s already gone", name)
            return None
        if not err:
            return None
        msg = f"docker rm -f {name} 返回 {result.returncode}: {err}"
        logger.warning(msg)
        return msg

    logger.info("removed marvin container %s", name)
    return None


def _disable_leader(
    repo_root: str,
    *,
    timeout_s: float,
    via_docker: bool,
    image: str,
) -> str | None:
    script = Path(repo_root) / "scripts" / "disable_leader_dynamixel.py"
    if not script.is_file():
        msg = f"缺少去使能脚本: {script}"
        logger.warning(msg)
        return msg

    marvin_ws = Path(repo_root) / "marvin_ws"
    scripts_dir = Path(repo_root) / "scripts"

    if via_docker:
        note = _disable_via_docker(
            image=image,
            marvin_ws=marvin_ws,
            scripts_dir=scripts_dir,
            timeout_s=timeout_s,
        )
        if note is None:
            return None
        logger.warning("docker 去使能失败，尝试主机 Python: %s", note)

    return _disable_on_host(script, marvin_ws=marvin_ws, timeout_s=timeout_s)


def _disable_via_docker(
    *,
    image: str,
    marvin_ws: Path,
    scripts_dir: Path,
    timeout_s: float,
) -> str | None:
    # One-shot container with /dev so serial by-id is visible; image has SDK.
    argv = [
        "docker",
        "run",
        "--rm",
        "--privileged",
        "--network",
        "host",
        "-v",
        "/dev:/dev",
        "-v",
        f"{marvin_ws}:/marvin_ws",
        "-v",
        f"{scripts_dir}:/scripts:ro",
        "-e",
        "MARVIN_WS=/marvin_ws",
        "-w",
        "/marvin_ws",
        image,
        "python3",
        "/scripts/disable_leader_dynamixel.py",
    ]
    try:
        result = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
        )
    except FileNotFoundError:
        return "docker 不可用，无法在容器内去使能"
    except subprocess.TimeoutExpired:
        return "docker 去使能超时"
    except OSError as exc:
        return f"docker 去使能失败: {exc}"

    if result.returncode != 0:
        err = (result.stderr or result.stdout or "").strip()
        return f"docker 去使能返回 {result.returncode}: {err}"

    logger.info("leader Dynamixel disabled via docker")
    return None


def _disable_on_host(
    script: Path,
    *,
    marvin_ws: Path,
    timeout_s: float,
) -> str | None:
    argv = [sys.executable or "python3", str(script)]
    env = os.environ.copy()
    env.setdefault("MARVIN_WS", str(marvin_ws))
    try:
        result = subprocess.run(
            argv,
            capture_output=True,
            text=True,
            timeout=timeout_s,
            check=False,
            env=env,
            cwd=str(script.parent.parent),
        )
    except subprocess.TimeoutExpired:
        msg = "去使能脚本超时"
        logger.warning(msg)
        return msg
    except OSError as exc:
        msg = f"去使能脚本失败: {exc}"
        logger.warning(msg)
        return msg

    if result.returncode != 0:
        err = (result.stderr or result.stdout or "").strip()
        msg = f"去使能失败 ({result.returncode}): {err}"
        logger.warning(msg)
        return msg

    logger.info("leader Dynamixel disabled on host")
    return None
