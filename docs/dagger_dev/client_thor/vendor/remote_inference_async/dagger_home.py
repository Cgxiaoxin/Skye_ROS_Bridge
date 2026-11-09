"""Quintic absolute-radian go-home for HITL DAgger (via /skye/policy_action).

Does NOT link Gento SDK. Requires arbiter in AUTONOMOUS so abs cmds reach
skye_robot_driver. Matches legacy run_robot initial move semantics:
synchronized quintic, default 0.5deg @ 20Hz, max feedback lead gating.
"""

from __future__ import annotations

import logging
import time
from typing import Optional

import numpy as np

from remote_inference_async.dagger_chunk import (
    ARBITER_CHUNK_SIZE,
    assert_policy_chunk_radians,
    robot_command_deg_to_hold_chunk_rad,
)
from remote_inference_async.dagger_quintic import (
    max_command_lead_deg,
    plan_quintic_trajectory_steps,
    sample_quintic_commands,
)

LOGGER = logging.getLogger("tianji_async_client")
ARM_DOF = 7


def _hold_from_arms_deg(
    left_deg: np.ndarray,
    right_deg: np.ndarray,
    *,
    left_gripper: float,
    right_gripper: float,
) -> np.ndarray:
    command = np.concatenate(
        [
            np.asarray(left_deg, dtype=np.float64).reshape(ARM_DOF),
            np.asarray(right_deg, dtype=np.float64).reshape(ARM_DOF),
            [float(left_gripper), float(right_gripper)],
        ]
    )
    return robot_command_deg_to_hold_chunk_rad(command, steps=ARBITER_CHUNK_SIZE)


def run_dagger_slow_home(
    *,
    robot,
    publisher,
    left_home_deg: np.ndarray,
    right_home_deg: np.ndarray,
    step_deg: float = 0.5,
    hz: float = 20.0,
    max_lead_deg: float = 5.0,
    settle_tol_deg: float = 2.0,
    settle_timeout_s: float = 60.0,
    policy_version: str = "unijepa-home",
    stop_event: Optional[object] = None,
    left_gripper: float = 0.0,
    right_gripper: float = 0.0,
) -> None:
    """Publish a quintic home trajectory through arbiter, then wait near goal."""
    if hz <= 0.0 or step_deg <= 0.0 or max_lead_deg <= 0.0:
        raise ValueError(
            f"invalid home speed: step_deg={step_deg} hz={hz} max_lead_deg={max_lead_deg}"
        )
    if max_lead_deg < step_deg:
        raise ValueError("home max lead must be at least one command step")
    dt = 1.0 / float(hz)

    rt = robot.get_rt_dict()
    if "error" in rt:
        raise RuntimeError(f"slow-home cannot read joint_states: {rt['error']}")
    left_now = np.asarray(rt["arms"][0]["fb"]["fb_pos"], dtype=np.float64)
    right_now = np.asarray(rt["arms"][1]["fb"]["fb_pos"], dtype=np.float64)
    if left_now.shape != (ARM_DOF,) or right_now.shape != (ARM_DOF,):
        raise RuntimeError(
            f"invalid joint feedback shapes: L={left_now.shape} R={right_now.shape}"
        )

    starts = {0: left_now.copy(), 1: right_now.copy()}
    targets = {
        0: np.asarray(left_home_deg, dtype=np.float64).reshape(ARM_DOF),
        1: np.asarray(right_home_deg, dtype=np.float64).reshape(ARM_DOF),
    }
    trajectory_steps, max_distance_deg = plan_quintic_trajectory_steps(
        starts, targets, step_deg=step_deg
    )
    planned_duration = trajectory_steps / float(hz)
    plan_limit_s = 180.0
    exec_margin_s = 45.0
    if planned_duration > plan_limit_s:
        raise TimeoutError(
            f"DAGGER quintic home requires {planned_duration:.2f}s > "
            f"{plan_limit_s:.0f}s cap (max_distance={max_distance_deg:.3f} deg, "
            f"step={step_deg:.3f})"
        )
    execution_timeout_s = max(float(settle_timeout_s), planned_duration + exec_margin_s)

    LOGGER.info(
        "DAGGER quintic-home: steps=%d step_deg=%.2f hz=%.1f lead<=%.1f "
        "duration=%.1fs timeout=%.1fs from L=%s R=%s → L=%s R=%s "
        "(publish absolute RADIAN via /skye/policy_action)",
        trajectory_steps,
        step_deg,
        hz,
        max_lead_deg,
        planned_duration,
        execution_timeout_s,
        np.round(left_now, 1).tolist(),
        np.round(right_now, 1).tolist(),
        np.round(targets[0], 1).tolist(),
        np.round(targets[1], 1).tolist(),
    )

    period = dt
    deadline = time.monotonic() + execution_timeout_s
    last_commands = {obj: start.copy() for obj, start in starts.items()}
    trajectory_index = 0
    blocked_cycles = 0
    next_command_at = time.monotonic()
    log_interval_cycles = max(1, int(round(hz)))

    while trajectory_index <= trajectory_steps:
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError("slow-home interrupted by stop")
        if time.monotonic() >= deadline:
            raise TimeoutError(
                f"DAGGER quintic-home did not finish within {execution_timeout_s:.1f}s "
                f"(step={trajectory_index}/{trajectory_steps}); "
                "check AUTONOMOUS + arbiter + driver"
            )

        rt = robot.get_rt_dict()
        if "error" in rt:
            raise RuntimeError(f"slow-home joint_states error: {rt['error']}")
        feedback = {
            0: np.asarray(rt["arms"][0]["fb"]["fb_pos"], dtype=np.float64),
            1: np.asarray(rt["arms"][1]["fb"]["fb_pos"], dtype=np.float64),
        }
        proposed = sample_quintic_commands(
            starts,
            targets,
            trajectory_index=trajectory_index,
            trajectory_steps=trajectory_steps,
        )
        for obj, command in proposed.items():
            step = float(np.max(np.abs(command - last_commands[obj])))
            if step > step_deg + 1e-6:
                raise RuntimeError(
                    f"arm {obj} quintic step {step:.6f} deg exceeds limit {step_deg:.6f}"
                )

        max_lead, blocked_arm = max_command_lead_deg(proposed, feedback)
        advance = max_lead <= max_lead_deg
        commands = proposed if advance else last_commands
        hold = _hold_from_arms_deg(
            commands[0],
            commands[1],
            left_gripper=left_gripper,
            right_gripper=right_gripper,
        )
        assert_policy_chunk_radians(hold, context="dagger_quintic_home")
        publisher.publish_policy_actions(hold, dt=dt, policy_version=policy_version)

        if advance:
            last_commands = {obj: cmd.copy() for obj, cmd in proposed.items()}
            if trajectory_index == 0 or trajectory_index % log_interval_cycles == 0:
                LOGGER.info(
                    "DAGGER quintic-home step=%d/%d lead=%.2f deg (rad check ok)",
                    trajectory_index,
                    trajectory_steps,
                    max_lead,
                )
            trajectory_index += 1
            blocked_cycles = 0
        else:
            blocked_cycles += 1
            if blocked_cycles == 1 or blocked_cycles % log_interval_cycles == 0:
                LOGGER.warning(
                    "Pausing DAGGER quintic-home for feedback: arm=%d, "
                    "command_lead=%.3f deg, limit=%.3f deg, step=%d/%d",
                    blocked_arm,
                    max_lead,
                    max_lead_deg,
                    trajectory_index,
                    trajectory_steps,
                )

        next_command_at += period
        remaining = next_command_at - time.monotonic()
        t_end = time.monotonic() + max(remaining, 0.0)
        while time.monotonic() < t_end:
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("slow-home interrupted by stop")
            robot.spin_once(0.005)
            publisher.spin_once(0.0)
            time.sleep(0.001)
        if remaining <= 0:
            next_command_at = time.monotonic()

    # Hold final home pose while settling.
    hold = _hold_from_arms_deg(
        targets[0],
        targets[1],
        left_gripper=left_gripper,
        right_gripper=right_gripper,
    )
    settle_deadline = time.monotonic() + max(float(settle_timeout_s), 5.0)
    while time.monotonic() < settle_deadline:
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError("slow-home interrupted by stop")
        publisher.publish_policy_actions(hold, dt=dt, policy_version=policy_version)
        rt = robot.get_rt_dict()
        if "error" not in rt:
            left = np.asarray(rt["arms"][0]["fb"]["fb_pos"], dtype=np.float64)
            right = np.asarray(rt["arms"][1]["fb"]["fb_pos"], dtype=np.float64)
            err = max(
                float(np.max(np.abs(left - targets[0]))),
                float(np.max(np.abs(right - targets[1]))),
            )
            if err <= settle_tol_deg:
                LOGGER.info(
                    "DAGGER quintic-home settled: max_joint_err=%.2f deg (tol=%.2f)",
                    err,
                    settle_tol_deg,
                )
                return
        robot.spin_once(0.05)
        publisher.spin_once(0.0)
        time.sleep(dt)

    raise TimeoutError(
        f"DAGGER quintic-home did not settle within {settle_timeout_s:.1f}s "
        f"(tol={settle_tol_deg} deg); check AUTONOMOUS + arbiter + driver"
    )
