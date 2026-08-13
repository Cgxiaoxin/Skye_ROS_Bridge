from __future__ import annotations

import logging
import time
from typing import Optional

import numpy as np

logger = logging.getLogger(__name__)


class ChangingTekGripperAdapter:
    """
    ChangingTek / CTAG2F90D Servo gripper adapter.

    Purpose
    -------
    Adapt `changingtek_p_rtu_Servo.MotorController` to the same minimal
    interface used by the Robotiq HandE driver in `marvin_bimanual.py`:

        g.move(pos=..., speed=..., force=...)
        g.position
        g.ForceValue

    External LeRobot / send_action semantics for arm A
    --------------------------------------------------
    `pos` is an opening value in the range [0, 90]:

        pos = 0   -> fully closed
        pos = 90  -> fully open

    Low-level ChangingTek controller position semantics
    ---------------------------------------------------
    By default, this adapter assumes:

        close_pos = 9000  -> fully closed
        open_pos  = 0     -> fully open

    Therefore the default mapping is:

        action pos = 0   -> controller target = 9000
        action pos = 45  -> controller target = 4500
        action pos = 90  -> controller target = 0

    Notes
    -----
    - The name `position_mm` in the underlying SDK is kept by the SDK, but the
      value is treated here as the controller's internal position/count unit.
    - `ForceValue` is approximated using the controller's real-time current
      feedback because this controller does not expose the same force feedback
      semantics as Robotiq HandE.
    """

    def __init__(
        self,
        port: str,
        slave_id: int = 1,
        baudrate: int = 115200,
        timeout: float = 0.3,
        min_action: float = 0.0,
        max_action: float = 90.0,
        open_pos: int = 0,
        close_pos: int = 9000,
        default_accel: int = 60,
        default_decel: int = 60,
        init_action: Optional[float] = 90.0,
        command_retries: int = 3,
        retry_delay_s: float = 0.15,
    ) -> None:
        try:
            from .changingtek_p_rtu_Servo import MotorController
        except ImportError:
            from changingtek_p_rtu_Servo import MotorController

        self.controller = MotorController(
            port=port,
            slave_id=slave_id,
            baudrate=baudrate,
            timeout=timeout,
        )

        self.min_action = float(min_action)
        self.max_action = float(max_action)
        if self.max_action <= self.min_action:
            raise ValueError(
                "max_action must be larger than min_action, got "
                f"min_action={self.min_action}, max_action={self.max_action}"
            )

        self.open_pos = int(open_pos)
        self.close_pos = int(close_pos)
        self.default_accel = int(default_accel)
        self.default_decel = int(default_decel)
        self.port = port
        self.slave_id = int(slave_id)
        self.command_retries = max(1, int(command_retries))
        self.retry_delay_s = max(0.0, float(retry_delay_s))

        # Last known external action value. Semantics: 0=closed, 90=open.
        self._last_action = float(
            init_action if init_action is not None else self.max_action
        )

        if init_action is not None:
            self.move(pos=float(init_action), speed=50, force=25)

    def _clip_action(self, action_value: float) -> float:
        """Clamp the external gripper action to [min_action, max_action]."""
        return float(np.clip(float(action_value), self.min_action, self.max_action))

    @staticmethod
    def _clip_percent(value: float | int | None, default: int) -> int:
        """Clamp speed/force percentage to [0, 100]."""
        if value is None:
            return int(default)
        return int(np.clip(float(value), 0.0, 100.0))

    def _action_to_controller_pos(self, action_value: float) -> int:
        """
        Convert external action value to ChangingTek controller target position.

        External action:
            min_action / 0   = closed
            max_action / 90  = open

        Controller target:
            close_pos = closed
            open_pos  = open
        """
        action_value = self._clip_action(action_value)

        # open_ratio: 0 = closed, 1 = open
        open_ratio = (
            (action_value - self.min_action)
            / (self.max_action - self.min_action)
        )

        # action=0 -> close_pos, action=90 -> open_pos
        target_pos = self.close_pos + open_ratio * (self.open_pos - self.close_pos)
        return int(round(target_pos))

    def _controller_pos_to_action(self, controller_pos: float) -> float:
        """
        Convert ChangingTek controller feedback position back to external action.

        With the default mapping:
            controller_pos=9000 -> action=0
            controller_pos=0    -> action=90
        """
        denom = float(self.open_pos - self.close_pos)
        if abs(denom) < 1e-9:
            return self._last_action

        open_ratio = (float(controller_pos) - self.close_pos) / denom
        open_ratio = float(np.clip(open_ratio, 0.0, 1.0))

        action_value = self.min_action + open_ratio * (
            self.max_action - self.min_action
        )
        return self._clip_action(action_value)

    def move(self, pos: float, speed: int = 50, force: int = 25) -> None:
        """
        Move gripper using the same public interface as HandEForRtu.move().

        Parameters
        ----------
        pos:
            External opening command in [0, 90].
            0 = closed, 90 = open.
        speed:
            Speed percentage, clipped to [0, 100].
        force:
            Force/torque percentage, clipped to [0, 100].
        """
        action_value = self._clip_action(pos)
        target_pos = self._action_to_controller_pos(action_value)

        speed_pct = self._clip_percent(speed, default=50)
        force_pct = self._clip_percent(force, default=25)

        for attempt in range(1, self.command_retries + 1):
            try:
                self.controller.temp_move(
                    position_mm=target_pos,
                    speed_pct=speed_pct,
                    force_pct=force_pct,
                    accel=self.default_accel,
                    decel=self.default_decel,
                    trigger=True,
                )
                break
            except Exception as exc:
                if attempt >= self.command_retries:
                    raise RuntimeError(
                        "ChangingTek gripper did not respond on "
                        f"{self.port} (slave_id={self.slave_id}) while moving "
                        f"to action={action_value:.1f}, target_pos={target_pos}. "
                        "Check USB-RS485 adapter, gripper power, A/B wiring, "
                        "Modbus slave id, baudrate, and whether another process "
                        "is using the serial port."
                    ) from exc
                logger.warning(
                    "ChangingTek move failed on %s slave_id=%s "
                    "(attempt %d/%d): %s",
                    self.port,
                    self.slave_id,
                    attempt,
                    self.command_retries,
                    exc,
                )
                time.sleep(self.retry_delay_s)

        self._last_action = action_value

    @property
    def position(self) -> float:
        """
        Return current gripper opening in external action semantics.

        Returns
        -------
        float
            0 = closed, 90 = open.

        This is intentionally different from the Robotiq HandE raw position
        semantics. `marvin_bimanual.py` handles A/B gripper type differences.
        """
        try:
            real_pos = self.controller.read_real_position()
            action_value = self._controller_pos_to_action(real_pos)
            self._last_action = action_value
            return action_value
        except Exception:
            logger.exception(
                "ChangingTekGripperAdapter.read_real_position failed; "
                "using last action %.3f",
                self._last_action,
            )
            return self._last_action

    @property
    def ForceValue(self) -> float:
        """
        Return force-like feedback for compatibility with HandEForRtu.

        ChangingTek does not expose the same force feedback as Robotiq HandE in
        this SDK, so real-time current is used as an approximate feedback value.
        """
        try:
            return float(self.controller.read_real_current())
        except Exception:
            logger.exception(
                "ChangingTekGripperAdapter.read_real_current failed; return 0.0"
            )
            return 0.0
