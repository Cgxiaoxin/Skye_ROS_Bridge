"""Safely probe the right DM gripper without commanding either robot arm."""

from __future__ import annotations

import json
import logging
import time

from PYTHON_SDK.GentoRobot import FXLogMask, GentoRobot
from remote_inference_async.gento_dm_gripper import (
    CHANNEL_CANFD,
    RIGHT_MASTER_ID,
    RIGHT_SLAVE_ID,
    TERMINAL_ARM1,
    GentoDmGripper,
    _DISABLE,
    terminal_can_frame,
)


CONTROLLER_IP = (6, 6, 7, 191)


def main() -> int:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    robot = GentoRobot()
    gripper: GentoDmGripper | None = None
    try:
        ret = robot.link(*CONTROLLER_IP, log_level=FXLogMask.FX_LOG_INFO_FLAG)
        if not robot._connected:
            raise RuntimeError(
                f"robot link failed: {ret} ({robot._get_operate_error_msg(ret)})"
            )

        gripper = GentoDmGripper(
            robot,
            limits=(0.097, 1.212),
            terminal_type=TERMINAL_ARM1,
            arm_label="right",
            slave_id=RIGHT_SLAVE_ID,
            master_id=RIGHT_MASTER_ID,
            command_hz=100.0,
            kp=3.0,
            kd=0.12,
            initialization_timeout_s=3.0,
            trace_interval_s=0.0,
        )
        time.sleep(0.25)
        # _snapshot verifies that feedback is current and status is enabled.
        normalized = gripper._snapshot(max_age_s=0.5)
        diagnostics = gripper.diagnostics()
        diagnostics["normalized_position"] = normalized
        print("RIGHT_GRIPPER_OK " + json.dumps(diagnostics, sort_keys=True))
        return 0
    finally:
        if gripper is not None:
            gripper.close()
        if robot._connected:
            # The normal backend holds the measured pose. For this one-shot
            # probe, explicitly leave the motor disabled after its thread has
            # stopped so no command can race with the disable frame.
            disable_ret, _ = robot.terminal_set(
                TERMINAL_ARM1,
                CHANNEL_CANFD,
                terminal_can_frame(RIGHT_SLAVE_ID, _DISABLE),
                timeout=20,
            )
            if disable_ret != 0:
                logging.getLogger("right_gripper_probe").error(
                    "right gripper disable failed: ret=%d", disable_ret
                )
            robot.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
