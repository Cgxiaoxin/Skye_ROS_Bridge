# Task 6 Report: Driver absolute joint control path

## Status
Implemented; commit pending.

## Changes
- Added isolated `handle_absolute_command` for `*_joint_control_abs`.
- Absolute commands skip relative mapping and do not update `leader_ref` / `gento_ref`.
- Preserved finite validation, joint clamp, `max_delta_per_cycle`, and SDK `send_position`.
- Added left/right subscriptions and launch remaps to `/gento/*_joint_control_abs`.
- Updated `docs/ros_interfaces.md` and added `verify_hitl_abs_interfaces.sh`.

## Verification
- `./scripts/build.sh`: passed.
- `colcon test --packages-select skye_robot_driver`: 1/1 passed, 5 tests passed.
- `./scripts/verify_hitl_abs_interfaces.sh`: passed.
