# Task 4 Report: `control_arbiter_node`

## Status

Implemented and committed as `db3bba4` (`feat(hitl): add control arbiter ROS node`).

## Changes

- Added `ControlArbiterNode` with policy chunk playback through `ChunkPlayer`.
- Added AUTONOMOUS absolute joint outputs and HUMAN teleop passthrough.
- Added takeover/return handling, HANDOVER_SYNC hold behavior, sync and teleop switch commands.
- Added optional `/teleop/state` completion and timeout warning without automatic HUMAN transition.
- Added independent policy gripper publishing with configurable `1-x` inversion.
- Added stale-tail warning and mode publication on `/skye/control_mode`.
- Installed executable `control_arbiter` under `lib/skye_hitl_dagger`.
- Added unit coverage for policy gripper inversion.

## Verification

- Python byte-compilation passed for the node.
- `ReadLints` reported no diagnostics.
- `git diff --check` passed.
- New pytest execution was attempted with `/usr/bin/python3`, but collection was blocked by the repository/ROS environment not exposing the package on `PYTHONPATH`.
- `colcon build --packages-select skye_hitl_dagger` was attempted and failed before compiling this node because the installed ROS `rosidl_generator_py` cannot import `generate_py` from its own package.
- Full ROS startup and topic smoke test could not be completed until the ROS Python/rosidl installation is repaired.

## Concerns

The build failure appears environmental: `/opt/ros/humble` contains an inconsistent `rosidl_generator_py` installation. Re-run the build after repairing that installation or using a clean ROS Humble environment.

## Fix: FACTR mode switch payload (post-review)

- `/mode/switch_sync` now publishes `switch_sync` (not bare `sync`).
- `/mode/switch_teleop` now publishes `switch_teleop` (not bare `teleop`).
- Matches FACTR convention in `docs/新主臂串口绑定.md`.

**Out of scope (Task 6):** downstream consumer wiring for `/gento/{left,right}_joint_control_abs` is not part of Task 4; arbiter publishes abs targets during AUTONOMOUS/HANDOVER_SYNC only.

## Fix: Task 4 important findings

- HANDOVER_SYNC now freezes the takeover-time target, publishes it every joint/gripper tick, and rejects new chunks from replacing the live player until sync completes.
- `/teleop/state` now uses `KEEP_LAST(1)`, `RELIABLE`, and `TRANSIENT_LOCAL` QoS.
- Takeover clears cached teleop state and requires a post-request `TELEOP` observation before completing sync.

## Verification (2026-08-21)

- `colcon build --packages-select skye_hitl_dagger --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3`: passed.
- `/usr/bin/python3 -m py_compile .../control_arbiter_node.py`: passed.
- `git diff --check`: passed.
- `ReadLints`: no diagnostics.
- Focused pytest was attempted but blocked by the host Python/ROS mismatch (`rclpy` Humble binary is Python 3.10 while active pytest uses Python 3.13); no code test failure was observed.

## Fix: P1 stale TRANSIENT_LOCAL TELEOP sync completion

- Replaced timestamp gating with `_awaiting_sync` + `_seen_non_teleop_since_sync_req`.
- Takeover clears `_teleop_state=None`; callback marks non-TELEOP only; timer completes sync on fresh TELEOP after FACTR leaves TELEOP (SYNC) and returns.
- Added `sync_ready()` helper; tests reject stale TELEOP without prior non-TELEOP.
- `colcon build --packages-select skye_hitl_dagger`: passed. 4/4 pytest pass (ROS-sourced).
