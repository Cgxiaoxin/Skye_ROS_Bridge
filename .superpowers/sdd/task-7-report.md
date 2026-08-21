# Task 7 Report: HITL launch + FACTR remap overlay

## Status

Implemented and committed as `ef522d5`.

## Changes

- `marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py`: FACTR joint/gripper → `/skye/teleop_action_{left,right}` / `/skye/teleop_gripper_{left,right}`; feedback仍 `/gento/joint_states`。
- `skye_hitl_dagger/launch/hitl_dagger.launch.py`: 启动 `control_arbiter` + `hitl_keyboard`；`gripper_invert_on_driver` 默认 true；`episode_recorder` 留 P6.3。
- 设计 spec 状态 → 实现中；`docs/小臂大臂启动步骤.md` 增 HITL 启停小节。

## Verification

- `colcon build --packages-select skye_hitl_dagger`: passed.
- Launch 安装至 `install/skye_hitl_dagger/share/skye_hitl_dagger/launch/hitl_dagger.launch.py`.

## Manual test

```bash
# 1 主机 driver
./scripts/start_skye_for_factr.sh
# 2 Docker HITL FACTR
ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py use_keyboard:=false
# 3 主机 arbiter
source skye_ros2_ws/install/setup.bash
ros2 launch skye_hitl_dagger hitl_dagger.launch.py
```

## Notes

- 日常遥操仍用 `start_teleop_m6_dual_gento.launch.py`（直连 `/gento/*`）。
- HITL FACTR overlay 未同步进 `install/share`；Docker 用 overlay 绝对路径。

## Review fix

- HITL 启停顺序：主机 driver → Docker HITL FACTR launch → `hitl_dagger.launch.py`（对齐 plan Step 3）。
- 设计 spec 状态行去掉尾随空格（`git diff --check`）。
