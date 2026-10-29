# HITL：同构遥操接入 Arbiter Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.  
> **Spec:** [`docs/dagger_dev/skye-hitl-dagger-refactor-plan.md`](../../dagger_dev/skye-hitl-dagger-refactor-plan.md) Phase 1

**Goal:** HITL on 时 `skye_factr_teleop` 不再直写 `/gento/*_joint_control`，改为 `/skye/teleop_action_*`，由已有 `control_arbiter` 唯一写出 gento。

**Architecture:** 扩展 `launch_support` 的 remapping（`hitl=True`）；新增 dual HITL launch；纯遥操 launch 不变。Arbiter 仍用 `skye_hitl_dagger/launch/hitl_dagger.launch.py`。

**Tech Stack:** ROS2 Humble, `skye_factr_teleop`, `skye_hitl_dagger`, pytest

## Global Constraints

- HITL off：`start_teleop_m6_*.launch.py` remaps 保持 `/gento/{side}_joint_control`
- HITL on：`/joint_control` → `/skye/teleop_action_{side}`；`/gripper/ctrl` → `/skye/teleop_gripper_{side}`
- `/joint_state` 仍订 `/gento/{side}_joint_states`
- **`robot_profile:=thor|orin` 三处一致**：driver、dual_hitl、hitl_dagger（arbiter signs = driver 7 轴，非小臂 8 轴）
- 正式拓扑：driver+arbiter 在对应开发机；本机起 driver 仅调试（见 SoT §0.1）
- 不重写 arbiter 状态机

---

### Task 1: HITL remappings in launch_support

**Files:**
- Modify: `skye_ros2_ws/src/skye_factr_teleop/skye_factr_teleop/launch_support.py`
- Modify: `skye_ros2_ws/src/skye_factr_teleop/test/test_launch.py`
- Create: `skye_ros2_ws/src/skye_factr_teleop/launch/start_teleop_m6_dual_hitl.launch.py`

- [x] Add `leader_remappings(side, *, hitl=False)` and wire into `_launch_setup`
- [x] Add `leader_launch_description(sides, *, hitl=False)`
- [x] Create dual HITL launch calling `hitl=True`
- [x] Tests: pure teleop remaps unchanged; HITL dual remaps to `/skye/teleop_*`
- [x] Run: `pytest …/test_launch.py` → 38 passed

### Task 2: Startup notes (formal vs debug)

**Files:**
- Create: `docs/dagger_dev/phase1_hitl_isomorphic_startup.md`
- Modify: `docs/dagger_dev/skye-hitl-dagger-refactor-plan.md` §9 checkbox status

- [x] Document start order: driver (dev machine) → dual_hitl → hitl_dagger → optional dummy
- [x] Two columns: formal Thor/Orin vs single-user laptop debug
- [x] Mark Phase 1 launch items done in refactor plan when verified
