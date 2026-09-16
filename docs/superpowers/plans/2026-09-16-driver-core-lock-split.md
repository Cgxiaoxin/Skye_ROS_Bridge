# DriverCore SDK Lock Split Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Prevent gripper Terminal I/O (Robotiq 485 / DM4310 CAN) from blocking joint feedback and teleop at 250 Hz.

**Architecture:** Split `DriverCore::mutex_` into `runtime_mutex_` (GetRT, SetJointPosCmd, mode/link/shutdown) and `terminal_mutex_` (Terminal_Set/Get/Clear). Use `std::atomic<bool> linked_` for cheap cross-path checks. `shutdown()` takes both locks in fixed order via `std::scoped_lock`.

**Tech Stack:** C++17, Gento L1 SDK, ament_gtest

## Global Constraints

- Single SDK client only (unchanged).
- Assume SDK allows concurrent Runtime feedback and Terminal I/O (separate firmware paths); if not, revert and use async terminal queue instead.
- Thor (DM4310 CAN) and Orin (Robotiq 485) both use `terminal_*` — fix is profile-agnostic.

---

### Task 1: Split locks in DriverCore

**Files:**
- Modify: `skye_ros2_ws/src/skye_robot_driver/include/skye_robot_driver/driver_core.hpp`
- Modify: `skye_ros2_ws/src/skye_robot_driver/src/driver_core.cpp`

**Interfaces:**
- Consumes: existing public API unchanged
- Produces: `runtime_mutex_`, `terminal_mutex_`, atomic `linked_`

- [x] Replace single `mutex_` with dual locks + atomic `linked_`
- [x] `terminal_*` → `terminal_mutex_` only
- [x] `read_state` / `send_position` / runtime control → `runtime_mutex_` only
- [x] `shutdown()` → `std::scoped_lock(runtime_mutex_, terminal_mutex_)`

### Task 2: Verify

- [x] `colcon test --packages-select skye_robot_driver` (7/7 pass)
- [ ] Orin hardware: `enable_gripper:=true`, 扣扳机, `ros2 topic hz /gento/joint_states` stays ~250 Hz
