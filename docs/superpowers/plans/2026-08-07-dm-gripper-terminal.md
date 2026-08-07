# DM 夹爪 Terminal 桥接 Implementation Plan

> **For agentic workers:** Implement task-by-task. Steps use checkbox syntax.

**Goal:** 在 `skye_robot_driver` 内用 Gento Terminal CANFD + DM4310 MIT 协议对接 FACTR 夹爪 topic。

**Architecture:** `dm_mit`（纯协议）→ `DriverCore` terminal API → `GripperBridge` → `DriverNode` 订/发 FACTR topic；与臂共用同一 SDK link。

**Tech Stack:** C++17, ROS2 Humble, `libGentoSDK.so`, `sensor_msgs/JointState`

**Spec:** `docs/superpowers/specs/2026-08-07-dm-gripper-terminal-design.md`

## Global Constraints

- 不使用 Hand 24 API
- 不第二次 `FX_L1_System_Link`
- Topic 名：`/left|right_teleop_gripper/ctrl`、`/left|right_gripper/state`
- 语义：`position[0]` 归一化 `[0,1]`，0=开，1=闭

## File map

| File | Role |
|------|------|
| `include/.../dm_mit.hpp` + `src/dm_mit.cpp` | MIT 编解码 / terminal pack |
| `include/.../gripper_bridge.hpp` + `src/gripper_bridge.cpp` | 使能、目标、控制/反馈 tick |
| `driver_core.hpp/.cpp` | `terminal_set` / `terminal_get` |
| `driver_node.hpp/.cpp` | ROS 接线 + 定时器 |
| `config/skye_robot.yaml` | 夹爪参数 |
| `CMakeLists.txt` | 编入库/可执行 |
| `docs/ros_interfaces.md` / `dev_plan.md` | 文档对齐 |

## Tasks

### Task 1: dm_mit 协议

- [x] 实现 `float_to_uint` / `uint_to_float` / `encode_mit` / `decode_feedback` / `pack_terminal` / `unpack_terminal`（对齐 Thor `gripper_bridge.py`）

### Task 2: DriverCore terminal

- [x] `terminal_set(FXTerminalType, FXChnType, bytes)` / `terminal_get(...)`，持 `mutex_`，要求 `linked_`

### Task 3: GripperBridge

- [x] start/stop、set_target、tick_control、tick_feedback、Feedback 归一化

### Task 4: DriverNode + yaml

- [x] 参数、订阅、发布、定时器；析构/急停失能

### Task 5: 文档 + 编译

- [x] 更新接口文档；`colcon build --packages-select skye_robot_driver`
