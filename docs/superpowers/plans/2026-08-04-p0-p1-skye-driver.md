# Skye Driver P0/P1 Implementation Plan

> **For agentic workers:** execute task-by-task; checkboxes track progress.

**Goal:** P0 编译+ABI 通过；P1 最小 PD 遥操闭环（对外 `/gento/*`）。

**Architecture:** `DriverCore`（SDK，°）+ `DriverNode`（ROS，rad）；控制流 QoS=`KeepLast(1)+BEST_EFFORT`；逻辑对齐 `marvin_ws/.../gento_robot_driver`，Position→PD。

**Tech Stack:** ROS2 Humble, rclcpp, libGentoSDK.so 4.4.2

## Global Constraints

- ROS 对外单位一律 rad；SDK 边界仅在 `driver_core` 内 °↔rad
- 对外 topic/service 兼容 `/gento/*`（节点内短名 + launch remap）
- 控制流：`KeepLast(1)` + `BEST_EFFORT`
- 同一控制器同时只允许一个 SDK 客户端
- 构建脚本避开 conda Python（`/usr/bin/python3`）

---

## P0 — 能编过

- [x] 修 `scripts/build.sh`（nounset + system python）
- [x] `colcon build --packages-select skye_robot_driver`
- [x] `scripts/check_sdk_abi.sh` + `ldd` 二进制链到 `libGentoSDK.so`
- [x] 更新 `docs/dev_plan.md` P0 勾选

**P0 结果：** 通过（2026-08-04）

**核验命令：**
```bash
cd skye_ros2_ws && ./scripts/build.sh && ./scripts/check_sdk_abi.sh
ldd install/skye_robot_driver/lib/skye_robot_driver/skye_robot_driver | grep Gento
```

---

## P1 — 最小 PD 闭环（v0.1）

### 文件

| 文件 | 职责 |
|------|------|
| `driver_core.hpp/.cpp` | Link / ResetError / Idle→PD / SetJointPosPDCmd / GetRT / hold / stop / E-Stop / Unlink；映射与单位换算 |
| `driver_node.hpp/.cpp` | 参数、pub/sub/service、定时状态、命令路径 |
| `config/skye_robot.yaml` | 对齐 gento 默认参数 + PD gains |
| `launch/skye_robot_driver.launch.py` | remap → `/gento/*` |
| `docs/dev_plan.md` | P1 勾选 |

### 任务

- [x] 扩展 `DriverCore`：`connect_and_enable`（Link→ResetError→Idle→PD）、`send_pd_position`、`hold`/`stop`/`estop`、映射与 °↔rad
- [x] 实现 `DriverNode`：对齐 marvin；控制流 QoS；三服务；`connect_on_startup`
- [x] yaml + launch remap → `/gento/*`
- [x] 重建并通过 P1 核验

### P1 核验（无真机）

```bash
./scripts/build.sh
./scripts/verify_p1_interfaces.sh
```

**P1 结果：** 通过（2026-08-04）— topics/services 已 remap；控制流 Reliability=`BEST_EFFORT`。

### 明确延后到 P3

- `SetPDCmdCycleTime` 与频率精调（P1 可用默认）
- 真机冒烟（P2）
