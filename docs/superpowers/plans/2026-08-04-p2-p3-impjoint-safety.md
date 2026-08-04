# P2/P3 Skye Driver Implementation Plan

> **For agentic workers:** execute task-by-task; checkboxes track progress.

**Goal:** P2 真机冒烟可核验；遥操默认切到关节阻抗（对齐 Apex `set_mode=3`）；P3 安全层补齐并核验。

**Architecture:** `DriverCore` 支持 `imp_joint`（默认）与 `pd`；启动 Link→ResetError→Idle→目标模式；指令在 ImpJoint 用 `SetJointPosCmd`，PD 用 `SetJointPosPDCmd`。节点发布 `/gento/robot_state`，提供 `/gento/set_mode`（Int16，3=阻抗遥操）。

**Tech Stack:** ROS2 Humble, rclcpp, libGentoSDK 4.4.2, std_msgs/std_srvs

## Global Constraints

- ROS 单位 rad；SDK ° 仅在 driver_core
- 对外 `/gento/*`；控制流 BEST_EFFORT KeepLast(1)
- Apex `set_mode data:3` ≡ 本驱动 `imp_joint`（`FX_STATE_IMP_JOINT=2`），**不是** `FX_STATE_IMP_CART=3`
- 同一控制器仅一个 SDK 客户端

---

## Mode 映射（重要）

| 语境 | 值 | 含义 |
|------|-----|------|
| Apex `/control/set_mode` | **3** | 扭矩/阻抗遥操（其自身枚举） |
| Gento `FXStateType` | `IMP_JOINT=2` | 关节阻抗（遥操应对齐这个） |
| Gento `FXStateType` | `IMP_CART=3` | 笛卡尔阻抗（**不要**当关节遥操默认） |
| Gento `FXStateType` | `PD=11` | PD 模式 |
| 官方 ImpJoint 例程 | `SwitchToImpJointMode` + `SetJointPosCmd` | 权威路径 |

当前缺口：仅有 `SwitchToPDMode` + `SetJointPosPDCmd`，**无**显式 ImpJoint 切换与 Apex 风格 `set_mode`。

---

## P2 任务

- [x] Core：`ControlMode::{kImpJoint,kPd}`；`enter_mode`；`current_state`；ImpJoint 发 `SetJointPosCmd`
- [x] Node：默认 `control_mode:=imp_joint`；发布 `/gento/robot_state`；订 `/gento/set_mode`（3→ImpJoint）
- [x] Soft gains 可配（`impedance_stiffness/damping`）
- [x] `scripts/verify_p2_smoke.sh`
- [x] 更新 `dev_plan` / `teleop_sop` / `ros_interfaces`
- [ ] 现场：小幅单臂运动人工确认

## P3 任务

- [x] 启动时 `FX_L1_Config_SetPDCmdCycleTime`（250 Hz → 4 ms）
- [x] 独立 `acc_ratio` 参数
- [x] 限位 / max_delta / timeout 路径确认
- [x] `scripts/verify_p3_safety.sh` — **P3 VERIFY OK**

## 核验结果

- 编译：通过
- P3 脚本：通过（干跑）
- P2 模式链路：静态符号 + 源码确认 ImpJoint/`set_mode`
- P2 真机脚本：需用户本机执行（自动连真机被拦截）
```bash
cd skye_ros2_ws && source install/setup.bash && export ROS_DOMAIN_ID=20
ros2 launch skye_robot_driver skye_robot_driver.launch.py
# 另开终端：
./scripts/verify_p2_smoke.sh
ros2 topic pub --once /gento/set_mode std_msgs/msg/Int16 "{data: 3}"
```
