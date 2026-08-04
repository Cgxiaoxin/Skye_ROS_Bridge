# ROS 接口（与 marvin `/gento/*` 对齐）

单位：**rad**。控制流 QoS：`KeepLast(1)` + `BEST_EFFORT`。  
细节与开发顺序见 `dev_plan.md`。

## 控制模式（对齐 Gento `FXStateType`）

| mode | 名称 | SDK API |
|------|------|---------|
| **0** | IDLE | `SwitchToIdle` |
| **1** | POSITION | `SwitchToPositionMode` + `SetJointPosCmd` |
| **2** | IMP_JOINT | `SwitchToImpJointMode` + `SetJointPosCmd`（关节遥操推荐） |
| **3** | IMP_CART | `SwitchToImpCartMode`（笛卡尔阻抗；关节 topic 仅作过渡） |
| 11 | PD | `SwitchToPDMode` + `SetJointPosPDCmd`（可选） |

切换（**service，有回执**）：

```bash
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 1}"
ros2 service call /gento/set_mode skye_robot_driver/srv/SetMode "{mode: 2}"
ros2 topic echo --once /gento/robot_state --qos-reliability best_effort
```

注意：旧 Apex `/control/set_mode data:3` 表示其「阻抗遥操」枚举，对应本驱动 **mode=2（IMP_JOINT）**，不是 FX 的 IMP_CART=3。

## Topic

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/gento/joint_states` | `sensor_msgs/JointState` | 14 轴 rad / rad·s⁻¹ |
| 发布 | `/gento/robot_state` | `std_msgs/Int16MultiArray` | `[left_fx_state, right_fx_state]` |
| 订阅 | `/gento/left_joint_control` | `JointState` | 7 轴 position（rad） |
| 订阅 | `/gento/right_joint_control` | `JointState` | 7 轴 position（rad） |

## Service

| Service | 类型 | 说明 |
|---------|------|------|
| `/gento/set_mode` | `skye_robot_driver/srv/SetMode` | `mode`→切换；回 `success/message/left_state/right_state` |
| `/gento/hold_current` | `std_srvs/Trigger` | 保持当前位姿 |
| `/gento/stop_motion` | `std_srvs/Trigger` | 停止并进 IDLE |
| `/gento/emergency_stop` | `std_srvs/Trigger` | 软件急停 → IDLE |

## 关键参数

| 参数 | 默认 | 说明 |
|------|------|------|
| `control_mode` | `imp_joint` | 启动模式（idle/position/imp_joint/imp_cart/pd） |
| `cmd_cycle_time_ms` | `4` | `SetPDCmdCycleTime` |
| `impedance_stiffness/damping` | 100 / 10 | 关节阻抗 K/D |
| `cartesian_stiffness/damping` | SDK 例程默认 | 笛卡尔阻抗 K/D |
| `max_delta_per_cycle` | `0.05` | rad/周期 |
| `command_timeout_s` | `0.20` | 超时 hold |
