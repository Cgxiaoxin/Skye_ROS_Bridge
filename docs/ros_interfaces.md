# ROS 接口（与 marvin `/gento/*` 对齐）

单位：**rad**。  
指令 QoS：`KeepLast(1)` + `BEST_EFFORT`。  
状态 QoS：`KeepLast(1)` + `RELIABLE`（供 FACTR sync）。  
细节与开发顺序见 `dev_plan.md`；FACTR 对接见 `小臂大臂启动步骤.md`。

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
ros2 topic echo --once /gento/robot_state --qos-reliability reliable
```

注意：旧 Apex `/control/set_mode data:3` 表示其「阻抗遥操」枚举，对应本驱动 **mode=2（IMP_JOINT）**，不是 FX 的 IMP_CART=3。

## Topic

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 发布 | `/gento/joint_states` | `sensor_msgs/JointState` | 14 轴 rad / rad·s⁻¹ |
| 发布 | `/gento/robot_state` | `std_msgs/Int16MultiArray` | `[left_fx_state, right_fx_state]` |
| 订阅 | `/gento/left_joint_control` | `JointState` | 7 轴 position（rad） |
| 订阅 | `/gento/right_joint_control` | `JointState` | 7 轴 position（rad） |
| 订阅 | `/left_teleop_gripper/ctrl` | `JointState` | 夹爪指令 `position[0]∈[0,1]`。`gripper_invert:=true`（默认）时按 FACTR 扳机：1=松开/开，0=按下/闭；内部再 `1-x` 到电机 0=开 1=闭 |
| 订阅 | `/right_teleop_gripper/ctrl` | `JointState` | 同上 |
| 发布 | `/left_gripper/state` | `JointState` | `name=[gripper_joint]`；归一化位/速/力矩 |
| 发布 | `/right_gripper/state` | `JointState` | 同上 |

夹爪走 **Terminal CANFD + DM4310 MIT**（非 Hand 24）。参数见 `enable_gripper` / `gripper_*`。

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
| `max_delta_per_cycle` | `0.25` | rad/周期 |
| `teleop_mapping_mode` | `relative` | `relative`=增量遥操；`absolute`=旧绝对映射 |
| `command_timeout_s` | `0.50` | 超时 hold（按臂独立，不拖死对侧） |
| `left/right_joint_limits_*` | 见 yaml | 超限逐轴 clamp。J4 大臂 URDF `[-2.5307, 1.0472]`（−145°~+60°）；`signs` 全 `+1` |
| `enable_gripper` | `true` | 同进程夹爪桥；`ros2 param set` 不会停已创建的定时器 |
| `gripper_left_motor_id` / `gripper_right_motor_id` | `1` / `2` | 左 ARM0+ID1；右 ARM1+ID2 |
| `gripper_invert` | `true` | FACTR 扳机 1=开/0=闭 → 电机 0=开/1=闭；state 同样反回去给 FACTR |
| `gripper_close_limit` | `0.93` | 电机空间闭合上限，左右相同。FACTR 按下(0)经 invert 后不会发到 1.0 |
| `gripper_rate_hz` | `100.0` | 夹爪控制/状态发布频率 |
| `gripper_feedback_timeout_ms` | `1` | 运行时 `terminal_set/get` 超时；夹爪与关节分线程，但 SDK mutex 仍串行，必须短 |
