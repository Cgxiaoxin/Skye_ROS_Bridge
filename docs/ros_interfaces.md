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
| 发布 | `/gento/joint_states` | `sensor_msgs/JointState` | 14 轴 rad / rad·s⁻¹；HITL/录包 |
| 发布 | `/gento/left_joint_states` | `JointState` | 7 轴左大臂；FACTR sync |
| 发布 | `/gento/right_joint_states` | `JointState` | 7 轴右大臂；FACTR sync |
| 发布 | `/gento/robot_state` | `std_msgs/Int16MultiArray` | `[left_fx_state, right_fx_state]` |
| 订阅 | `/gento/left_joint_control` | `JointState` | 7 轴 position（rad） |
| 订阅 | `/gento/right_joint_control` | `JointState` | 7 轴 position（rad） |
| 订阅 | `/gento/left_joint_control_abs` | `JointState` | HITL/策略绝对角入口；跳过 relative 映射，仍执行限位与单周期增量限制 |
| 订阅 | `/gento/right_joint_control_abs` | `JointState` | 同上 |
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
| `robot_profile` | `thor` | launch 参数 / 环境变量 `ROBOT_PROFILE`：`thor`（DM4310 夹爪）或 `orin`（Robotiq Hand-E + 右 J6/J7 signs）。叠加 `config/profiles/{profile}.yaml`；见 `docs/小臂大臂启动步骤.md` |
| `control_mode` | `imp_joint` | 启动模式（idle/position/imp_joint/imp_cart/pd） |
| `cmd_cycle_time_ms` | `4` | `SetPDCmdCycleTime` |
| `impedance_stiffness/damping` | 100 / 10 | 关节阻抗 K/D |
| `cartesian_stiffness/damping` | SDK 例程默认 | 笛卡尔阻抗 K/D |
| `max_delta_per_cycle` | `0.25` | rad/周期 |
| `teleop_mapping_mode` | `relative` | `relative`=增量遥操；`absolute`=旧绝对映射 |
| `command_timeout_s` | `0.50` | 超时 hold（按臂独立，不拖死对侧） |
| `left/right_joint_limits_*` | 见 yaml | 超限逐轴 clamp。J4 大臂 URDF `[-2.5307, 1.0472]`（−145°~+60°） |
| `left/right_joint_signs` | 左全 `+1`；右 `[1,1,1,1,1,-1,-1]` | 相对遥操；右 J6/J7 取反 |
| `enable_gripper` | `true` | 同进程夹爪桥；`ros2 param set` 不会停已创建的定时器 |
| `gripper_left_motor_id` / `gripper_right_motor_id` | `1` / `2` | 左 ARM0+ID1；右 ARM1+ID2 |
| `gripper_invert` | `true` | FACTR 扳机 1=开/0=闭 → 电机 0=开/1=闭；state 同样反回去给 FACTR |
| `gripper_close_limit` | `0.93` | 电机空间闭合上限，左右相同。FACTR 按下(0)经 invert 后不会发到 1.0 |
| `gripper_rate_hz` | `100.0` | 夹爪控制/状态发布频率 |
| `gripper_feedback_timeout_ms` | `1` | 运行时 `terminal_set/get` 超时；夹爪与关节分线程，但 SDK mutex 仍串行，必须短 |

`*_joint_control_abs` 是策略/HITL 的绝对关节角入口；遥操继续使用原
`*_joint_control` relative 路径。绝对路径不会更新 teleop 的
`leader_ref` / `gento_ref`。

## HITL DAgger（`hitl_enable:=true`）

无真机验收：`skye_ros2_ws/scripts/verify_hitl_p61_interfaces.sh`（起 arbiter、假 chunk、
`takeover`、查 `/skye/control_mode` 与 abs topic）。

| Topic | 类型 | 说明 |
|-------|------|------|
| `/skye/policy_action` | `skye_hitl_dagger/msg/PolicyActionChunk` | VLA/假策略 → arbiter；16 步绝对角 chunk |
| `/skye/teleop_action_left` / `_right` | `sensor_msgs/JointState` | FACTR 遥操 → arbiter（仅 HUMAN 转发到 `/gento/*`） |
| `/skye/teleop_gripper_left` / `_right` | `sensor_msgs/JointState` | FACTR 夹爪 → arbiter |
| `/skye/intervention_cmd` | `std_msgs/String` | `takeover` / `return`（键盘 `q`/`w`） |
| `/skye/control_mode` | `skye_hitl_dagger/msg/ControlMode` | `mode`=`AUTONOMOUS`\|`HANDOVER_SYNC`\|`HUMAN`；含 `source`/`policy_version` |
| `/skye/recorder/start` / `stop` | `std_srvs/Trigger` | mcap episode 录制（P6.3） |

AUTONOMOUS / hold：`control_arbiter` 写 `/gento/*_joint_control_abs`；HUMAN 写
`/gento/*_joint_control`（relative）。设计细节见
`docs/superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md`。
