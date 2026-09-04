# ROS 接口（与 marvin `/gento/*` 对齐）

单位：**rad**。  
指令 QoS：`KeepLast(1)` + `BEST_EFFORT`。  
状态 QoS：`KeepLast(1)` + `RELIABLE`（供 FACTR sync）。  
数采 applied QoS：`KeepLast(≥10，默认 20)` + `RELIABLE`（仅 `*_action_applied`；与指令分离，降漏录）。  
细节与开发顺序见 `dev_plan.md`；FACTR 对接见 `小臂大臂启动步骤.md`。  
遥操 applied 数采设计：`docs/superpowers/specs/2026-09-04-applied-action-data-collection-design.md`。

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
| 发布 | `/left_gripper/state` | `JointState` | `name=[gripper_joint]`；归一化位/速/力矩（FACTR 语义回映；**observation**，非 applied action） |
| 发布 | `/right_gripper/state` | `JointState` | 同上 |
| 发布 | `/gento/left_joint_action_applied` | `JointState` | **数采真源（关节）**。`skye_robot_driver` 在预处理（relative/absolute、signs、限位、Δ限制）后、`SetJointPosCmd` **成功**时发布；7×`position` **rad** = 内存 `last_command_`。**不是**入站 `/gento/left_joint_control`。QoS：`RELIABLE` + `KeepLast(≥10，默认 20)`（与指令 BEST_EFFORT 分离，降漏录） |
| 发布 | `/gento/right_joint_action_applied` | `JointState` | 同上（右臂） |
| 发布 | `/gento/left_gripper_action_applied` | `JointState` | **数采真源（夹爪）**。电机空间 `position[0]`：0=张开、1=闭合（invert + `close_limit` 之后，与夹爪硬件下发同源）。**不是**扳机 `/left_teleop_gripper/ctrl`。QoS 同关节 applied |
| 发布 | `/gento/right_gripper_action_applied` | `JointState` | 同上（右爪） |

夹爪走 **Terminal CANFD + DM4310 MIT**（非 Hand 24；`orin` profile 可为 Robotiq）。参数见 `enable_gripper` / `gripper_*`。

> **数采注意：** 训练 action 请订 `*_action_applied`，不要订 `*_joint_control` / `*_teleop_gripper/ctrl`。状态用 `/gento/joint_states` 与 `/left|right_gripper/state`。设计见 `docs/superpowers/specs/2026-09-04-applied-action-data-collection-design.md`。

## 遥操数采节点 `skye_data_recorder`（新增）

独立于 HITL 的 `episode_recorder`（`/skye/recorder/*`）。本节点**只订阅、写 mcap，不控制机械臂**。

| 项 | 说明 |
|----|------|
| 节点名 | `skye_data_recorder` |
| 作用 | 把 driver 发布的 applied action + 臂/夹爪状态录成 episode mcap，供遥操数采 / 训练 |
| 落盘 | rosbag2 **mcap**；目录参数 `output_dir`，每集 `episode_XXXX/` |
| 与 HITL 区别 | HITL recorder 默认录预处理前的 control/ctrl；本节点默认录 **`*_action_applied` 真源** |

**默认订阅（可参数 `topics` 覆盖）：**

| Topic | 用途 |
|-------|------|
| `/gento/left_joint_action_applied` | 左臂 action 标签 |
| `/gento/right_joint_action_applied` | 右臂 action 标签 |
| `/gento/left_gripper_action_applied` | 左爪 action 标签（电机空间） |
| `/gento/right_gripper_action_applied` | 右爪 action 标签 |
| `/gento/joint_states` | 双臂 observation |
| `/left_gripper/state` | 左爪 observation |
| `/right_gripper/state` | 右爪 observation |

**服务：**

| Service | 类型 | 说明 |
|---------|------|------|
| `/skye/data_recorder/start` | `std_srvs/Trigger` | 开始新 episode；成功时 `message` 含路径 |
| `/skye/data_recorder/stop` | `std_srvs/Trigger` | 结束并关闭 bag |

订阅 applied 时须匹配 driver 的 **RELIABLE + 足够 depth**（参数 `applied_qos_depth`，默认 20）。建议与 `skye_robot_driver` **同机、同 `ROS_DOMAIN_ID`**。v1 默认不录相机。

## Service

| Service | 类型 | 说明 |
|---------|------|------|
| `/gento/set_mode` | `skye_robot_driver/srv/SetMode` | `mode`→切换；回 `success/message/left_state/right_state` |
| `/gento/hold_current` | `std_srvs/Trigger` | 保持当前位姿 |
| `/gento/stop_motion` | `std_srvs/Trigger` | 停止并进 IDLE |
| `/gento/emergency_stop` | `std_srvs/Trigger` | 软件急停 → IDLE |
| `/gento/set_motion_rates` | `skye_robot_driver/srv/SetMotionRates` | 左右 vel/acc ratio（1–100%）。对齐节点对齐前写 10/10，结束后写回 profile 默认（如 30/30） |

```bash
ros2 service call /gento/set_motion_rates skye_robot_driver/srv/SetMotionRates \
  "{left_vel_ratio: 10, left_acc_ratio: 10, right_vel_ratio: 10, right_acc_ratio: 10}"
```

## Follower align（FACTR sync 之后，`skye_follower_align`）

独立主机节点；FACTR Docker 内 `1/2/3` 不变。对齐方向：**大臂跟小臂**（绝对 `*_joint_control_abs`），对齐中 vel/acc 强制 10%。

| 方向 | Topic | 类型 | 说明 |
|------|-------|------|------|
| 订阅 | `/mode/align_follower` | `std_msgs/String` | `data: align_follower` 开始对齐（主机键盘 `s` 与 CLI 共用） |
| 订阅 | `/mode/align_cancel` | `std_msgs/String` | `data: align_cancel` 取消对齐（主机键盘 `x`） |
| 发布 | `/align/status` | `std_msgs/String` | `IDLE` / `ALIGNING` / `ALIGNED` / `TIMEOUT_WARN` |
| 订阅 | `/left_leader_arm/current_state` | `sensor_msgs/JointState` | 小臂左 7 轴（对齐目标源） |
| 订阅 | `/right_leader_arm/current_state` | `sensor_msgs/JointState` | 小臂右 7 轴 |
| 订阅 | `/gento/joint_states` | `sensor_msgs/JointState` | 大臂 14 轴反馈（误差计算） |
| 发布 | `/gento/left_joint_control_abs` | `sensor_msgs/JointState` | 对齐绝对命令（左） |
| 发布 | `/gento/right_joint_control_abs` | `sensor_msgs/JointState` | 对齐绝对命令（右） |

启动: `ROBOT_PROFILE=thor|orin ./scripts/start_follower_align.sh`（`enable_keyboard:=true` 时终端焦点按 `s`/`x`）。操作流程见 `docs/Thor_Orin_遥操启动.md`。

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
| `left/right_joint_signs` | `thor`：左右全 `+1`；`orin`：右 `[1,1,1,1,1,-1,-1]` | 由 `robot_profile` 叠加 `profiles/{profile}.yaml`；相对遥操 |
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
