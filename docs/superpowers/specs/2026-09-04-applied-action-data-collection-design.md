# Applied Action 数采设计（driver 真源 + 轻量 recorder）

日期：2026-09-04  
状态：draft（待实现 plan）

## 背景与目标

遥操数采 / 模型训练需要的 **action 标签** 是「实际发给大臂与夹爪执行侧的命令」，不是 FACTR 入站的 `/gento/*_joint_control` 或扳机 `/left|right_teleop_gripper/ctrl`。

当前缺口：

- 关节：`last_command_`（relative/absolute 映射、signs、限位、单周期 Δ、超时 hold 之后）只存在于 `skye_robot_driver` 内存，成功后调用 `FX_L1_Runtime_SetJointPosCmd`，**无 ROS 对外发布**。
- 夹爪：`factr_to_motor_norm` + `close_limit` 后的电机目标经 `GripperBridge::set_target` / `tick_control` 下发，**无「applied」topic**。
- 现有 HITL `episode_recorder` 默认录的是预处理前的 control / ctrl，**不能**当训练标签真源。

目标（v1）：

1. 在 `skye_robot_driver` 内，于真正下发成功后发布 **4 路 applied `JointState`**。
2. 新增独立节点 **`skye_data_recorder`**：订阅 applied + 臂/夹爪状态，episode start/stop 写 **mcap**。
3. 在 `docs/ros_interfaces.md` 中明确区分「driver 新增 topic」与「数采 recorder 新节点」。

非目标（v1）：

- 不改 FACTR / 不重算映射（禁止方案「recorder 侧猜 applied」）。
- 不强制录相机；不强制接 HITL arbiter。
- 不做 pkl 导出；不保证跨机器、跨 domain 零丢帧。
- 不把 applied 与指令 topic 混用同一 BEST_EFFORT QoS。

## 架构

```
FACTR / 人遥操
  → /gento/*_joint_control、/left|right_teleop_gripper/ctrl
      → skye_robot_driver
            ├─ 预处理 → last_command_ / 夹爪电机目标
            ├─ SetJointPosCmd / 夹爪硬件下发
            └─ publish（数采真源）
                  /gento/left_joint_action_applied
                  /gento/right_joint_action_applied
                  /gento/left_gripper_action_applied
                  /gento/right_gripper_action_applied

skye_data_recorder（新节点，只订不控）
  → 订 4×applied + /gento/joint_states + /left|right_gripper/state
  → /skye/data_recorder/start|stop → mcap episode_XXXX/
```

原则：

- **真源只在 driver**：仅 SDK/夹爪下发成功（或与下发同源的控制 tick）后 publish。
- **recorder 只订不控**：不碰 SDK，不复制 relative 逻辑。
- **控制与数采 QoS 分离**：指令仍 BEST_EFFORT；applied 用 RELIABLE + 更大 depth，降低漏录。

## Topic 语义（`skye_robot_driver` 发布）

| Topic | 类型 | 语义 |
|-------|------|------|
| `/gento/left_joint_action_applied` | `sensor_msgs/JointState` | 左臂 7× `position`，**rad**，等于送入 `send_position` 的 `mapped` / `last_command_` |
| `/gento/right_joint_action_applied` | 同上 | 右臂同上 |
| `/gento/left_gripper_action_applied` | `JointState` | `name=[gripper_joint]`；`position[0]` **电机空间** 0=开、1=闭（invert + close_limit 之后） |
| `/gento/right_gripper_action_applied` | 同上 | 右爪同上 |

发布时机：

- 关节：`core_.send_position(...)` **返回 true** 之后；relative 与 abs 入口共用。
- 夹爪：与 `tick_control` **实际下发** 同源（每控制周期发布当前电机目标）；无反馈时仍发「将要/正在发的目标」，不得改回 FACTR 扳机语义。

其它约定：

- 关节 `name` 与反馈对齐：`l_j1..l_j7` / `r_j1..r_j7`。
- 单位：关节 **rad**（与 `/gento/joint_states` 一致）；不录 SDK 内部角度制。
- SDK reject / idle / 未连接：关节 **不** 发 applied（避免假标签）。
- **QoS**：`RELIABLE` + `KeepLast(depth)`，默认 depth **20**（参数可配，下限建议 ≥10）；`VOLATILE`。与 `/gento/*_joint_control` 的 `BEST_EFFORT`+depth1 刻意不同。

对比（避免误用）：

| Topic | 是什么 | 能否当训练 action |
|-------|--------|-------------------|
| `/gento/*_joint_control` | 预处理前入站 | 否（relative 下非大臂绝对目标） |
| `/gento/*_joint_action_applied` | 预处理后、进 SDK 的目标 | **是** |
| `/gento/*_joint_states` | 实机反馈 | observation，不是 action |
| `/left|right_teleop_gripper/ctrl` | FACTR 扳机 | 否（语义/invert 前） |
| `/gento/*_gripper_action_applied` | 电机空间实际下发 | **是** |
| `/left|right_gripper/state` | 夹爪反馈（FACTR 语义回映） | observation |

## 数采节点 `skye_data_recorder`

- 新包：`skye_ros2_ws/src/skye_data_recorder`；节点名 `skye_data_recorder`。
- **独立于** HITL `episode_recorder`（`/skye/recorder/*`），避免纯遥操数采依赖 HITL launch。
- 默认订阅：
  - `/gento/left_joint_action_applied`
  - `/gento/right_joint_action_applied`
  - `/gento/left_gripper_action_applied`
  - `/gento/right_gripper_action_applied`
  - `/gento/joint_states`
  - `/left_gripper/state`
  - `/right_gripper/state`
- 订阅 applied：QoS 与 driver 匹配（RELIABLE + depth≥20）。
- 落盘：rosbag2 **mcap**；仅 episode active 时 `write`。
- 服务：
  - `/skye/data_recorder/start` — `std_srvs/Trigger`；成功消息含 `episode_XXXX` 路径
  - `/skye/data_recorder/stop` — `std_srvs/Trigger`
- 参数：`output_dir`、`topics`、`storage_id`（默认 `mcap`）、`applied_qos_depth`（默认 20）。

防丢包 / 稳定性（v1 要求）：

- applied 专用 RELIABLE + KeepLast(≥10，默认 20)。
- recorder 与 driver **同机、同 `ROS_DOMAIN_ID`**。
- 未 start 不写盘；重复 start 失败回执；stop 关闭 writer。
- 文档注明：不保证每一控制周期 100% 落盘；同机 RELIABLE 下遥操数采可接受。

## 错误处理

| 情况 | 行为 |
|------|------|
| 关节 `send_position` 失败 | 不发 `*_joint_action_applied`；打 error 日志（现有） |
| 夹爪未 enable | 不发 gripper applied |
| mcap 插件缺失 | start 失败，明确提示安装 `ros-humble-rosbag2-storage-mcap` |
| 录制中再次 start | `success=false`，`recording already active` |
| 未录制时 stop | `success=false`，`recording is not active` |

## 测试计划

1. 单测 / 节点测：mock 或 bench 上 `send_position` 成功后 applied 与输入映射结果一致（relative 一帧）。
2. `ros2 topic echo` applied：遥操时有流量；idle 无关节 applied。
3. QoS：`ros2 topic info -v` 确认 applied 为 RELIABLE、depth≥10。
4. recorder：start → 动臂/夹爪 → stop → mcap 内含 4×applied + joint_states + gripper state。
5. 对比：同窗口内 `*_joint_control` 与 `*_joint_action_applied` **不等价**（relative 下可观测差异）。

## 文档交付

- 更新 `docs/ros_interfaces.md`：
  - Topic 表增加 4 路 applied，注释写清「driver 发布 / 数采真源 / 非入站 control」。
  - 新增 **「遥操数采 `skye_data_recorder`」** 小节：节点、默认订阅、start/stop、与 HITL recorder 的区别。
- 本设计文档：`docs/superpowers/specs/2026-09-04-applied-action-data-collection-design.md`。

## 实现顺序（供 writing-plans）

1. driver：关节 applied publish（成功路径 + QoS + remap 如需）。
2. driver：夹爪 applied publish（tick 下发路径 + 电机语义）。
3. 新包节点 `skye_data_recorder` + launch。
4. 文档 `ros_interfaces.md` 已同步；补验证脚本或手册命令。
5. 现场：同机遥操一集 mcap 抽检。
