# P5 DM 夹爪（Terminal CANFD）设计

**日期：** 2026-08-07  
**状态：** 已定稿，进入实现

## 背景

- 大臂末端为 **达妙 DM4310** 开合夹爪，走臂末端 **CANFD**，经 Gento `FX_L1_Terminal_*` 透传。
- **不是** `SetHandPos` 24 维灵巧手 API。
- 小臂 FACTR 已发 `/left|right_teleop_gripper/ctrl`（`JointState` 1 维 `[0,1]`），并订 `/left|right_gripper/state`。
- 参考实现：Thor `~/wbc/robot/gripper_bridge.py`（MIT 编码 + terminal 透传）。

## 目标

在现有 `skye_robot_driver`（同一 SDK `link`）内补齐夹爪：

| 方向 | Topic | 类型 | 语义 |
|------|-------|------|------|
| 订 | `/left_teleop_gripper/ctrl`、`/right_teleop_gripper/ctrl` | `sensor_msgs/JointState` | `position[0] ∈ [0,1]`，0=开，1=闭 |
| 发 | `/left_gripper/state`、`/right_gripper/state` | `sensor_msgs/JointState` | `name=[gripper_joint]`；position 归一化 `[0,1]`；velocity/effort 来自电机反馈 |

## 架构

```text
DriverNode
  ├─ 臂：现有 joint_control / joint_states
  └─ GripperBridge（同进程）
        ├─ dm_mit：MIT 编解码 + terminal 打包
        └─ DriverCore::terminal_set/get（持同一 mutex_ / link）
              └─ FX_L1_Terminal_SetData/GetData(CANFD)
```

- **唯一 SDK 客户端**：夹爪不另开 `link`。
- **控制环**：定时器 `@ gripper_rate_hz`（默认 100）：目标→MIT→`terminal_set`；同环或短周期 `terminal_get` 解码反馈。
- **QoS**：指令 KeepLast(1)+BEST_EFFORT；状态 KeepLast(1)+RELIABLE（对齐臂状态给 FACTR）。

## 参数（yaml）

| 参数 | 默认 | 说明 |
|------|------|------|
| `enable_gripper` | `true` | 连接成功后使能夹爪桥 |
| `gripper_left_motor_id` | `1` | 左 CAN ID |
| `gripper_right_motor_id` | `2` | 右 CAN ID |
| `gripper_kp` / `gripper_kd` | `3.0` / `0.12` | MIT 增益 |
| `gripper_rate_hz` | `100.0` | 下发频率 |
| `gripper_pos_min` / `gripper_pos_max` | `0.0` / `1.6` | 电机角 (rad)，对应 0/1 |

## 非目标

- 灵巧手 Hand 24 API
- 独立夹爪进程 / 二次 `link`
- Apex `/control/gripperValueL` 兼容（可用外部 remap）

## 安全

- 析构 / `emergency_stop`：发失能帧（`FF…FD`）
- `enable_gripper:=false` 或 `connect_on_startup:=false` 时不启夹爪
- 目标值 clamp 到 `[0,1]`
