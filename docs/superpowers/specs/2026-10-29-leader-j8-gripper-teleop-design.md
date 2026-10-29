# Leader J8 → 大臂夹爪遥操链路设计

**日期：** 2026-10-29  
**状态：** 已定稿，待实现计划  
**分支语境：** `skye_factr_teleop` reboot（主机包），对接已有 `skye_robot_driver` GripperBridge

## 背景

双臂关节遥操（J1–J7 → `/gento/*_joint_control`）已可用。小臂扳机 J8 在 leader 侧已参与本地限位与 sync `gripper_setpoint`，但 **未发布** 到大臂夹爪话题，末端夹爪不跟随。

main 旧 FACTR 路径已通过 `/left|right_teleop_gripper/ctrl` 打通；驱动侧 `GripperBridge`（Thor=`dm4310`，Orin=`robotiq`）与 invert 语义已就绪。reboot 包 phase0 契约已规定节点内 `/gripper/ctrl` 与 remap，但 `ros_driver` / launch **尚未实现**。

## 目标

在 `skye_factr_teleop` 中补齐 **命令单向链路**（不做力反馈）：

| 方向 | 节点内 | 左系统话题 | 右系统话题 | 类型 / QoS | 内容 |
|------|--------|------------|------------|------------|------|
| 发布 | `/gripper/ctrl` | `/left_teleop_gripper/ctrl` | `/right_teleop_gripper/ctrl` | `sensor_msgs/JointState`，`command_qos` | `position` 长度 1，`[0,1]`；FACTR：`1=开`，`0=闭` |

成功标准：TELEOP 下扳动 J8，对应侧 `/left|right_teleop_gripper/ctrl` 有更新，大臂夹爪开合跟随（驱动 `enable_gripper` 且硬件正常）。

## 已确认决策

| 项 | 选择 |
|----|------|
| 范围 | 仅命令链路；不做夹爪 / 7 轴力反馈 |
| 发布模式 | **仅 `TELEOP`**（A1）；IDLE / sync 相关模式不发 |
| 映射 | **B1**：用现有 `controller.limit_barrier_control.gripper.limit_min/max` 线性 clamp |
| 实现形态 | **方案 1**：`ros_driver` TELEOP tick 发布 + launch remap；纯函数可单测 |
| 配置分层 | 不新增 yaml 字段；继续 Thor / Orin 现有 grav_comp 与驱动 `profiles/*` |

说明：当前 reboot 包中 `force_feedback_control.joint|gripper` 仅配置解析，运行时未消费；本设计不改变该状态。

## 架构

```text
SkyeFactrTeleopNode (side=left|right)
  control tick @ controller.frequency
    └─ mode == TELEOP:
         q8 = logical gripper_position (after joint_signs)
         norm = clip((q8 - limit_min) / (limit_max - limit_min), 0, 1)
         publish JointState(position=[norm]) on /gripper/ctrl
              │
              ▼ launch remap
         /left_teleop_gripper/ctrl  or  /right_teleop_gripper/ctrl
              │
              ▼
         skye_robot_driver::handle_gripper_command
              └─ factr_to_motor_norm (gripper_*_invert) → GripperBridge
```

- 映射在 **逻辑角** 上计算（与 barrier / sync 同一套 `gripper_position`）。
- 假定逻辑角越大越开；物理方向由 `dynamixel.joint_signs[7]` 与驱动 `gripper_left|right_invert` 承担（与现网一致，默认 invert=true）。
- 分母 `(limit_max - limit_min)` 必须为正（配置校验已保证 barrier 上下限有序）。

## 组件改动

### 1. 映射纯函数

新增小函数（建议放在 `control_core.py` 或同级工具模块），输入 `q, limit_min, limit_max`，输出 `[0,1]` float。有限 `q`：线性映射后 clamp；非有限 `q`：本拍跳过发布并 warning（不因此把整臂 TELEOP fault 掉）。`limit_max <= limit_min`：视为配置错误，进入 fault 路径并停止发夹爪。

### 2. `ros_driver.py`

- 创建 `/gripper/ctrl` 发布者（`command_qos`，与 `/joint_control` 同档）。
- 在 TELEOP 且本周期已有有效 leader 状态时发布；非 TELEOP **不发布**（不主动清零）。
- 不订阅 `/gripper/state`（本阶段非目标）。

### 3. Launch remap

`launch_support.py` 为每侧增加：

```text
("/gripper/ctrl", f"/{side}_teleop_gripper/ctrl")
```

单臂 / 双臂 teleop launch 共用该 helper。更新 `test_launch.py` 期望 remap 列表。

### 4. 配置（Thor / Orin）

| Profile | Leader 配置路径 | 夹爪限位来源 | 大臂夹爪类型（已有） |
|---------|-----------------|--------------|----------------------|
| Thor | `config/grav_comp_m6_{left,right}.yaml` | `limit_barrier_control.gripper.limit_*` | `profiles/thor.yaml` → dm4310 |
| Orin | `config/orin/grav_comp_m6_{left,right}.yaml` | 同上 | `profiles/orin.yaml` → robotiq |

本任务 **不修改** 驱动 gripper 参数结构；若真机开合方向反了，先查 `joint_signs[7]` / `gripper_*_invert`，再考虑后续加独立 invert 开关（非本版范围）。

## 模式与安全

- 仅 TELEOP 发令；sync 阶段 leader J8 仍可走本地 setpoint，但 **不驱动** 大臂夹爪。
- 进入 TELEOP 后第一拍即按当前 J8 发令（避免卡在陈旧目标）。
- 节点退出 / stop：不再发夹爪命令；驱动保持最后目标（与现有关节相对控制停发行为同类）。
- `limit_min == limit_max` 不应出现；若运行时读到则打 fault / 停止发夹爪（实现计划写清）。

## 测试

- 单元：映射边界（min→0、max→1、中点、越界 clamp）。
- `test_ros_driver`：TELEOP 发布 `position=[…]`；IDLE/SYNC 不发布（或发布计数为 0）。
- `test_launch`：左右 remap 含 gripper。
- 真机（操作员）：Thor 双臂 TELEOP 扳 J8 看末端；Orin 同测（Robotiq）。

## 非目标

- 夹爪力反馈到扳机 / 7 轴力矩反馈
- HITL arbiter 路径（`/skye/teleop_gripper_*`）
- 改 `skye_robot_driver` GripperBridge / 电机协议
- Docker / `marvin_ws` 旧 FACTR 恢复
- 新增独立 `gripper_teleop` yaml 段或 `limit_setpoint` 参与映射

## 文档跟随（实现时）

- `docs/marvin_dev/双臂启动遥操.md` / 单臂文档：一句说明 TELEOP 后 J8 驱动末端夹爪，并提示 latency_timer。
- 不强制改 phase0 契约正文（已描述该话题）；实现后可在 checklist 加验收项。
