# Gento + factr 无 bridge 遥操接线设计

## 目标

在不使用 `skye_leader_bridge` 的前提下，恢复与旧同构遥操一致的操作语义：

- 键盘 `1`：小臂同步到大臂（factr 内置 `TELEOP_SYNCING → SYNCED`）
- 键盘 `2`：小臂遥操大臂（factr 内置 `TELEOP`，经 `/joint_control` 下发）
- 键盘 `3`：停止遥操输出

参数沿用原 `gento_leader_bridge.yaml` 中的映射与安全门控，但落到 `gento_robot_driver` 命令入口。

## 非目标

- 不新建安全监督器（`safe-bidirectional` / `safety-supervisor` 设计延后）
- 不启动 `skye_leader_bridge`
- 不隔离 factr 内置 `/mode/switch_*`
- 不做相对/离合器映射、不做人工 confirm Trigger

## 拓扑

```text
keyboard_gripper
  └─ /mode/switch_{sync,teleop,stop}
        └─ factr_teleop_left / factr_teleop_right

gento_robot_driver
  ├─ pub /gento/joint_states  (14 rad)
  └─ sub /gento/left_joint_control, /gento/right_joint_control

factr_teleop_* remaps
  ├─ /joint_state   ← /gento/joint_states     # sync 从臂反馈
  ├─ /joint_control → /gento/{left|right}_joint_control
  └─ /joint_move、夹爪话题保持现有命名
```

## 迁入驱动的原框架参数

左右臂共用（来自废弃的 `gento_leader_bridge.yaml`）：

```yaml
joint_order: [0, 1, 2, 3, 4, 5, 6]
joint_signs: [1, 1, 1, -1, 1, -1, -1]
joint_offsets: [0, 0, 0, 0, 0, 0, 0]
limits_min: [-3.1067, -2.01, -3.1067, -1.0472, -3.1067, -1.0472, -1.5708]
limits_max: [3.1067, 2.01, 3.1067, 2.53, 3.1067, 1.0472, 1.5708]
max_delta_per_cycle: 0.05
command_timeout_s: 0.20
```

命令处理顺序：长度/有限检查 → order/sign/offset → 限位拒绝（不静默夹紧） → 相对上一发布命令的步长限制 → SDK 下发。超时无新命令则执行受控保持。

## 驱动服务

| 服务 | 语义 |
|---|---|
| `/gento/hold_current` | 读当前反馈，把当前位置设为保持目标；允许后续新命令 |
| `/gento/stop_motion` | `StopTraj` + 切 idle；拒绝命令直到再次成功 `hold_current` 或受控重连就绪 |

## 启动

`start_gento_dual_arm_sync.sh` 增加：

1. 小臂容器内启动 `keyboard_gripper`（或主机等价节点）
2. 左右 `/joint_control` remap 到 `/gento/{left,right}_joint_control`
3. 保留 `/joint_state:=/gento/joint_states`（factr 内置 sync 需要）
4. 启动后可继续 `enable_position_sync` 调试路径；正式遥操以键盘 `1/2/3` 为准

## 验收

- 无硬件：映射符号、步长限制、超时、hold/stop 单元测试
- 有硬件：按 `1` 小臂跟随；按 `2` 后轻移小臂，大臂同向/符号符合表；按 `3` 停止新命令
