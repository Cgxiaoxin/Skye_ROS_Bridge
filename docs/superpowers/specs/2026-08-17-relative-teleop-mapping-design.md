# 相对增量关节遥操映射（skye_robot_driver）

## 问题

当前 P4 链路为**绝对位置映射**：`q_cmd = map(q_leader)`。若 sync 未完成就切 TELEOP，大臂会以 `max_delta_per_cycle` 追小臂绝对角，存在碰撞风险。

## 目标

- 默认改为**相对增量映射**：切 TELEOP 首帧大臂保持当前位，只跟手小臂相对位移。
- 保留 `absolute` 模式参数回退。
- 不改 `factr_teleop` 二进制。

## 公式

进入新遥操会话时捕获：

```text
q_leader_ref  ← 首帧小臂 7 轴 (rad)
q_gento_ref   ← 首帧大臂反馈 (rad)
```

每周期：

```text
q_cmd[i] = q_gento_ref[i] + sign[i] * (q_leader[order[i]] - q_leader_ref[order[i]])
```

首帧 delta=0 → `q_cmd = q_gento_ref`（大臂不动），与 sync 残差无关。

## 会话边界

以下事件结束当前会话，下一帧指令重新 capture ref：

- 首条指令 / `streaming=false` 后恢复（hold、timeout、stop、set_mode）
- 某一轴 `clamp_to_limits` 贴边时 **仅该轴 re-clutch**（吃掉超出行程，回来不猛追）

`limit_delta` 只做速度帽，不再整臂改写 ref。

## 安全链（保留）

`clamp_to_limits`（逐轴离合）→ `limit_delta` → SDK；`command_timeout_s` → hold。

## 配置

`skye_robot.yaml`:

```yaml
teleop_mapping_mode: relative   # relative | absolute
```

## 验收

- sync 未完成切 TELEOP：大臂首帧不追小臂绝对位。
- 小臂单轴 +0.1 rad：大臂同轴约 +0.1（符号一致）。
- 限幅裁剪后 ref 更新，无积累跳变。
