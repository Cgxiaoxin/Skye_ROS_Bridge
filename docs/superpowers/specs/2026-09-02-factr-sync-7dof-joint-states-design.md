# FACTR sync 7-DOF 分侧 joint_states（workaround）

## 问题

FACTR 闭源 bug：`follower_joint_offset` 在 **sync 路径**未生效。左右两个小臂节点均订阅同一 14 轴 `/gento/joint_states` 时，sync 目标均取 `position[0:7]`（左大臂），右臂无法对齐右大臂零位。

重力补偿等路径能正确识别 yaml 中 `follower_joint_offset: 0 / 7`。

## 目标

- 不依赖 relay 节点；由 `skye_robot_driver` **同周期**发布 14 轴 + 左右 7 轴。
- 每个 factr 节点 **仅一条** `/joint_state` 订阅（sync 与 gravity 共用），禁止分订两路。
- yaml `follower_joint_offset` **不变**（左 0、右 7），与 14-DOF `marvin_m6.urdf` 段号一致。
- 保留 `/gento/joint_states`（14 轴）供 HITL、arbiter、录包。

## 方案（推荐，已选）

```text
skye_robot_driver (一次 GetRT)
  → /gento/joint_states           # 14 轴 [l_j1..l_j7, r_j1..r_j7]
  → /gento/left_joint_states      # 7 轴 [l_j1..l_j7]
  → /gento/right_joint_states     # 7 轴 [r_j1..r_j7]

factr_teleop_left   /joint_state → /gento/left_joint_states   (offset=0)
factr_teleop_right  /joint_state → /gento/right_joint_states  (offset=7)
```

FACTR sync 始终读 `position[0:7]`；7 轴 topic 已预切片，右臂 sync 正确。  
重力路径用 offset 写入 14-DOF URDF 的对应段（右臂 offset=7 → `q[7:14]`），与现网行为一致。

## 未选方案

| 方案 | 弃用原因 |
|------|----------|
| relay 切片 topic | 多节点、不对称、额外延迟 |
| sync/gravity 分订 14 轴与 7 轴 | 两路 q 可能不同步 → 动力学隐藏 bug |
| 右臂 yaml offset 改 0 | 会把 7 轴值灌进 URDF 左臂段 |

## 改动范围

1. `skye_robot_driver`: `driver_node.hpp/cpp` 增加 `left/right_joint_states` publisher；`publish_state()` 同 stamp 三发。
2. `skye_robot_driver.launch.py`: remap → `/gento/left_joint_states`、`/gento/right_joint_states`。
3. `start_teleop_m6_dual_gento.launch.py` + HITL 版：factr `/joint_state` 改订分侧 7 轴 topic。
4. 文档：`dev_plan.md`、`ros_interfaces.md`、`小臂大臂启动步骤.md`。

**不改：** `grav_comp_m6_{left,right}.yaml` 的 `follower_joint_offset`；HITL/arbiter 仍用 14 轴。

## 验收

1. `ros2 topic echo /gento/left_joint_states --once` → 7 轴，与 14 轴 `[0:7]` 一致。
2. `ros2 topic echo /gento/right_joint_states --once` → 7 轴，与 14 轴 `[7:14]` 一致。
3. Docker 内 sync：左/右 factr 日志「同步目标位置」分别对应左/右大臂（右大臂≈0 时右小臂 J5 不再≈150°）。
4. 启动日志 gravity 仍显示 left offset=0、right offset=7。
5. HITL：`/gento/joint_states` 仍为 14 轴，录包维度不变。

## 后续

FACTR 厂商修复 sync offset 后，可恢复 factr 双订 14 轴 + yaml offset，7 轴 topic 可保留作兼容。
