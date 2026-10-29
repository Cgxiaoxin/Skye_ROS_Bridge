# Phase 1：同构遥操 HITL 启停（正式 vs 调试 · Thor / Orin）

配合 [`skye-hitl-dagger-refactor-plan.md`](skye-hitl-dagger-refactor-plan.md) §0.1 / Phase 1。  
**不要**与纯遥操 `start_teleop_m6_dual.launch.py` 同 Domain 混开。

## 机型必须一致

`robot_profile` 在三处必须是同一个值（`thor` 或 `orin`）：

| 组件 | 作用 | 差异要点 |
|------|------|----------|
| `skye_robot_driver` | 大臂 SDK / relative·abs 符号 | Thor：腕部全 +1，DM4310，IP `6.6.7.191`；Orin：右 J6/J7=-1，Robotiq，IP `6.6.7.190` |
| `skye_factr_teleop` HITL | 小臂 YAML / 串口 / leader signs | `config/` vs `config/orin/` |
| `control_arbiter` | abs 反变换用的 **driver** 7 轴 signs | `skye_hitl_dagger/config/profiles/{thor,orin}.yaml` |

不要把小臂 8 轴 `joint_signs` 抄进 arbiter；arbiter 只跟 **driver profile** 对齐。

可用环境变量统一默认：`export ROBOT_PROFILE=thor` 或 `orin`（driver / HITL arbiter launch 都会读）。

## Remap 契约

| 节点内部话题 | 纯遥操 | HITL（本 Phase） |
|--------------|--------|------------------|
| `/joint_control` | `/gento/{side}_joint_control` | `/skye/teleop_action_{side}` |
| `/gripper/ctrl` | `/{side}_teleop_gripper/ctrl` | `/skye/teleop_gripper_{side}` |
| `/joint_state` | `/gento/{side}_joint_states` | 同左（不变） |

## 启动顺序

### A. 正式（推荐 / 对外）

开发机 = 对应机型的 Thor 或 Orin；笔记本 = USB 小臂。

**Thor**

| 步骤 | 机器 | 命令 |
|------|------|------|
| 1 | Thor | `./scripts/start_skye_for_factr.sh thor` — **仅此处起 driver** |
| 2 | 笔记本 | `ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py robot_profile:=thor` |
| 3 | Thor | `ros2 launch skye_hitl_dagger hitl_dagger.launch.py robot_profile:=thor` |
| 4 | 任意 | 可选 `ros2 run skye_hitl_dagger pub_dummy_policy_chunk` |

**Orin**

| 步骤 | 机器 | 命令 |
|------|------|------|
| 1 | Orin | `./scripts/start_skye_for_factr.sh orin` |
| 2 | 笔记本 | `ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py robot_profile:=orin` |
| 3 | Orin | `ros2 launch skye_hitl_dagger hitl_dagger.launch.py robot_profile:=orin` |
| 4 | 任意 | 可选 dummy chunk |

两端 `ROS_DOMAIN_ID` / RMW 一致；笔记本**不起** `skye_robot_driver`。

### B. 单人调试（非契约）

同一台机临时起 driver + HITL 小臂 + arbiter 亦可。  
`robot_profile` 仍须与实机一致；离场前停 driver；**不得**与开发机再起第二路 SDK。

## 验收（图）

```bash
ros2 topic info --verbose /gento/left_joint_control
ros2 topic info --verbose /gento/left_joint_control_abs
ros2 topic info --verbose /skye/teleop_action_left
```

期望（HITL on）：

- `/skye/teleop_action_*`：发布者为 `skye_factr_teleop_*`
- `/gento/*_joint_control(_abs)`：策略/遥操相关发布者仅为 `control_arbiter`

Orin 另核：右臂 abs 方向与纯遥操一致（错 profile 时右腕会反）。

## 纯遥操回退

日常不用 Dagger：只起 `start_teleop_m6_dual.launch.py robot_profile:=thor|orin`，**不要**起 `*_hitl` 与 `hitl_dagger`。
