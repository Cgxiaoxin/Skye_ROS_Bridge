# Follower Align After Sync — Design

**日期:** 2026-09-04  
**状态:** 已实现（软件）；待 Thor/Orin 实机 HW 验收  
**相关:** FACTR `1` sync 后小臂 GC 不足导致位姿残差；相对遥操前需大臂跟小臂绝对对齐

## 1. 问题

FACTR 键 `1` 让小臂去跟大臂。小臂重力补偿偏弱时，sync 报完成（或肉眼仍偏）后两侧关节仍不准。直接 `2` 进相对遥操时，残差会留在会话基线里，手感/安全都差。

## 2. 目标

在 FACTR sync 与相对遥操之间，增加**手动触发**的一步：

- **大臂跟随小臂**（绝对位置）
- 对齐过程 **vel/acc ratio 强制 10% / 10%**
- 主机 Docker **外**按键 `s`（及等价 ROS topic）
- 进阈值打 `ALIGNED`；超时只 `WARN`，**不禁止**随后手动开遥操（软提示）

非目标：不改闭源 FACTR sync/GC；不对齐阶段自动 `switch_teleop`；不对齐时动夹爪。

## 3. 操作流程

```text
主机 A: skye_robot_driver (robot_profile=thor|orin)
主机 B: follower_align (+ host keyboard)，Docker 外
Docker:  factr_teleop（1/2/3 不变）
```

1. 起大臂驱动 + 小臂 Docker（同 `ROS_DOMAIN_ID=21`、同 profile）  
2. Docker：`1` → FACTR sync  
3. 主机：焦点在键盘节点终端，按 **`s`**（或 pub `/mode/align_follower`）  
4. 对齐节点：ratio→10% → 读小臂角 → signs 映射 → `/gento/*_joint_control_abs` 慢速追  
5. 进阈值 → `ALIGNED`；超时 → `TIMEOUT_WARN`（仍可开遥操）  
6. hold + 恢复原 ratio（如 30%）  
7. Docker：`2` → 相对遥操  

## 4. 架构（推荐）

独立主机节点，不把状态机塞进 `skye_robot_driver` 主路径；驱动仅增加 ratio 服务。

| 组件 | 位置 | 职责 |
|------|------|------|
| `follower_align_node` | 主机 | 对齐状态机、abs 下发、阈值/超时、调 ratio / hold |
| `host_keyboard_align` | 主机（可与上合并） | `s`→align，`x`→cancel |
| `skye_robot_driver` | 主机 | 现有 abs / hold；**新增** `set_motion_ratios` |
| FACTR | Docker | 不变 |

放弃方案：对齐逻辑深嵌 driver（耦合大）；继续只靠小臂再 sync（不解决 GC）。

## 5. 接口

### 5.1 话题

| 名称 | 消息 | 说明 |
|------|------|------|
| `/mode/align_follower` | `std_msgs/String` `data: align_follower` | 开始对齐（`s` 与 CLI 共用） |
| `/mode/align_cancel` | `std_msgs/String` `data: align_cancel` | 取消（`x`） |
| `/{left,right}_leader_arm/current_state` | `JointState` | 小臂反馈（目标源） |
| `/gento/joint_states` | `JointState` | 大臂反馈（误差） |
| `/gento/{left,right}_joint_control_abs` | `JointState` | 大臂绝对命令 |
| `/align/status` | `std_msgs/String` | `IDLE` / `ALIGNING` / `ALIGNED` / `TIMEOUT_WARN` |

### 5.2 服务

| 名称 | 说明 |
|------|------|
| `/gento/hold_current` | 现有 Trigger；对齐结束/取消后调用 |
| `/gento/set_motion_ratios` | **新建**：左右 vel/acc（int 1–100）。对齐前写 10/10，结束后写回 |

急停 / `stop_motion`：对齐节点检测到后立即停发 abs，不抢急停语义。

### 5.3 参数（对齐节点）

| 参数 | 默认 | 说明 |
|------|------|------|
| `align_vel_ratio` / `align_acc_ratio` | 10 / 10 | 对齐强制速度 |
| `restore_vel_ratio` / `restore_acc_ratio` | 与驱动 yaml 一致（如 30） | 或开始前读出再写回 |
| `align_error_threshold_rad` | 0.05 | 每轴 \|误差\| 阈值 |
| `align_hold_frames` | 若干帧 | 连续满足才算 ALIGNED |
| `align_timeout_s` | 8–10 | 超时仅 WARN |
| `align_rate_hz` | 50–100 | abs 下发频率 |
| `left/right_joint_signs` | 跟 `robot_profile` | thor 全 +1；orin 右 J6/J7=-1 |

### 5.4 映射

与现网 absolute / signs 约定一致（offsets 默认 0）：

```text
q_big_cmd[i] = sign[i] * q_leader[i]
```

走 `*_joint_control_abs`，**不**走 relative，避免污染 teleop 会话基线。

## 6. 状态机与安全

```text
IDLE --align--> ALIGNING --阈值--> ALIGNED --> IDLE
                  |--timeout--> TIMEOUT_WARN --> IDLE
                  |--cancel/estop--> IDLE
```

- `ALIGNING` 中再次 `s`：**忽略**  
- 缺一侧 leader 反馈：**整次取消**，已动侧 hold（默认更安全）  
- 超时 / 成功：均 hold + 尽量恢复 ratio；恢复失败则保持 10% 并 ERROR  
- Docker 在对齐中按 `2`：**不拦截**，打 WARN（软策略）  
- 对齐**不**发夹爪命令  
- 若驱动 `max_enable_error` 仍挡 teleop：属现有门禁；文档说明可调阈值或该参数  

## 7. 测试与验收

**单测：** 映射、阈值/超时判定、状态机（二次 `s`、cancel）。  

**驱动：** `set_motion_ratios` 合法写入 / 非法拒绝；abs 仍受限位与 `max_delta`。  

**实机（Thor + Orin）：**  
`1` 后仍有偏差 → `s` 慢速靠近且 ratio=10% → `ALIGNED` 后恢复 ratio → `2` 正常；缩短 timeout 得 `TIMEOUT_WARN` 仍可 `2`；`x` 立即 hold；Orin 右腕方向正确。  

**文档：** `Thor_Orin_遥操启动.md` 流程改为 `1 → s → 2`；`ros_interfaces.md` 补接口。

## 8. 决策记录

| 项 | 选择 |
|----|------|
| 对齐方向 | 大臂跟小臂（绝对） |
| 速度 | 对齐中 vel/acc=10% |
| 触发 | 主机键盘 `s` + `/mode/align_follower` |
| 完成 | 阈值自动判定；超时软提示 |
| 未进阈值能否 teleop | 能（不硬门禁） |
| 键盘位置 | 主机 Docker 外（避开闭源 FACTR 键盘） |
| 实现落点 | 独立对齐节点 + driver ratio 服务 |
