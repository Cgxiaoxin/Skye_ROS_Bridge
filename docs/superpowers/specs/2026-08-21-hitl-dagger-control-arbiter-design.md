# HITL DAgger：control_arbiter + 数采接入设计

**日期：** 2026-08-21  
**状态：** 设计已审阅；实现计划见 [`docs/superpowers/plans/2026-08-21-hitl-dagger-control-arbiter.md`](../plans/2026-08-21-hitl-dagger-control-arbiter.md)  
**前置方案：** [`docs/HITL_DAgger_集成方案.md`](../../HITL_DAgger_集成方案.md)  
**仓库落点：** 本仓库新建 ROS2 包 `skye_hitl_dagger`（方案 A），不另起平行工程

---

## 1. 目标与非目标

### 目标

- 在现有 FACTR ↔ `/gento/*` ↔ `skye_robot_driver` 链路上插入 **控制权仲裁**，支持策略自主 + 人接管（HG-DAgger / Sirius 式数采）。
- 纯遥操场景 **零性能回退**：`hitl_enable:=false` 时不启动仲裁，FACTR 仍直连 `/gento/*`。
- 记录 mcap（可含 `control_mode` / `source` / `policy_version`），与同事数采管线对齐；不强制 resample 到 30 Hz。

### 非目标（本 spec 不做）

- VLA 推理进程实现（独立仓库/进程，只约定 `/skye/policy_action`）。
- 训练侧加权聚合脚本（P6.5，另开 plan）。
- 接管硬件真实按键 / 反应延迟 N 标定（v1 用键盘；N 后置）。
- 用字母键绑定 `/gento/emergency_stop`（急停仍走现有流程）。

---

## 2. 架构

### 2.1 纯遥操（HITL off）— 必须与现网一致

```text
factr_teleop_{left,right}
  → /gento/{left,right}_joint_control
  → skye_robot_driver（relative 映射 + 安全）→ SDK
```

`hitl_enable:=false`：不启动 `control_arbiter` / HITL recorder；launch remap 保持现状。

### 2.2 HITL on

```text
VLA 推理 ──► /skye/policy_action          (PolicyActionChunk)
FACTR    ──► /skye/teleop_action_*        (关节；夹爪仍可走现有 gripper topic 或经 arbiter)
键盘 q/w ──► 内部 / 轻量 trigger topic
                    │
                    ▼
            control_arbiter
              · 状态机选源
              · chunk 按 dt 展开；无新包 hold 末步
              · 接管：SYNC → HUMAN；交还：→ AUTONOMOUS
                    │
                    ▼
     /gento/{left,right}_joint_control   （唯一执行写入口）
     /left|right_teleop_gripper/ctrl     （AUTONOMOUS 时由 arbiter 写电机语义）
                    │
                    ▼
            skye_robot_driver
                    │
                    ▼
            episode_recorder（纯订阅 → mcap）
```

**硬约束：** VLA **不得**直连 `/gento/*_joint_control`，否则绕过仲裁，且会触发 driver 的 `relative` 映射把绝对策略角解错。

### 2.3 包边界

| 组件 | 位置 |
|------|------|
| `control_arbiter`、键盘桥、msg、launch | 新包 `skye_ros2_ws/src/skye_hitl_dagger` |
| `episode_recorder`（mcap） | 同包（可 Python） |
| `skye_robot_driver` | 最小改动：支持「来自 arbiter 的绝对角透传」（见 §5） |
| FACTR 二进制 | 不改；只改 HITL launch 的 remap |

---

## 3. 控制模式状态机

```text
                q（接管）
AUTONOMOUS ─────────────────► HANDOVER_SYNC ──(对齐完成)──► HUMAN
    ▲                                                         │
    │                         w（交还）                        │
    ◄─────────────────────────────────────────────────────────┘
```

| 状态 | 向 `/gento/*` 写什么 |
|------|----------------------|
| `AUTONOMOUS` | 当前 chunk 时间轴上的绝对关节目标；夹爪 100 Hz 插值 |
| `HANDOVER_SYNC` | 大臂 hold 当前目标；发 FACTR sync（`/mode/switch_sync`）；等对齐 |
| `HUMAN` | 透传 `/skye/teleop_action_*`（最新帧）；夹爪走 FACTR 现有路径 + driver invert |
| （可选过渡） | v1 可不做插值混合；SYNC 对齐后再放人，降低跳变风险 |

**交还策略：** 做完任务再还（策略 B）。`w` 仅手动切回 `AUTONOMOUS`，不自动交还。

**无新 chunk：** hold **该 chunk 最后一步**目标，打告警；**不**自动切 HUMAN。切人只靠 `q`。

---

## 4. 键盘（v1）

仅两个功能键，不占用多余键：

| 键 | 作用 |
|----|------|
| `q` | 请求接管：`AUTONOMOUS` → `HANDOVER_SYNC` → `HUMAN` |
| `w` | 交还：`HUMAN` → `AUTONOMOUS` |

- **不**使用 `e`；急停继续用现有 `/gento/emergency_stop` 等流程。
- **不**占用 FACTR 的 `1/2/3`（SYNC / TELEOP / STOP）。HITL 键盘节点与 FACTR `keyboard_gripper` **并存**：`1/2/3` 仍管小臂模式；`q/w` 只管 arbiter 数据源。
- `HANDOVER_SYNC` 内由 arbiter（或配套脚本）发布 `/mode/switch_sync`，对齐后再 `/mode/switch_teleop`，再进入 `HUMAN` 透传。

实现：独立小节点读 stdin / `keyboard`，发 `std_msgs/String` 或 `Bool` 到 `/skye/intervention_cmd`（`data: "takeover"|"return"`）。

---

## 5. 动作表示与 driver 映射

### 5.1 线上执行口（不变）

| Topic | 类型 | 语义 |
|-------|------|------|
| `/gento/left_joint_control` | `sensor_msgs/JointState` | 7× position，**rad 绝对角目标** |
| `/gento/right_joint_control` | 同上 | 同上 |
| `/left\|right_teleop_gripper/ctrl` | `JointState` | 见夹爪语义 |

文档中的「`follower_cmd`」= 上述 `/gento/*` 对；**不**另造同名 topic。

### 5.2 策略绝对角 vs teleop relative（硬约束）

- Driver 默认 `teleop_mapping_mode: relative`：把收到的 `joint_control` 当**主臂角**做增量映射。
- 策略输出是**大臂绝对角** → 若原样进现网 relative，会错。

**v1 已锁定（实现 plan）：** AUTONOMOUS / hold 写 `/gento/{left,right}_joint_control_abs`（driver 新入口，跳过 relative，仍 clamp + `max_delta`）；HUMAN 仍写原 `/gento/{left,right}_joint_control`（relative）。abs 路径不得污染 teleop 的 `leader_ref` / `gento_ref`。

验收：固定 chunk 绝对角，大臂到位误差在安全阈值内，且不被 relative 扭曲。

### 5.3 Teleop 支路

HITL on：FACTR remap

- `/joint_control` → `/skye/teleop_action_left` / `_right`（或统一命名，launch 写清）
- 仅 `HUMAN` 时由 arbiter 转发到 `/gento/*`

---

## 6. `/skye/policy_action` 消息约定

**类型名（建议）：** `skye_hitl_dagger/msg/PolicyActionChunk`

| 字段 | 类型 | 约定 |
|------|------|------|
| `header` | `std_msgs/Header` | `stamp` = 该 chunk 对应的当前时刻（与 step0 对齐） |
| `policy_version` | `string` | 模型版本标识 |
| `chunk_size` | `uint32` | 固定 **16** |
| `dt` | `float64` | 步间隔（秒）；按实际到达使用，不假定名义 30 Hz |
| `left_joints` | `float64[16*7]` | 行优先 step0..15；顺序 `l_j1..l_j7`，与 `/gento/joint_states` 一致；**绝对角 rad** |
| `right_joints` | `float64[16*7]` | 同上 `r_j1..r_j7` |
| `left_gripper` | `float64[16]` | **大臂电机语义：0=开，1=闭**（不是 FACTR 扳机语义） |
| `right_gripper` | `float64[16]` | 同上 |

**Chunk 语义：**

- **step0 = 当前时刻**目标（不是“下一拍”）。
- 左右同包。
- QoS：`KeepLast(1)` + `BEST_EFFORT`。
- 播完 16 步仍无新包 → hold **step15**；可选 1–2 s 无包打告警；不自动切人。

**展开节奏（事件驱动，非固定 30 Hz 主钟）：**

- `AUTONOMOUS`：按 `header.stamp` + `i * dt` 取步；实际推理 27/45 Hz 都以**收到的包与 dt**为准。
- `HUMAN`：跟 teleop 回调，一帧一写；与策略频率无关。
- 夹爪：chunk 内有轨迹，执行侧独立 **~100 Hz** 插值下发（对齐 `gripper_rate_hz`）。

---

## 7. 夹爪语义

| 路径 | 语义 |
|------|------|
| FACTR → driver（HUMAN） | 扳机 1=开 0=闭；`gripper_invert:=true` → 电机 0=开 1=闭 |
| Policy chunk（AUTONOMOUS） | 直接 **电机空间 0=开 1=闭**；arbiter 写入 gripper ctrl 时 **不再二次 invert**（实现时用专用参数或旁路 invert） |

HITL on 时 FACTR 夹爪 topic 也须经 arbiter（或 AUTONOMOUS 下屏蔽 FACTR 夹爪写口），避免与策略夹爪双写抢占。

须与 VLA / 数采同事书面确认训练 label 与此一致。

---

## 8. Arbiter 输出与性能

### 8.1 选源逻辑（状态机 + 事件）

Arbiter 是状态机选源器，不是死锁名义 Hz 的定时器：

- 策略侧：跟 chunk 到达 + `dt` 时间轴。
- 人侧：跟 teleop 消息到达。
- 禁止在 arbiter 回调里写盘、重推理、阻塞 IO。

### 8.2 遥操性能硬约束

| 场景 | 要求 |
|------|------|
| `hitl_enable:=false` | 链路与今日 P4 相同；`topic hz` 与现网同量级 |
| HITL + `HUMAN` | 不把遥操降到策略频率；透传最新 teleop 帧 |
| 验收 | 文档写明对比命令：`ros2 topic hz /gento/left_joint_control` |

---

## 9. 数据记录（mcap）

- 节点：`episode_recorder`，**只订阅**，不进控制环。
- 格式：**mcap**（rosbag2）；可含字段/话题：`control_mode`、`source`、`policy_version`。
- **不**强制 resample 到 30 Hz；保留各话题原生时间戳（ROS 时间）。
- 建议至少录：`policy_action`、teleop 支路、`/gento/*_joint_control`、`/gento/joint_states`、gripper、`/skye/control_mode`、相机（与数采同事最终 topic 表对齐）。

离线标签（与总方案一致）：

1. 自主：`AUTONOMOUS`  
2. 人类：`HUMAN`  
3. 接管前缓冲：触发时刻前 N（后置标定）  
4. `HANDOVER_SYNC` 可单独标记或并入过渡  

本仓库旁路 `record_live_vis.py`（pkl）仅作调试兼容，正式 HITL 以 mcap 为准。

---

## 10. 话题一览（HITL on）

| Topic | 方向 | 说明 |
|-------|------|------|
| `/skye/policy_action` | VLA → arbiter | `PolicyActionChunk` |
| `/skye/teleop_action_left` / `_right` | FACTR → arbiter | `JointState` 7 轴 |
| `/skye/intervention_cmd` | 键盘 → arbiter | takeover / return |
| `/skye/control_mode` | arbiter → 全局 | 模式 + stamp |
| `/gento/{left,right}_joint_control` | arbiter → driver | 唯一关节执行口 |
| `/left\|right_teleop_gripper/ctrl` | arbiter 或 FACTR → driver | 见 §7 |

---

## 11. 错误处理

| 情况 | 行为 |
|------|------|
| Chunk 尺寸 ≠ 16 / 数组长度不符 | 拒收该包，打 error，保持上一有效目标 |
| AUTONOMOUS 无新 chunk | hold 末步 + warn |
| 接管中 SYNC 超时 | 保持 hold，warn；不盲目进 HUMAN（阈值可配） |
| 推理进程崩溃 | 同无 chunk；人按 `q` 接手 |
| 双源同时有数据 | 只按当前模式取一路，禁止叠加 |

---

## 12. 测试 / 验收（实现阶段）

1. **HITL off：** 与现网遥操无差异（频率 / 手感抽检）。  
2. **模拟 policy：** 发布固定 chunk，大臂按绝对角到位。  
3. **`q`：** SYNC 对齐后 HUMAN，主臂可改轨迹且无危险跳变。  
4. **`w`：** 回到 AUTONOMOUS，继续跟新 chunk。  
5. **断流：** 无 chunk → hold 末步，不自动切人。  
6. **mcap：** 抽查含 mode / source / version，时间戳可读。

---

## 13. 分阶段（与总方案 P6 对齐）

| 阶段 | 内容 |
|------|------|
| P6.1 | 包骨架 + 状态机 + `q/w` + 模拟两路输入（不接真 VLA） |
| P6.2 | SYNC 接管路径 + 绝对角透传；真机 teleop 切换连续性 |
| P6.3 | mcap recorder + 标签字段 |
| P6.4 | 接真 VLA `/skye/policy_action` 端到端 |
| P6.5 | 训练加权（另 plan） |

---

## 14. 决策记录（头脑风暴锁定）

| 项 | 决策 |
|----|------|
| 仓库 | 本仓库新包 `skye_hitl_dagger` |
| 接管 | 先 SYNC 再 HUMAN |
| 交还 | 做完再还；仅 `w` 手动 |
| 键盘 | 仅 `q` / `w`；不用 `e`；不占用 `1/2/3` |
| Chunk | 16；step0=当前时刻；绝对角；左右同包；夹爪在 chunk，执行 100 Hz |
| 夹爪策略语义 | 电机 0 开 1 闭 |
| 无 chunk | hold 末步；人切 |
| 记录 | mcap；含 mode/source/version；不强制 30 Hz resample |
| 性能 | 纯遥操 HITL off |
| Arbiter 时钟 | 事件驱动（chunk/teleop），不用固定 30 Hz 驱动人控 |

---

## 15. 仍待与外团队对齐（不挡 P6.1）

- VLA：按本 §6 发 `/skye/policy_action`（已约定方向）。  
- 数采：mcap 最终 topic 清单。  
- 反应延迟 N：后置实测。  
- driver 绝对透传：已锁定为 `/gento/*_joint_control_abs`（见实现 plan Task 6）。
