# 同构遥操 → HITL DAgger 改造方案（dev-Dagger）

> **分支：** `dev-Dagger`（基于 `integrate/dev-reboot-rebased`，同构遥操已通后的改造线）  
> **日期：** 2026-10-29  
> **状态：** 方案定稿待实施；纯遥操行为不得回退（HITL off 时与现网一致）  
> **对照：** 千寻「常驻执行平面 + 标准 chunk 口」思路；topic/service **以本仓现网为准**，不照搬 `/mx_*`  
> **已有实现：** `skye_hitl_dagger`（arbiter / PolicyActionChunk / abs / split takeover）可复用，重点是接入同构遥操并固化部署

---

## 0. 一句话结论

**策略只交口 A（`PolicyActionChunk`）；大臂唯一写者是 `control_arbiter` → `skye_robot_driver`；同构遥操 `skye_factr_teleop` 在 HITL on 时改经 `/skye/teleop_action_*`，禁止直写 `/gento/*`。**

**正式部署：执行平面（driver + arbiter）只跑在 Thor/Orin；笔记本不跑 driver。**  
本地起 driver 仅允许单人调试，**不是**对外契约。

同机跑模型时：VLA → `/skye/policy_action` → arbiter 即可，仍禁止同机直连 SDK / 直写 `/gento/*`。

---

## 0.1 部署拓扑：正式 vs 调试（必读）

`skye_robot_driver` 做的事是：订 `/gento/*` → 调 `libGentoSDK` → UDP 打控制器。  
技术上笔记本也能跑（同网 + 对应架构 SDK），但若把它当正式方案：

- 笔记本关盖/离场 → 别人无法只连口 A 动臂（公共执行平面绑在人身上）  
- 有人再在 Thor 起一份 driver → **双 SDK** 抢控制器  

因此约定如下。

### 正式部署（对外 / 多人 / Dagger）

```text
[笔记本 · 遥操机]                    [Thor / Orin · 执行平面 · 常驻]
 skye_factr_teleop (USB)              control_arbiter
 Operator UI / 干预                   skye_robot_driver  →  SDK  →  控制器
        │  ROS Domain 一致                  ▲
        └──── /skye/teleop_action_* ────────┘
              /mode/* 、/teleop/state

[推理机 · 可同 Thor 可另机]
 只 pub /skye/policy_action（口 A）
 不装整仓、不起 driver、不写 /gento/*
```

| 机器 | 允许跑 | 禁止 |
|------|--------|------|
| **Thor / Orin** | `skye_robot_driver` + `control_arbiter`（+ 可选口 A gateway） | 第二路 SDK；策略直写 `/gento/*` |
| **笔记本** | `skye_factr_teleop`、Operator UI、干预桥 | **正式流程不起** `skye_robot_driver` |
| **推理机** | 只连口 A | SDK / `/gento/*_joint_control*` |

**代码不另开仓：** 仍是本仓库 `skye_ros2_ws` 里的 `skye_robot_driver`、`skye_hitl_dagger`；在 **开发机上 clone/同步后 colcon 编译并启动**。笔记本只需编/跑小臂与 UI，用同一 `ROS_DOMAIN_ID` 与开发机通信。

### 单人调试例外（非契约）

仅当你一人占机、明确不会有第二路 SDK 时，可在笔记本临时起 driver 联调。  
离场前必须停掉；不得写进「别人按标准下发」的启停说明。  
Dagger / 对外验收 **必须以 Thor/Orin 上的执行平面为准**。

### 与改造节奏的关系

- **Phase 1–3**：可先在「单人调试拓扑」把 remap / 仲裁图跑通（甚至同机起 driver+小臂），但 launch/文档须标清「调试专用」。  
- **Phase 4**：必须把正式拓扑落成脚本与验收——笔记本卸 driver；会话前健康检查开发机上的 driver/arbiter。

---

## 1. 背景与动机

### 1.1 现状（As-Is）

| 能力 | 状态 |
|------|------|
| 同构双臂同步 + 关节遥操（`skye_factr_teleop`） | 阶段 7.5 已通；直写 `/gento/{left,right}_joint_control` |
| 大臂驱动（`skye_robot_driver`） | 现网常驻；消费 relative / abs |
| HITL 包（`skye_hitl_dagger`） | 代码已在；旧 Marvin FACTR + HITL remap 路径上冒烟过；**未接到同构遥操** |
| 口 A / arbiter 常驻 | 纯遥操图上通常不起；策略侧易绕过写 `/gento/*` 或 SDK |

### 1.2 要解决的问题

1. **策略输出契约**：任何人/任何机按标准丢 chunk，不装整仓/SDK。  
2. **唯一执行契约**：只有一套 `skye_robot_driver`；策略与遥操不得双写、不得第二路 SDK。  
3. **HITL DAgger**：AUTONOMOUS ↔ 接管（sync → enter teleop → HUMAN）↔ 交还；数采可打标。

### 1.3 非目标（本阶段不做）

- 重写 `control_arbiter` / `PolicyActionChunk` / ChunkPlayer / abs 符号反变换  
- 照搬千寻 `/mx_*` 命名或 `MCControlNode`  
- 训练侧加权聚合脚本（可后置）  
- 零 ROS 的 ZMQ/HTTP gateway（可选后置）

---

## 2. 目标架构（三平面）

```text
[任意策略 / 同机 VLA]                 [FACTR 笔记本 / USB]
 口 A: /skye/policy_action            skye_factr_teleop
 (PolicyActionChunk)                  /mode/switch_* 、/teleop/state
                                      HITL on: remap → /skye/teleop_action_*
           \                                   /
            ▼                                 ▼
         [control_arbiter]  ←—— 与 driver 同驻 Thor/Orin
              │  ChunkPlayer 展开
              │  AUTONOMOUS / HANDOVER_SYNC / HUMAN
              ▼
     /gento/{left,right}_joint_control(_abs)   ← 唯一写者 = arbiter（HITL on）
              │
              ▼
         [skye_robot_driver]
              │
              ▼
         /gento/*_joint_states 、*_action_applied 、robot_state
```

| 平面 | 跑什么 | 在哪（正式） |
|------|--------|----------------|
| **执行** | `skye_robot_driver` + `control_arbiter`（+ 可选 gateway） | **仅** Thor / Orin 常驻；见 §0.1 |
| **遥操 / HITL** | `skye_factr_teleop`、Operator UI、干预键 | USB 所在机（笔记本）；**不起 driver** |
| **推理** | VLA / dummy | Thor 同机或外置 GPU；只连口 A |

**HITL off（日常纯遥操）：** 正式拓扑下仍由开发机跑 driver；笔记本只跑小臂，直连 `/gento/*_joint_control`（跨机 ROS）。不起 arbiter。  
单人调试时可临时本机起 driver，见 §0.1 例外。

---

## 3. 标准口与现网映射

### 3.1 口 A（对外唯一宣传）

| 项 | 约定 |
|----|------|
| Topic | `/skye/policy_action`（`skye_hitl_dagger/msg/PolicyActionChunk`） |
| 语义 | 关节绝对角 rad（follower / joint_states 空间）；含 gripper；`action_dt_ns` / `request_id` / `policy_version` |
| 禁止 | 策略 pub `/gento/*_joint_control(_abs)`；禁止直连 Gento SDK |

### 3.2 口 B（内部写臂，复用现网）

| 源 | arbiter 写出 |
|----|----------------|
| Policy / AUTONOMOUS hold | `/gento/{left,right}_joint_control_abs`（经 `follower_pose_to_leader_abs`） |
| HUMAN 遥操 | `/gento/{left,right}_joint_control`（relative，透传 teleop） |

### 3.3 遥操进仲裁

| 现网（纯遥操） | HITL on |
|----------------|---------|
| `/joint_control` → `/gento/{side}_joint_control` | remap → `/skye/teleop_action_{left,right}` → arbiter |
| 夹爪 `/gripper/ctrl` → `/{side}_teleop_gripper/ctrl` | 经 `/skye/teleop_gripper_*` 或等价，由 arbiter 在 AUTONOMOUS 时写电机语义 |

### 3.4 状态机对齐（两套必须约定主权）

**小臂（`skye_factr_teleop`）：** `IDLE` ↔ sync ↔ `TELEOP`（`/mode/switch_*` + `/teleop/state`）

**仲裁（`control_arbiter`）：**

```text
                takeover
AUTONOMOUS ──────────────► HANDOVER_SYNC ──(enter_teleop + 对齐)──► HUMAN
    ▲                                                                  │
    │                         return                                   │
    ◄──────────────────────────────────────────────────────────────────┘
```

| 约定 | 说明 |
|------|------|
| AUTONOMOUS | 小臂可处 SYNCED/保持；**不要**人工再开 TELEOP 直写 gento |
| takeover | 只 sync + hold；**不**自动 `switch_teleop` |
| enter_teleop | `hold_current` → `switch_teleop` → HUMAN 后才转发 relative |
| return | 回 AUTONOMOUS；与 `/mode/switch_stop` 急停职责分离 |
| 模式话题 | HITL on 时优先由 arbiter 代发 `switch_sync` / `switch_teleop`；避免双源打架 |

---

## 4. 与已有代码的关系（不必重做）

| 组件 | 位置 | 本分支策略 |
|------|------|------------|
| `ControlArbiterNode` / `ControlArbiterLogic` | `skye_hitl_dagger` | **复用**；按需改 remap / 参数对齐同构 YAML |
| `policy_abs.follower_pose_to_leader_abs` | 同上 | **复用**；核对 signs/order/offsets 与同构 + `skye_robot.yaml` |
| `ChunkPlayer` / freshness / rebase | 同上 | **复用** |
| `TeleopHandshake` | 同上 | **复用**；对接同构 `/teleop/state` |
| `hitl_keyboard` / `intervention_cmd` | 同上 | **复用**；UI 按钮与之一致 |
| `episode_recorder` | 同上 | 可选启用；话题列表核对同构命名 |
| `skye_factr_teleop` | 新同构包 | **HITL launch remap**；纯遥操 launch 不动 |
| 旧 Marvin HITL Docker launch | `docs/Hint_Dagger启动使用说明.md` 等 | **文档改指向同构包**；代码路径废弃 |

---

## 5. 改造清单与落地节奏

推荐顺序（相对原「先下沉再改 FACTR」）：**先把同构遥操收进唯一写臂链，再固化常驻与真 VLA**，减少 remap/握手返工。

```text
Phase 0  契约：HITL on 禁止策略/小臂直写 /gento/*；SDK 互斥
Phase 1  同构遥操 → teleop_action remap + arbiter 联调（本分支主路径）
Phase 2  dummy → /skye/policy_action → abs → 真机 AUTONOMOUS
Phase 3  接管全流程：takeover → enter_teleop → HUMAN → return
Phase 4  执行栈常驻脚本（Thor/Orin）+ 会话健康检查；笔记本卸 driver
Phase 5  真 VLA 客户端 EXECUTOR=ros（同机或外置 GPU）
Phase 6  可选：ZMQ/HTTP gateway；数采加权；多机每台一套执行平面
```

### Phase 0 — 契约

- [ ] 文档写死 §0.1：正式执行平面 = Thor/Orin；笔记本正式流程不起 driver  
- [ ] 启动检查：图上已有 `skye_robot_driver` 则禁止第二路 SDK（跨机探测或本机锁）  
- [ ] 策略与训练同学只宣传口 A  
- [ ] 明确纯遥操（HITL off）与 HITL on 两套 launch，不可混开  
- [ ] 启停说明分「正式（开发机 driver）」与「单人调试（本机 driver）」两栏，避免混用

### Phase 1 — 同构遥操接入仲裁（最大工程量）

- [x] 新增 HITL launch：`start_teleop_m6_dual_hitl.launch.py`（`/joint_control`、夹爪 → `/skye/teleop_*`）；启停见 [`phase1_hitl_isomorphic_startup.md`](phase1_hitl_isomorphic_startup.md)  
- [x] arbiter / driver / 小臂统一 `robot_profile:=thor|orin`；arbiter signs 来自 `skye_hitl_dagger/config/profiles/{thor,orin}.yaml`（跟 driver，不是小臂 8 轴）  
- [ ] 确认 HUMAN 时 relative 转发与 driver reseed 行为  
- [ ] 确认 `/ee_control`、`/enable_position_sync` 等是否经 arbiter（默认：不经则文档标明「非 HITL 控制面」）  
- [x] Launch 单测：纯遥操 remap 不变；HITL dual → `/skye/teleop_*`；profile signs 与 driver 对齐测（真机图测仍待）

### Phase 2 — dummy AUTONOMOUS

- [ ] `pub_dummy_policy_chunk` + arbiter + driver 真机  
- [ ] 无新 chunk 时 hold 末步；不自动切人  
- [ ] `ros2 topic info /gento/left_joint_control_abs -v` 仅 arbiter

### Phase 3 — HITL 切换

- [ ] 键盘 / UI：`takeover` / `enter_teleop` / `return`  
- [ ] 与同构 sync 容差、min_sync_hold、hold_current 联调  
- [ ] 更新 `Hint_Dagger启动使用说明.md`、Operator UI 提示为同构路径

### Phase 4 — 执行平面常驻（把 §0.1 从「约定」变成「默认」）

- [ ] Thor/Orin：`skye_robot_driver` + `control_arbiter` 进实验/开机类脚本（同仓 `skye_ros2_ws` 在开发机编译）  
- [ ] 笔记本启动脚本：**默认不**起 driver；只起小臂 + UI；会话前检查开发机 driver/arbiter 健康  
- [ ] 验收：笔记本断电/离域，开发机上口 A 仍能动臂（推理在开发机或仍可达 Domain 时）  
- [ ] 废弃或降级「笔记本默认 `start_skye_for_factr.sh`」为调试文档，不作为 Dagger 对外步骤

### Phase 5–6 — 推理与扩展

- [ ] VLA 去掉 `runtime_set_joint_pos_cmd` / SDK 直写；`EXECUTOR=ros` 或 zmq→口 A  
- [ ] 可选 gateway、多机、训练加权

---

## 6. 验收标准

1. **HITL off：** 同构双臂遥操行为与改造前一致。  
2. **HITL on：** `*_joint_control` / `*_joint_control_abs` 上策略与遥操相关发布者仅为 `control_arbiter`。  
3. **无双 SDK：** 第二路 driver/SDK 起不来或启动脚本拒绝。  
4. **dummy：** AUTONOMOUS 真机运动；断 chunk 后 hold，不自动 HUMAN。  
5. **接管：** AUTONOMOUS → sync → enter_teleop → HUMAN → return → AUTONOMOUS，无跳变双写。  
6. **（Phase 4）** 笔记本掉线后，开发机仍可靠口 A 控臂。

---

## 7. 风险与约束

| 风险 | 缓解 |
|------|------|
| 把「本机起 driver」写成正式流程 | §0.1 + Phase 4；启停说明分栏；对外只认开发机执行平面 |
| 笔记本与 Thor 同时起 driver | 启动互斥检查；验收禁止双 SDK |
| 小臂与 arbiter 双写 `/gento/*` | HITL launch 强制 remap；checklist 查 publisher |
| `/mode/switch_*` 双源 | HITL on 约定由 arbiter 代发；UI 走 intervention |
| signs/offsets 不一致导致 abs 飞车 | 参数与同构 YAML、driver profile 同源；先小幅度 dummy |
| 旧 Hint 文档误导现场 | Phase 3 同步改文档与 Operator UI |
| Phase 1–3 调试拓扑固化成习惯 | Phase 4 强制切换正式拓扑并做掉电验收 |

---

## 8. 相关文档

| 文档 | 用途 |
|------|------|
| 本文件 | **dev-Dagger SoT**：同构接入 + 三平面 + **§0.1 部署拓扑** |
| [`../HITL_DAgger_集成方案.md`](../HITL_DAgger_集成方案.md) | 早期总方案（动机 / 需求）；实现细节以现网 + 本文件为准 |
| [`../superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md`](../superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md) | arbiter 已定稿设计 |
| [`../superpowers/plans/2026-09-20-hitl-abs-split-takeover.md`](../superpowers/plans/2026-09-20-hitl-abs-split-takeover.md) | abs + 拆分接管（代码侧多已落地） |
| [`../Hint_Dagger启动使用说明.md`](../Hint_Dagger启动使用说明.md) | 现场启停（待改为同构路径 + 正式拓扑） |
| [`../../skye_ros2_ws/src/skye_factr_teleop/dev_docs/checklist/phase7_5_dual_follower_sync_teleop_checklist.md`](../../skye_ros2_ws/src/skye_factr_teleop/dev_docs/checklist/phase7_5_dual_follower_sync_teleop_checklist.md) | 纯遥操验收基线 |

---

## 9. 下一步（立即执行）

**契约与拓扑已按 §0.1 写死。下一步仍优先 Phase 1 图正确，但文档/脚本命名上就要区分「调试本机 driver」与「正式开发机 driver」，避免 Phase 4 才第一次改习惯。**

1. **写 Phase 1 实施计划**（建议：`docs/superpowers/plans/2026-10-29-hitl-isomorphic-teleop-arbiter.md`）  
   - HITL launch 落点、remap 表、arbiter 参数来源  
   - 启动说明两栏：正式（Thor/Orin driver）/ 调试（本机 driver）  
   - `/teleop/state` 握手测试；纯遥操回归条目  
2. **盘点** HITL launch / Operator UI / `start_skye_for_factr.sh` 默认假设：哪些仍暗示「笔记本起 driver」。  
3. **参数对齐表**：左右 `joint_signs` / `order` / `offsets`（小臂 YAML ↔ driver ↔ arbiter）。  
4. **图测**：HITL on 时 gento 命令 pub 仅 arbiter（可先同机调试拓扑）。

完成 Phase 1–3 后做 Phase 4 掉电验收，把正式拓扑变成默认。
