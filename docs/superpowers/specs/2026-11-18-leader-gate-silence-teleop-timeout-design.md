# Leader Gate 静默 + TELEOP follower 超时放宽

日期：2026-11-18  
状态：已通过 brainstorming 评审，待实现  
范围：`skye_factr_teleop` 驱动软门控与 follower 新鲜度；Operator UI 文档同步说明  
相关：`docs/superpowers/specs/2026-10-03-leader-arm-gate-ui-design.md`（关臂产品流程）；本 spec 补齐软门控数据面隔离与 TELEOP 看门狗

## 1. 背景与目标

现场对照：

- **双臂** sync → teleop：稳定
- **关左或关右（软门控）** → sync → teleop：开侧约数秒后出现 `Follower state expired; stopping`，随后大臂 `command timeout` hold

根因分层：

1. **软门控不彻底**：关侧进程仍跑控制 tick，持续发 `/{side}_leader_arm/current_state`，并作为共用 `/teleop/state` 的 `IDLE` 发布者残留在图上；该侧故意不 sync，表现为「数据还在发、一直未对齐」。
2. **TELEOP 仍用偏紧的 follower 连续新鲜度看门狗**（Orin `state_timeout=0.50`）：互斥回调下偶发处理间隔即可误杀开侧遥操（日志直接死因）。

目标（方案 A + A1）：

- 关侧 **静默**：停控制 tick / 停发 leader 状态；**不再污染** `/teleop/state`
- 开侧 TELEOP：**follower 超时放宽到 2.0s**；SYNC/HOLD 仍用现有 `state_timeout`
- 过期日志带上 age，便于区分误杀与真断流
- 单测覆盖「关一侧 → sync → TELEOP 持续」相关行为

非目标（本轮不做）：

- 拆 `/teleop/state` 为左右话题或加聚合器（方案 B）
- TELEOP 阶段完全取消 follower 看门狗（A2）
- 关臂改回杀进程（方案 C）
- 改 `skye_robot_driver` callback group / DDS 架构
- DAgger 专用路径改动（软门控语义在 FACTR 驱动层统一即可）

## 2. 操作员可见行为

流程不变：开始会话 →（可选）关左/右 → 同步 → 开启遥操。

关臂后额外保证：

| 项目 | 关侧 | 开侧 |
|---|---|---|
| Dynamixel 力矩 | 关闭 | 正常 |
| 控制 tick / `current_state` | **停止** | 正常 |
| `/teleop/state` | **不再发布**（含不再刷 IDLE） | 正常状态机发布 |
| sync / teleop 全局指令 | 忽略 | 正常处理 |
| TELEOP 中 follower 过期阈值 | n/a | **2.0s** |
| SYNC/HOLD 中 follower 过期阈值 | n/a | 仍为 `state_timeout`（Orin 0.50s） |

复能（gate ON，仅同步前允许）：恢复 tick，并 publish 当前真实模式到 `/teleop/state`。

## 3. 架构

```
/mode/<side>/leader_gate  off
  → SkyeFactrTeleopNode._on_leader_gate
      stop + torque off
      _leader_gated_off = True
      _tick early-return（无硬件读、无 current_state / temp / diagnostics 发布）
      不向 /teleop/state publish

/mode/<side>/leader_gate  on
  → _leader_gated_off = False
  → 恢复 _tick
  → _publish_state() 一次（当前模式）

TELEOP 周期 _fresh_follower
  → max_age = teleop_state_timeout（默认 2.0）
SYNC / HOLD / 进入 teleop 前检查
  → max_age = state_timeout（现有）
```

本轮仍保留左右共用 `/teleop/state`；靠「关侧不发布」消除 IDLE/TELEOP 双发布者污染，不引入新话题。

## 4. 配置

扩展 `RuntimeConfig`：

| 字段 | 含义 | 默认 |
|---|---|---|
| `state_timeout` | SYNC/HOLD（及非 TELEOP 会话检查）最大 age | 现有（代码默认 0.25；Orin yaml 0.50） |
| `teleop_state_timeout` | **仅** `ControlMode.TELEOP` 下 `_fresh_follower` / 入 TELEOP 前检查的 max_age | **2.0** |
| `future_tolerance` | 不变 | 现有 0.05 |

校验：`teleop_state_timeout` 必须为正有限值；允许 `>= state_timeout`（本轮产品值为 2.0）。

Orin 左右 yaml 显式写入：

```yaml
runtime:
  state_timeout: 0.50
  teleop_state_timeout: 2.0
  future_tolerance: 0.05
```

Thor 打包 yaml 若无 `runtime` 段：依赖代码默认 `teleop_state_timeout=2.0`；若后续补 `runtime`，同样写上 `teleop_state_timeout: 2.0`。

`_on_follower` 入队校验仍用 **较严** 的阈值还是分模式？

- **入队 `validate_joint_state`**：继续用 `state_timeout`（0.50），避免缓存明显过期 stamp 的样本。
- **会话看门狗 `_fresh_follower`**：按当前模式在 `state_timeout` vs `teleop_state_timeout` 之间选择 limit。

这样 TELEOP 容忍「接收间隔偶发变长」，但不会接受 stamp 已烂掉的新包（新包仍按 0.50 拒绝并清空，行为与今日拒绝路径一致；若拒绝时 session active 仍会 stop——见下节边界）。

## 5. 驱动行为细节

### 5.1 Gate OFF

1. `_follower_session_active = False`；`core.stop()`（力矩关）
2. `_leader_gated_off = True`
3. **不调用**会向 `/teleop/state` 发报的 `_publish_state()`（或 `_publish_state` 在 gated 时 no-op）
4. 打日志：`[side] leader gate OFF：忽略 sync/teleop，力矩已关闭，控制发布已静默`
5. 之后 `_tick`：若 `_leader_gated_off`，立即 return（在 follower 看门狗与 `core.step` 之前）

### 5.2 Gate ON

1. `_leader_gated_off = False`
2. 日志：可再次接收 sync/teleop
3. `_publish_state()` 发布当前模式（通常 IDLE）

### 5.3 TELEOP 看门狗

`_fresh_follower(limit=...)` 或内部按 `core.mode` 选 limit：

- `mode == TELEOP` → `teleop_state_timeout`（2.0）
- 其它 follower 会话模式 → `state_timeout`

过期时日志示例：

```text
Follower state expired; stopping (age=0.812s stamp_age=0.805s limit=2.000s mode=TELEOP)
```

### 5.4 `_on_follower` 拒绝包

保持现状：校验失败 → `_follower = None`；若 `_follower_session_active` 则 `_on_stop`。  
GATE OFF 时 `_follower_session_active` 已为 False，关侧不会因 follower 拒包停「会话」（本无无会话）。  
开侧 TELEOP 下若连续拒包导致 `_follower is None`，下一 tick 仍会因不 fresh 而停——**2.0s 只放宽 age 窗口，不放宽「无样本」**。

## 6. Operator UI / 文档

- **代码**：本轮不强制改 UI；关侧不再发 `/teleop/state` 后，mailbox 自然以开侧为准（双臂时仍是最后一条，与今日双臂行为一致）。
- **文档**：更新 `docs/Operator_UI使用说明.md`「同步前关单侧小臂」：
  - 关侧静默（无 leader 状态流、不占用 `/teleop/state`）
  - TELEOP follower 超时 2.0s；SYNC 仍 0.50s（Orin）

## 7. 测试计划

`skye_factr_teleop` 单测（`test_ros_driver.py` 或邻近）：

1. **Gate silence**：sync 前 gate OFF → 推进若干 tick → `current_state` 发布计数不增加；`/teleop/state` 无新消息；gate ON 后恢复发布。
2. **Gate 仍忽略 sync/teleop**（保留/加固现有用例）。
3. **TELEOP 超时放宽**：进入 TELEOP 后，follower age 置 1.0s（>0.5 且 <2.0）→ 仍为 TELEOP；age 置 2.1s → IDLE + 停止发令。
4. **SYNC 仍严格**：SYNCING/HOLD 下 age 0.6s 且 `state_timeout=0.5` → 仍停止（回归）。
5. **config**：解析 `teleop_state_timeout`；非法非正数拒绝。

真机冒烟（文档勾选延续）：

- Orin：关左 → sync → teleop ≥30s 不断开；关右同理
- 双臂路径无回归

## 8. 风险与回滚

| 风险 | 缓解 |
|---|---|
| TELEOP 下大臂 ROS 反馈真断流时停得更慢（最多 ~2s） | driver `command_timeout_s` 与 SDK 仍在；仅放宽小臂侧 ROS 看门狗 |
| 关侧静默后诊断/温度话题无数据 | 接受；关侧本就不参与遥操 |
| 入队仍按 0.50 拒包导致 TELEOP 仍易停 | 若复测仍误杀，再单开 follow-up：TELEOP 入队也用 2.0 或只更新 received_at；**本轮不扩大范围** |

回滚：恢复 gate 后 `_publish_state`、去掉 tick early-return、去掉 `teleop_state_timeout` 或改回与 `state_timeout` 相同。

## 9. 实现落点（预览）

| 区域 | 文件 |
|---|---|
| 配置 | `skye_factr_teleop/config.py`；`config/orin/grav_comp_m6_{left,right}.yaml` |
| 驱动 | `skye_factr_teleop/ros_driver.py`（gate / `_tick` / `_fresh_follower` / 日志） |
| 测试 | `skye_factr_teleop/test/test_ros_driver.py`；必要时 `test_config.py` |
| 文档 | `docs/Operator_UI使用说明.md` |

详细步骤见实现计划（writing-plans 产出）。
