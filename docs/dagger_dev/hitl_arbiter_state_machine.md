# HITL Arbiter 状态转移表（A/B/C 后）

> 权威持有者：**`control_arbiter`**（`ControlArbiterLogic` + handshake/retry）。  
> Leaders / driver **只响应命令**，不反向改 arbiter 模式（除 `/teleop/state`、`sync_progress` 作为观测）。

## 状态映射

| 操作员表 | 代码 `ControlModeState` | 源 |
|----------|-------------------------|----|
| IDLE | `AUTONOMOUS` | policy（abs） |
| SYNCING | `HANDOVER_SYNC` + handshake `WAIT_ALIGNED` 未 ready | hold abs |
| SYNCED | `HANDOVER_SYNC` + `aligned_ready` | hold abs；**不再**发 `switch_sync` |
| TELEOP | `HUMAN` | relative teleop 转发 |
| FAULT | `FAULT` | hold abs；等 `stop`/`return` |

## 事件 × 状态

| 事件 | IDLE | SYNCING | SYNCED | TELEOP | FAULT |
|------|------|---------|--------|--------|-------|
| **takeover** | → SYNCING；`switch_sync`；启动 retry | ignore | ignore | ignore | ignore |
| **sync_ok** (`/teleop/state` SYNCED + hold) | — | → SYNCED；停 republish | stay | — | — |
| **sync_fail** 双臂确定性失败 / retry 耗尽 | — | → FAULT；提示操作员 | — | — | stay |
| **单臂确定性失败** | — | 记录 hint；**不**拖死已 SYNCED 侧；`dual_terminal` 后停 republish | 同左 | — | — |
| **enter_teleop** | ignore | 记请求；`hold_current` **ack 后**再 `switch_teleop` | 同左 | ignore | ignore |
| **TELEOP 确认** | — | — | → TELEOP | stay | — |
| **return** | ignore | → IDLE；发 `return_mode_command` | 同左 | → IDLE | → IDLE |
| **stop** (`/mode/switch_stop` 或 `intervention_cmd=stop`) | noop（幂等） | → IDLE；`switch_stop`；取消 handover/定时逻辑 | 同左 | 同左 | 同左 |
| **timeout / backoff** | — | `5s→10s→20s` 有限次 `switch_sync`；耗尽 → FAULT | **不**重发 sync | `switch_teleop` 有限重试 | — |

## 交接握手（Fix C）

1. `enter_teleop` → 停 abs 流意图 → 调 `/gento/hold_current`  
2. **收到 Trigger ack** 后才发 `switch_teleop`（仍须 `aligned_ready`）  
3. Driver：abs 活跃时 **缓存**最新 relative 帧；会话建立时同一时刻采 `leader_ref` + `gento_ref` 并打日志时间戳  
4. Seed 后 `teleop_seed_ramp_frames` 内放宽 `max_delta`（默认 8 帧 ×3）  
5. Seed = 离合（clutch）；边界接近时的 re-anchor 可复用同一套

## 验证用例

1. SYNCED 后静置 30s：日志无新的 `cmd=switch_sync`  
2. 任意状态发 stop 后 10s：无 `switch_sync`  
3. 右臂超界：`sync_progress` 确定性失败 → 提示；左臂可保持 SYNCED；不因右臂无限重试  
4. 进 TELEOP 后手不动：大臂位移 < 阈值（seed + ramp）
