# Operator UI 同步校正面板（Sync Coach）设计

日期：2026-11-03  
状态：已定稿（待实现计划）  
范围：`skye_operator_ui` 后端订阅 + 前端展示；不改 `skye_factr_teleop` 驱动发布契约；不起 `dual_trial` 进程

## 1. 背景与目标

同步阶段若主从误差未进阈值就切遥操，容易出现**单臂进入 TELEOP**。现场习惯在终端跑：

```bash
ros2 run skye_factr_teleop skye_factr_dual_trial \
  --phase sync --left-config "$LEFT_CONFIG" --right-config "$RIGHT_CONFIG" \
  --operator … --duration 180
```

用它查看左右臂超限关节与误差。Operator UI 同步时缺少同等反馈，遥操员难以手动矫正。

目标：在 UI **同步相位**展示与 dual_trial / driver `sync_progress` 同语义的左右臂超限信息；**SYNCED（或离开同步相位）后自动隐藏**；不额外常驻进程。

非目标：

- 启动或旁路 `skye_factr_dual_trial`
- 修改 teleop 驱动改为结构化 JSON（可后续演进）
- 误差条可视化、改同步阈值 / settle 参数
- 会话外常驻校正面板

## 2. 产品决策（已锁定）

| 决策 | 选择 |
|---|---|
| 数据源 | 现有 `/leader_arm/{left,right}/sync_progress`（driver ~1 Hz） |
| 布局 | 主操作栏「同步」相关控件下方，左右分栏 |
| 信息密度 | 结构化；**只列出超限轴**；达标侧短提示 |
| 模式范围 | **遥操数采** + **DAgger** 接管同步 |
| 实现路径 | Operator UI 订阅并解析文案（方案 1） |

## 3. 数据流

```
skye_factr_teleop_driver (_publish_sync_progress / _report_sync)
  → std_msgs/String JSON: { side, text, outcome, stamp_s }
  → topics: /leader_arm/left/sync_progress, /leader_arm/right/sync_progress
  → ros_bridge 订阅、解析、按侧缓存
  → snapshot.sync_coach
  → WebSocket /ws/state
  → SyncCoach.svelte
```

驱动已在 `TELEOP_SYNCING` / `HOLD_SETTLING`（及超时失败）发布；SYNCED/IDLE 后不再刷 active progress。UI **不**为校正另起节点。

## 4. Snapshot 契约

`snapshot.sync_coach`（会话无同步活动时可为 `null` 或 `{ visible: false }`）：

```json
{
  "visible": true,
  "left": {
    "side": "left",
    "mode": "HOLD_SETTLING",
    "within": 6,
    "total": 8,
    "blockers": [
      { "axis": "J1", "kind": "pos", "value": 0.216, "limit": 0.16 },
      { "axis": "J2", "kind": "vel", "value": 0.192, "limit": 0.10 }
    ],
    "settling": { "current_s": 0.0, "need_s": 0.25 },
    "outcome": "active",
    "stamp_s": 1793696500.1,
    "raw_text": "…",
    "parse_ok": true,
    "stale": false,
    "disabled": false
  },
  "right": { }
}
```

侧字段约定：

- `disabled: true`：该侧小臂已关（leader gate）；不期待 progress。
- `stale: true`：超过新鲜阈值（**2.0 s**）无新消息。
- `parse_ok: false`：保留 `raw_text`，前端 mono 兜底。
- `blockers`：仅超限轴；空数组表示该样本瞬时达标（可能仍在 continuous settling）。
- `kind`：`pos` ← `|position error|`；`vel` ← `|velocity|`；夹爪轴名 `gripper(ID8)` 或规范化为 `gripper`。

## 5. 显隐规则

| 模式 | `visible=true` 当 | 隐藏当 |
|---|---|---|
| `teleop_record` | `teleop_state == TELEOP_SYNCING`，或任一侧有新鲜 `outcome=active` progress；或任一侧新鲜 `outcome=failed` | `teleop_state ∈ {SYNCED, TELEOP, IDLE, null}` 且无新鲜 active/failed progress；或会话结束 |
| `dagger` | `hitl.mode == HANDOVER_SYNC` 且（有新鲜 progress 或仍在等待首包） | 离开 `HANDOVER_SYNC`；或会话结束 |

补充：

- 一侧已收敛、另一侧仍 settling：面板保持，直到同步相位整体结束。
- 两侧 progress 均 stale，但 `teleop_state` 仍为 `TELEOP_SYNCING`：面板仍显示，两侧标「反馈超时」。
- `clear_session_cache` / 结束会话：清空两侧缓存与 `sync_coach`。
- 关臂侧：`disabled`，文案「已关闭」。

## 6. 解析

纯函数（建议模块名 `sync_progress_parse.py`）：输入 `text`（及可选 JSON 信封字段）→ 侧结构。

从现有 driver 行识别：

- `N/8 轴当前达标` → `within` / `total`
- `TELEOP_SYNCING` / `HOLD_SETTLING`（及行首 mode）→ `mode`
- `J#: |position error|=X rad > Y rad` → blocker `pos`
- `J#|gripper…: |velocity|=X rad/s > Y` → blocker `vel`
- `Continuous settling: A / B s` → `settling`
- 失败前缀（同步失败 / timeout）→ 与 `outcome=failed` 对齐展示

解析失败不抛到 WS 循环：`parse_ok=false`，前端显示 `raw_text`。

## 7. 前端

- 新组件 `web/src/components/SyncCoach.svelte`（只读）。
- **TeleopPanel**：插在关臂块之后、「开启遥操」等后续按钮之前（同步按钮所在 `action-grid` 流程中、校正信息紧贴同步操作）。
- **DaggerPanel**：三按钮（同步 / 进入遥操 / 交还）之后、HITL 录制区块之前。
- UI：标题「同步校正」+ 状态胶囊；左右列；有 blockers 列琥珀边，无 blockers 绿提示「瞬时达标，保持中…」；底栏固定 hint「手动微调超限关节，左右均 SYNCED 后再开遥操」（DAgger 可用「再进入遥操」）。
- 样式：复用 Operator UI CSS 变量（`--accent` / `--wait` / `--font-mono` 等）。

## 8. 后端改动点

- `ros_bridge.py`：订阅左右 `sync_progress`；缓存；`mailbox()` / snapshot 组装处输出 `sync_coach`；会话清理。
- Snapshot 组装（`api_app` 或既有 snapshot builder）：合并 `teleop_state` / `hitl` / `leader_arms` 计算 `visible` 与 `disabled`。
- 单测：解析函数 + mailbox/显隐。
- 文档：`docs/Operator_UI使用说明.md` 增加同步校正说明（可不再依赖 dual_trial 盯误差；CLI 仍可用于离线验收）。

## 9. 测试计划

1. 单元：样例文案（左右、仅速度超限、仅位置、8/8 settling、失败）解析快照。
2. Bridge：注入 String 消息 → `sync_coach` 字段与 stale/visible。
3. 手工：Orin/Thor UI 点同步 → 面板出现超限轴 → 到位后 SYNCED 面板消失；DAgger takeover 同步同检；关一侧再同步。

## 10. 风险与后续

- **文案格式漂移**：解析绑 driver 中文/英文行；单测钉样例；若频繁破再演进方案 2（驱动发结构化字段）。
- **QoS**：与现有 teleop String 订阅策略对齐（KEEP_LAST depth 1；progress 为 volatile 时按 driver 实际 QoS）。

## 11. 参考

- Driver：`skye_factr_teleop/ros_driver.py` `_publish_sync_progress` / `_report_sync`
- 条件文案：`sync_trial.py` `_sync_conditions`
- 现场终端样例：`dual_trial --phase sync` / Operator 同步日志
- 布局选择：`.superpowers/brainstorm/` mockup（A 插位 + 密度 2）
