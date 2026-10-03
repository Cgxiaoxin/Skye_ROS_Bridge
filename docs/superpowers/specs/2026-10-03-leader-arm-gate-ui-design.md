# Leader Arm Gate UI（同步前关左/右小臂）

日期：2026-10-03  
状态：已通过 brainstorming 评审，待实现

## 1. 背景与目标

Operator UI 遥操数采会话启动后，大臂驱动与小臂 Docker（左右 `factr_teleop_*`）仍按现有 playbook **双臂上电**。数采有时只需单臂遥操。

本设计在「同步」按钮下方增加 **关左臂 / 关右臂** 切换：在进入同步之前可选关闭一侧（或两侧）小臂；关闭后同步/对齐只作用在仍开启的一侧。默认不点开关时，行为与今天双臂流程完全一致。

**适用**：Thor 与 Orin（同一套 UI / 脚本；差异仅 `ROBOT_PROFILE` 与已绑定的 Dynamixel 串口）。

**非目标**

- 不改会话启动 playbook（不在启动时只起单臂）
- 不改 FACTR 源码做 per-side `/mode/switch_*`
- DAgger 面板不提供关臂控件
- 不在同步/遥操中途允许开关臂

## 2. 操作员流程

1. 开始会话（双臂上电，与现网一致）
2. （可选）点「关左臂」和/或「关右臂」  
   - 需确认 toast：**请托住该侧小臂，去使能后会下落**
   - 同步前可再点同一按钮复能该侧
3. 点「同步」  
   - 至少一侧仍开启；两侧都关 → 同步灰掉不可点
4. 「开始对齐」→ 只对齐当前仍开启的一侧（或两侧）
5. 「开启遥操」→ 录制等后续与现逻辑相同

一旦 `teleop_state ∈ {TELEOP_SYNCING, SYNCED, TELEOP}`，或本会话已成功发出过 `switch_sync`，关臂按钮 **锁定灰掉**。

结束会话后 `leader_arms` 复位为双开、未锁定。

## 3. 架构（方案 1）

```
TeleopPanel 关左/右
    → POST /api/command  leader_{left|right}_{off|on}
    → 主机 scripts/leader_arm_gate.sh
         off: docker exec → pkill factr_teleop_{side} → disable DXL --side
         on:  docker exec → 后台 ros2 run 单侧 FACTR（与 dual launch remap 一致）
    → 更新进程内 leader_arms 状态 → snapshot

同步 / 对齐
    → switch_sync 仍发全局 /mode/switch_sync（关闭侧无订阅者）
    → align_start 按 enabled 选 align_follower[_left|_right]
```

会话状态机与 playbook **不改**。默认路径（不点关臂）零额外子进程。

## 4. UI

位置：遥操数采面板，「同步」正下方一行两个切换按钮。

| 控件 | 行为 |
|------|------|
| 关左臂 / 关右臂 | 切换 off/on；高亮表示「已关闭」 |
| 同步 | `left_enabled \|\| right_enabled`；否则 disabled |
| 开始对齐 | 仍要求 SYNCED；对齐侧由后端按 enabled 决定 |
| 状态行（建议） | `小臂：左开/关 · 右开/关` |

仅 `teleop_record` + 会话 `READY/RUNNING`（及门禁允许时）可操作。DAgger 面板不显示。

## 5. API / Snapshot / 门禁

### 5.1 新白名单 op（仅 teleop_record）

| op | 含义 |
|----|------|
| `leader_left_off` | 停左 FACTR + 左 Dynamixel 去使能 |
| `leader_left_on` | 容器内按现 remap 拉起左 FACTR |
| `leader_right_off` / `leader_right_on` | 右臂同理 |

前端根据 `left_enabled` / `right_enabled` 选择发 `*_off` 或 `*_on`。

### 5.2 Snapshot 增补

```json
"leader_arms": {
  "left_enabled": true,
  "right_enabled": true,
  "locked": false
}
```

现有字段保持兼容。可选配置：

```yaml
# skye_operator_ui/config/default.yaml
features:
  leader_arm_gate: true   # false 时隐藏按钮且拒绝对应 op
```

### 5.3 门禁规则

- `leader_*_off/on`：会话未就绪 / 非 teleop_record / `locked` → 拒绝
- `switch_sync`：两侧都 `enabled=false` → 拒绝「两侧小臂已关闭」；其余沿用现有 teleop_state 规则
- `align_start`：两侧都关 → 拒绝；否则允许（仍要求 SYNCED）
- 成功 `switch_sync` 或 `teleop_state` 进入同步/遥操 → `locked=true`
- 关/复能脚本失败 → API 报错，**不**翻转 enabled（UI 与真机一致）

## 6. 关臂 / 复能脚本

入口：`scripts/leader_arm_gate.sh <left|right> <off|on>`

容器名默认 `skye_marvin_m6`（与 teardown 一致，可配置）。

### 6.1 off

1. `docker exec`：`pkill -f factr_teleop_{left|right}`（先停节点，避免抢串口）
2. 同容器：`disable_leader_dynamixel.py --side {left|right}`（只写该侧 port，Torque Enable addr 64 = 0）
3. 端口来自 `marvin_ws/.skye/leader_arms.env`（`ROBOT_LEADER_DYNAMIXEL_PORT_*`）

走**已有容器** exec，不新起 one-shot 镜像（快于会话结束 teardown）。

### 6.2 on（仅同步前）

同容器后台 `ros2 run factr_teleop factr_teleop_robot_driver.py`，参数/remap 与  
`marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py` 中对应侧一致；  
config 使用 `configs/$ROBOT_PROFILE/grav_comp_m6_{side}.yaml`。

FACTR 启动时自行重新使能 Dynamixel。

### 6.3 `disable_leader_dynamixel.py`

增加 `--side left|right|both`（默认 `both`，保持现有 teardown 行为）。

## 7. 单侧对齐

`/mode/align_follower` 的 `data`：

| data | 行为 |
|------|------|
| `align_follower` | 双臂（现有，兼容键盘 `s`） |
| `align_follower_left` | 仅左 |
| `align_follower_right` | 仅右 |

UI `align_start`：双开 → `align_follower`；只开左/右 → 对应单侧；双关 → 拒绝。

`follower_align` 节点：

- 只 `start()` 活跃侧 `AlignSession`；非活跃侧不参与 `combine_phase`
- fresh / abs 指令 / 误差：仅活跃侧
- 非活跃侧不发 `*_joint_control_abs`（driver 超时 hold）
- `set_motion_rates` 可仍设双侧低速（实现简单）
- `/align/status` 语义不变：`IDLE → ALIGNING → ALIGNED | TIMEOUT_WARN`

## 8. 效率

- 默认双臂路径：不调用 gate 脚本，与现网同等延迟
- 关/复能：单次 `docker exec`（秒内），不拉新容器
- 门禁用进程内布尔，不轮询 `ros2 node list`
- 同步仍一次 `ros2 topic pub`（全局 switch_*）

## 9. 复原双臂默认体验

| 层级 | 做法 |
|------|------|
| 操作 | 不点关臂按钮 → 与今天完全一致 |
| 配置 | `features.leader_arm_gate: false` 隐藏并禁用 op |
| 代码 | 变更集中在 TeleopPanel、commands、snapshot、gate 脚本、`disable --side`、align payload；可整提交 revert |

## 10. 测试计划

**单测**

- `command_allowed`：双关拒 sync；locked 拒 leader_*；单开允许 sync
- snapshot：`leader_arms`；`align_start` dispatch payload 随 enabled
- align：`align_follower_right` 不要求左 leader fresh
- `disable_leader_dynamixel --side` 解析（不强制真机串口）

**真机冒烟（Thor + Orin 各一次）**

- 不点关臂：完整双臂路径
- 关左 → sync → 单侧对齐 → teleop → 短录
- 同步后确认关臂按钮灰掉
- 两侧都关：同步不可点

## 11. 文档

- 更新 `docs/Operator_UI使用说明.md`（按钮、托住小臂、同步后锁定）
- `docs/Thor_Orin_遥操启动.md` 可加一句：UI 支持同步前关单侧小臂

## 12. 实现触及文件（预期）

- `skye_operator_ui/web/.../TeleopPanel.svelte`、`commands.js`
- `skye_operator_ui/.../commands.py`、`ros_bridge.py`、`snapshot.py`、`api_app.py`（若需）
- `skye_operator_ui/config/default.yaml`
- `scripts/leader_arm_gate.sh`（新）
- `scripts/disable_leader_dynamixel.py`（`--side`）
- `skye_follower_align/.../follower_align_node.py`、`align_logic.py` + 测试
- 相关 `test_*.py` / 前端门禁测试
- 上述 docs
