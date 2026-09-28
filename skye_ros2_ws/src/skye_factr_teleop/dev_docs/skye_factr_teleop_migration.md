# skye_factr_teleop 迁移说明

## 2026-09-28：同步超时视为失败（任务 4.1）

本节冻结新包的契约；任务 4.2 已实现纯状态机并通过 44 项测试。任务 4.3 已在
`ControlCore` 中执行停止运动、清除同步数据等动作并接入逐周期 Ruckig/hold；
任务 4.4 已通过左右臂配置的无硬件联动验收，相关测试共 120 项通过。

| 场景 | 旧计划转述的扩展语义 | 新包冻结语义 |
|---|---|---|
| 同步正常完成 | `TELEOP_SYNCING -> SYNCED` | 不变 |
| 同步超时 | `TELEOP_SYNCING -> SYNCED` | `TELEOP_SYNCING -> IDLE`，同步失败 |
| 超时后请求遥操 | 可从 `SYNCED` 进入 `TELEOP` | 从 `IDLE` 拒绝，需重新同步成功 |

来源限定：旧开发计划称原二进制扩展的公开接口文档描述了超时进入 `SYNCED`，
但没有附具体方法名或 docstring 原文。这里记录的是相对旧计划所述契约的变化，
并不声称已核验原 `.so` 的实际超时行为。

调用方迁移要求：

- 等待同步时分别处理 `SYNCED`（成功）与回到 `IDLE`（失败或停止）；
  不得将计时结束直接当作成功，也不能无限等待 `SYNCED`。
- 超时后重新请求 `switch_sync`，只有本次同步成功后才请求 `switch_teleop`；
  不自动重试，不自动进入遥操。
- `ControlCore` 在 stop、超时或控制故障时尝试零电流并关闭 torque；重新同步前
  调用方须明确重新使能。核心从不自行使能 torque。
- `TELEOP` 中请求 `switch_sync` 会被拒绝，需要先 `switch_stop`。
- 超时退出同步须停止自动走位、清除轨迹、hold 积分和旧目标，不进入成功后的 hold。
- 截止时间到达即按超时处理，正常完成只在截止时间前有效；非同步状态下的
  完成/超时事件不改变状态。
- 运行层调用 `ControlCore.handle` 发送 sync/teleop/stop 请求，不直接驱动其内部
  状态机。完成/超时由 `step(now=...)` 内部产生，拒绝从外部注入；每次同步创建
  新轨迹，控制调用须串行。显式 stop 可使用无时钟的 `core.stop()`。

本次成功条件仍为 Ruckig 正常完成；实测关节误差达标并非本次新增的成功条件。
因此 `SYNCED` 本身不构成实际关节到位的独立测量证据。

完整规则与最小测试接口见
[开发计划任务 4.1](skye_factr_teleop_development_plan.md#任务-41冻结状态转换规则)，
参数化契约测试见 `../test/test_state_machine.py`。

## 2026-09-28：任务 5.1 ROS 节点

- executable 使用 `--side left|right`，配置 YAML 默认从包 share 查找；标定偏移、
  电流上限及轨迹上限须通过只读 ROS 参数显式提供。节点启动保持 torque 关闭，
  有效 sync 或独立走位请求才使能。
- 输入须为单侧 7 轴 JointState，具有非零且新鲜的 ROS 时间戳；可选 velocity/effort
  为空或长度 7。名称使用有序 `l_j1..l_j7` / `r_j1..r_j7`；无名称固定顺序输入需
  显式设置 `allow_unnamed_joint_states=true`。
- `/leader_arm/target_joint_state` 是发布接口；阶段 0 的方向笔误已更正。
  外部走位使用 `/joint_move`，仅在 IDLE 接受，完成后保持。该路径的 `SYNCED`
  仅表示外部目标完成，不能据此进入 TELEOP；需要 stop 后重新执行 follower sync。
- follower 同步、保持和 TELEOP 期间反馈失效会停止；运动初始化或控制异常会关闭
  串口并停用定时器，需要重启节点。一般 stop 或同步超时可在有效新请求时重新使能。
