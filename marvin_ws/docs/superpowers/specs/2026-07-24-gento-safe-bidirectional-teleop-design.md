# Gento 双向遥操安全监督器设计

> **Status (2026-07-24):** Deferred. Active Gento teleop path is bridge-less factr wiring:
> [2026-07-24-gento-factr-teleop-bridge-less-design.md](./2026-07-24-gento-factr-teleop-bridge-less-design.md)

## 目标

将当前 Gento 双大臂与同构小臂链路改造成一个可审计的双向遥操系统。系统必须复用旧链路的操作语义：键盘 `1` 请求“大臂到小臂”的平滑对齐，只有对齐完成后键盘 `2` 才能请求“小臂到大臂”的遥操，键盘 `3` 停止遥操输出。

本设计优先保证人身与设备安全。没有通过状态、单位、映射和人工确认检查时，系统不得对大臂或小臂发出新的位置目标。

## 范围

本次实现包含：

- 一个独立的 Gento 遥操安全监督器；
- 一个只向监督器发送 `1`、`2`、`3` 模式请求的键盘节点；
- Gento 驱动的受控保持、受控停止和命令超时接口；
- 启动脚本、配置和 SOP 的更新；
- 无硬件的状态机、单位、映射、限幅、重离合和超时测试。

不包含：

- 自动标定关节零位、方向或尺度；
- 绕过物理急停、控制器故障处理或硬件限位；
- 修改没有源码的 `factr_teleop` 二进制实现。

## 旧链路中保留的安全语义

旧链路使用 `IDLE -> TELEOP_SYNCING -> SYNCED -> TELEOP`。本设计保留其核心原则，但把状态、验证和大臂输出的决定权移到新监督器中：

1. `1` 不会控制大臂；它只能请求小臂向大臂当前姿态平滑对齐。
2. 只有两个小臂均持续满足对齐误差条件，才会进入 `SYNCED`。
3. `2` 不能直接进入 `TELEOP`；它必须从 `SYNCED` 发起，并额外经过人工确认。
4. 遥操采用相对映射（离合器），而不是把小臂绝对角度直接当成大臂绝对角度。
5. `3` 是受控停止/保持请求，不是急停；物理急停仍是唯一的紧急人身保护手段。

## 进程与话题边界

四类数据必须分离，禁止把小臂状态与 Gento 状态复用到同一话题。

| 类别 | 话题 | 发布者 | 订阅者 | 单位/长度 |
|---|---|---|---|---|
| Gento 大臂反馈 | `/gento/joint_states` | `gento_robot_driver` | 安全监督器 | rad，14，顺序固定为 `l_j1..l_j7,r_j1..r_j7` |
| 左小臂真实反馈 | `/gento_teleop/left_small_arm_state` | `factr_teleop_left` | 安全监督器 | rad，7 个臂关节；夹爪不参与映射 |
| 右小臂真实反馈 | `/gento_teleop/right_small_arm_state` | `factr_teleop_right` | 安全监督器 | rad，7 个臂关节；夹爪不参与映射 |
| 小臂对齐目标 | `/left_joint_move`、`/right_joint_move` | 安全监督器 | 对应 `factr_teleop` 节点 | rad，7 |
| 左大臂遥操目标 | `/gento/left_joint_control` | 安全监督器 | `gento_robot_driver` | rad，7 |
| 右大臂遥操目标 | `/gento/right_joint_control` | 安全监督器 | `gento_robot_driver` | rad，7 |
| 模式请求 | `/gento_teleop/mode_request` | 新键盘节点 | 安全监督器 | `std_msgs/msg/String`，仅 `sync`、`teleop`、`stop` |
| 状态公布 | `/gento_teleop/state` | 安全监督器 | 操作员/监控 | 状态名与拒绝原因 |

`start_gento_dual_arm_sync.sh` 不得再把 `factr_teleop` 的 `/joint_state` 重映射为 `/gento/joint_states`。它必须将左右小臂各自的状态发布重映射至上述 `gento_teleop/*_small_arm_state` 话题。

`factr_teleop` 的 `/enable_position_sync` 和 `/mode/switch_sync`、`/mode/switch_teleop`、`/mode/switch_stop` 订阅必须重映射到每个节点独有的禁用命名空间。启动后仅发布一次全局 `/enable_position_sync=false` 用于可观测性；安全监督器永远不发布 `true`。因此内置 `factr` 同步状态机与 Gento 监督器互斥，只有监督器能发小臂走位目标和大臂遥操目标。

## 小臂反馈的黑盒验收

在任何 `sync` 或 `teleop` 请求前，监督器都处于 `IDLE`，并持续验证这两个小臂反馈话题。验收规则：

- 每侧必须有唯一预期的 `factr_teleop_*` 发布者；
- 每条消息必须包含恰好 7 个有限臂关节 position；
- `name` 非空时，名称必须与配置的 7 个名称完全一致；
- 相邻反馈的时间间隔不超过 `0.10 s`；
- 任一关节不能超出配置的小臂 rad 软限位及其 `0.10 rad` 安全裕量；
- 监督器将首条和最近一条反馈写入运行诊断日志，包含来源 topic、长度、名称和 rad 数值。

验收失败时必须保持 `IDLE`，并在 `/gento_teleop/state` 发布明确原因；禁止以 `/gento/joint_states` 或任何大臂话题替代小臂反馈。

## 配置与标定

新增 `gento_safe_teleop.yaml`，包含左右独立的：

- Gento 反馈索引：左 `[0..6]`，右 `[7..13]`；
- 小臂状态 topic、走位目标 topic 与 Gento 命令 topic；
- 小臂和 Gento 的 rad 关节限位；
- `joint_order` 与 `joint_sign` 映射；
- 同步速度、加速度、每周期步长、对齐误差和持续时间；
- 遥操每周期步长、最大速度、跟踪误差和反馈超时；
- `calibrated` 布尔值。

配置初始值 `calibrated: false`。只要任一侧未标定，监督器可做状态诊断但必须拒绝 `sync` 和 `teleop`。

`joint_sign` 不得从 `factr_teleop` 的 Dynamixel `joint_signs` 自动推断；后者是物理伺服方向配置，不是经过验证的小臂模型关节到 Gento 关节的语义映射。两侧必须逐关节低速验证后，才由操作员将配置标为已标定。

默认初始安全参数如下，所有值均可在 YAML 中进一步收紧：

```yaml
feedback_timeout_s: 0.10
sync_confirm_timeout_s: 15.0
sync_max_velocity_rad_s: 0.10
sync_max_acceleration_rad_s2: 0.20
sync_max_step_rad: 0.001
sync_max_initial_error_rad: 0.35
sync_complete_error_rad: 0.03
sync_complete_hold_s: 0.50
teleop_max_velocity_rad_s: 0.10
teleop_max_step_rad: 0.001
teleop_tracking_error_rad: 0.10
teleop_tracking_error_hold_s: 0.20
command_timeout_s: 0.20
calibrated: false
```

`sync_max_step_rad` 和 `teleop_max_step_rad` 是监督器每次发布时允许的最大目标变化，不是只依赖底层控制器的插补参数。

## 状态机和确认流程

监督器维护一个全局双臂状态。任意一侧异常均使双臂一起退出活动状态，避免一侧在动而另一侧失去同步。

| 当前状态 | 请求 | 条件 | 结果 |
|---|---|---|---|
| `IDLE` | `sync`（键盘 `1`） | 双侧反馈与 Gento 反馈有效，映射已标定 | `SYNC_CONFIRM_REQUIRED` |
| `SYNC_CONFIRM_REQUIRED` | `/gento_teleop/confirm_sync` Trigger 成功 | 15 s 内，反馈仍有效 | `TELEOP_SYNCING` |
| `TELEOP_SYNCING` | `teleop`（键盘 `2`） | 任意条件 | 拒绝，不改变状态 |
| `TELEOP_SYNCING` | 两侧误差均小于 `0.03 rad` 且持续 `0.50 s` | 所有反馈新鲜 | `SYNCED` |
| `TELEOP_SYNCING` | `stop`（键盘 `3`） | 无 | 受控保持后 `IDLE` |
| `SYNCED` | `teleop`（键盘 `2`） | 所有反馈有效、映射已标定 | `TELEOP_CONFIRM_REQUIRED` |
| `TELEOP_CONFIRM_REQUIRED` | `/gento_teleop/confirm_teleop` Trigger 成功 | 15 s 内，反馈仍有效 | 捕获参考点后进入 `TELEOP` |
| `TELEOP` | `sync`（键盘 `1`） | 无 | 受控保持，返回 `SYNC_CONFIRM_REQUIRED`，不得直接移动小臂 |
| `TELEOP` | `stop`（键盘 `3`） | 无 | 受控保持后 `IDLE` |
| 任意活动状态 | 任一反馈过期、消息非法、超时、映射越界、单臂失步 | 无 | 两臂进入故障停止，随后 `IDLE` |

键盘节点只能发布 `/gento_teleop/mode_request`；它不能向 `/mode/switch_*` 或 `/enable_position_sync` 发布。确认服务由操作员在单独终端调用，因此 `1` 和 `2` 都不能因为一次误触而开始移动。

## 同步算法（大臂到小臂）

在 `TELEOP_SYNCING`，每个控制周期：

1. 读取并验证 Gento 14 轴状态，左侧取 `0..6`，右侧取 `7..13`。
2. 对每一侧应用显式 `joint_order` 和 `joint_sign`，得到小臂语义目标 `q_small_goal`。
3. 读取已验收的小臂真实反馈 `q_small_now`。
4. 从上一帧小臂目标向 `q_small_goal` 生成受速度、加速度和 `sync_max_step_rad` 约束的下一帧目标。
5. 目标超过小臂 rad 软限位或起始误差超过配置的 `sync_max_initial_error_rad` 时，拒绝同步并回 `IDLE`；不裁剪成另一个未知姿态。
6. 只向 `/left_joint_move` 和 `/right_joint_move` 发布该受限目标；不向任何 Gento command topic 发布。

同步期间任一小臂未按目标收敛、两侧之一失去反馈或超过 `sync_confirm_timeout_s`，两个小臂均停止接收新走位目标，并进入故障停止。

## 遥操算法（小臂到大臂）

在从 `TELEOP_CONFIRM_REQUIRED` 进入 `TELEOP` 的瞬间，监督器捕获每侧参考点：

```text
q_small_ref = q_small_now
q_gento_ref = q_gento_now
```

随后每周期根据已标定的映射计算：

```text
q_gento_raw = q_gento_ref + joint_sign * reorder(q_small_now - q_small_ref)
q_gento_cmd = rate_limit_and_validate(q_gento_raw)
```

`rate_limit_and_validate()` 先拒绝 NaN、非 7 轴、过期或越过 Gento rad 关节限位的值，再施加速度和 `teleop_max_step_rad` 限制。关节位置会触及限位时必须进入故障停止，不能把超出限位的命令静默夹到限位后继续遥操。

如果速度或单周期步长限幅确实发生，监督器必须立即重离合，使用当前 Gento 实测位置和当前小臂实测位置更新 `q_gento_ref`、`q_small_ref`。这防止解除限幅后积累的相对位移一次性释放为大跳变。关节位置限位饱和不允许重离合后继续遥操，而是按故障停止处理。

如果 Gento 实测位置与最近的受限命令差超过 `teleop_tracking_error_rad` 并持续 `teleop_tracking_error_hold_s`，监督器必须对两个大臂执行故障停止并回 `IDLE`。该检查在 `SYNCED` 和 `TELEOP` 持续执行，不能只在进入状态时检查一次。

## 停止、保持和急停语义

“停止发布”不是停止机械臂。本设计使用三个不同语义：

- **受控保持**：键盘 `3`，或从 `TELEOP` 请求重新同步时触发。监督器调用 Gento 驱动的 `hold_current` 服务，驱动读取两臂当前反馈并将当前位置设为保持目标；之后监督器停止周期性遥操命令并进入 `IDLE`。这是正常操作停止，不是急停。
- **故障停止**：状态过期、非法消息、超时、失步或内部异常时触发。监督器调用 Gento 驱动的 `stop_motion` 服务；驱动执行 `FX_L1_Runtime_StopTraj()`，切换两臂到 SDK idle，并拒绝后续命令直到新的受控启动。小臂侧停止发布新的 `/left_joint_move`、`/right_joint_move` 目标。
- **物理急停**：人身危险、碰撞、异常高速度或控制器不可预期动作时，由现场急停/断电执行。软件 `stop_motion` 不能替代物理急停。

Gento 驱动还必须实现本地 `command_timeout_s` 看门狗：在 `TELEOP` 期间没有新命令超过 `0.20 s` 时，驱动自行执行故障停止，而不能仅继续保持上一次遥操目标。

## 启动顺序

1. 启动小臂节点，但禁用其内置 sync/teleop 模式订阅，且不订阅 `/gento/joint_states`。
2. 运行小臂反馈黑盒验收；失败则不启动监督器活动状态。
3. 启动 Gento 驱动和安全监督器；监督器处于 `IDLE`，只诊断，不发布运动目标。
4. 启动新的键盘节点；确保旧 `keyboard_gripper` 不运行。
5. 操作者按 `1`，审阅状态与区域后调用 `confirm_sync`；等待 `SYNCED`。
6. 操作者按 `2`，再次审阅状态与区域后调用 `confirm_teleop`；才允许低速相对遥操。

## 验收与测试

无需硬件的测试必须覆盖：

- Gento 14 轴名称、长度、顺序、单位范围和超时拒绝；
- 小臂反馈 7 轴黑盒验收，且验证不会接受 Gento 话题作为小臂状态来源；
- `IDLE`、确认等待、`TELEOP_SYNCING`、`SYNCED`、`TELEOP` 和所有拒绝转换；
- `SYNCING` 中按 `2` 必须拒绝；`TELEOP` 中按 `1` 必须先受控保持；单臂异常导致双臂退出；
- 同步目标速度、加速度、步长和限位拒绝；
- 相对映射、关节重排/符号、未标定拒绝；
- 大臂命令限幅后的重离合；持续跟踪误差与命令超时；
- 受控保持、故障停止、驱动看门狗的不同服务语义；
- 启动脚本不再发布 `/enable_position_sync=true`，不再将 `/joint_state` 映射至 `/gento/joint_states`，并禁用 `factr` 原模式 topic。

首轮硬件测试只允许在无负载、清场、物理急停可用、两侧关节映射已逐轴验证、速度参数保持默认低值时进行。先验证 `1` 的单侧小幅同步，再验证 `2` 的单关节小幅相对移动，最后才验证双臂。
