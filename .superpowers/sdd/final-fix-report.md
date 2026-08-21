# Final whole-branch review fixes (feat/hitl-dagger-arbiter)

## C1 (Critical) 接管死锁

原实现要求 `/teleop/state` 先离开 `TELEOP` 再回到 `TELEOP` 才发 `switch_teleop`，
而 FACTR 只有收到 `switch_teleop` 才会进入 `TELEOP` → 互等死锁。

新握手（`skye_hitl_dagger/teleop_sync.py`，`TeleopHandshake`）两阶段：

1. `takeover` → 发 `/mode/switch_sync`，进入 `WAIT_ALIGNED`
2. `/teleop/state` 出现 `SYNCED`（`TELEOP_SYNCING` 不算对齐）→ 发
   `/mode/switch_teleop`，进入 `WAIT_TELEOP`，同时清掉缓存 state
3. 收到**新的** `TELEOP` → `sync_completed()` → `HUMAN`

- 阶段 1 不清缓存 state：接管前锁存的 `SYNCED` 就是对齐状态，可直接放行，避免
  FACTR 已 SYNCED 却不再发布状态导致的二次死锁。
- 阶段 2 清缓存：锁存的旧 `TELEOP` 不能冒充本次握手结果。
- `sync_timeout_s` 超时不再只打印警告，会按当前阶段重发 `switch_sync` /
  `switch_teleop`（丢包自愈）。
- `sync_ready()` 已删除，逻辑移入 `TeleopHandshake`。

## I1 交还不再发 switch_teleop

`w`（return）改为发布 `return_mode_command`（默认 `switch_sync`，可配
`switch_stop`），让 FACTR 退出 `TELEOP` 回到跟随大臂，而不是把它按在 `TELEOP`。
`/mode/{switch_sync,switch_teleop,switch_stop}` 三个 publisher 统一由
`_publish_mode_command()` 路由。

## I2 陈旧 chunk 丢弃

- `ChunkPlayer.clear()` / `has_chunk()`；接管与交还都清掉当前 chunk。
- 交还时记录 `_return_time`，`chunk_is_fresh()` 要求新 chunk 的 header stamp
  >= 交还时刻（无 stamp 的 chunk 在该窗口内一律拒收）；首个新鲜 chunk 落地后
  才解除门控。
- 门控期间只保持 hold 位姿：新增 `joint_states_topic`（默认
  `/gento/joint_states`）订阅，交还瞬间用大臂反馈刷新 hold 目标；该窗口内不再
  发夹爪指令（避免抢人手刚放开的夹爪）。
- `_policy_callback` 现在只在 `AUTONOMOUS` 下装载 chunk（原来 `HUMAN` 下也会装）。

## I3/I4 driver 双路径互斥

`skye_robot_driver`：

- 接受 relative 指令成功后，若该臂 abs 会话在流式中 → `reset_absolute_session()`
  （反之亦然），下次切回时会从反馈重新 seed，避免 `limit_delta` 参考错位。
- `check_command_timeout()` 新增 `path_active()` 判断：同一条臂的另一路径仍在活跃
  流式时，只失效本路径会话，不再调用 `hold_current()` 与之抢控。

## I5 QoS 对齐

arbiter 的 teleop 订阅（`/skye/teleop_action_*`、`/skye/teleop_gripper_*`、
`/skye/policy_action`）与命令发布（`/gento/*_joint_control_abs`、
`/gento/*_joint_control`、`/*_teleop_gripper/ctrl`）统一 KeepLast(1) +
BEST_EFFORT + VOLATILE，与 driver `control_qos()` 一致。

## I6 control_mode 心跳

`/skye/control_mode` 改为 TRANSIENT_LOCAL + RELIABLE，并按 `mode_publish_hz`
（默认 5 Hz）定时发布，晚启动的 recorder 也能拿到最后模式。

## 验证

- `colcon build --packages-select skye_hitl_dagger skye_robot_driver --cmake-args
  -DPython3_EXECUTABLE=/usr/bin/python3`：通过。
- `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest
  src/skye_hitl_dagger/test -q`：34 passed。
- `ctest`（skye_robot_driver）：1/1 passed。
- 节点内联冒烟：`takeover` → `TELEOP_SYNCING`（仍 WAIT_ALIGNED）→ `SYNCED`
  （发 switch_teleop，转 WAIT_TELEOP）→ 无新状态时保持 `HANDOVER_SYNC` → 新
  `TELEOP` → `HUMAN` → `return` → `AUTONOMOUS` 且 chunk 门控置位。

## 遗留

- 真机未验证：`SYNCED` 字面量取自 `docs/小臂大臂启动步骤.md`
  （`IDLE`/`TELEOP_SYNCING`/`SYNCED`/`TELEOP`），若 FACTR 版本改字符串需同步
  `teleop_sync.ALIGNED_TOKEN`。
- 交还后 hold 夹爪值沿用最后一次策略值，门控解除瞬间夹爪可能有一次跳变（幅度取决
  于人手放开时的开合差）。
- driver 双路径互斥仅有节点级逻辑，无 C++ 单测覆盖（`DriverNode` 未做可测拆分）。

## C2 / I7-I9 最终修复

- episode recorder：命令/action 订阅 KeepLast(1)+BEST_EFFORT；joint_states RELIABLE，control_mode RELIABLE+TRANSIENT_LOCAL。
- P6.1 verify 的 policy/abs 检查使用 BEST_EFFORT，control_mode 使用 RELIABLE+TRANSIENT_LOCAL。
- `w` 在 HANDOVER_SYNC 也可中止，清理 handshake、同步计时器和缓存并回 AUTONOMOUS。
- 交还后 2.0 s 内无新时间戳 chunk 时按接收时间兜底，stamp=0 直接按接收时间处理并只告警一次。
- 交还时反馈超过 0.2 s 则清空 hold target，暂停绝对位置发布，直到新反馈或新 policy chunk。
- 验证：`colcon build --packages-select skye_hitl_dagger`、38 个 pytest、P6.1 verify 均通过。
