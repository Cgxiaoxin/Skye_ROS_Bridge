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

## skye_operator_ui 终审修复（feat/skye-operator-ui）

### Critical

- **C1 抢占执行器**：`ros_bridge._call_trigger` 不再调用
  `rclpy.spin_until_future_complete(self._node, ...)`（会从 `MultiThreadedExecutor`
  手里抢走该节点的回调）。改为 `_await_future()` 轮询 `future.done()`
  （`TRIGGER_POLL_S=0.01`，`TRIGGER_TIMEOUT_S=2.0`），由既有执行器经
  `ReentrantCallbackGroup` 投递响应；超时时 `future.cancel()`。全仓已无其它
  `spin_until_*` 调用该节点。
- **C2 pending_op 永不清除**：新增 `PendingTracker`（`api_app.py`）。
  - 派发成功后才置位（失败即清）；
  - `_PENDING_CONFIRMERS` 按状态确认提前解除（如 `align_start` → `ALIGNING`，
    `recorder_stop` → `recording_active=False`，`takeover` → `hitl_mode=HUMAN`）；
  - 无可观测状态的 op（`emergency_stop` / `hold_current` / `stop_motion`）派发即清；
  - 兜底超时 `pending_timeout_s`（默认 8 s，`config/default.yaml` 可配），在 10 Hz
    `tick_loop` 与 snapshot 路径均会 resolve，按钮必定解锁。
- **C3 MARVIN_LAUNCH_CMD**：`scripts/run_marvin_m6_impedance.sh` 在
  `MARVIN_LAUNCH_CMD` 非空时执行 `docker run ... bash -lc "$MARVIN_LAUNCH_CMD"`；
  `-it` 改为 `-i`，仅当 `[[ -t 0 ]]` 时追加 `-t`，从而可在 supervisor 管道下运行。
  未设置时保持原交互式 shell 行为（脚本头部已注明）。`config/default.yaml` 的
  `playbook.marvin_launch_cmd` 补上 teleop / HITL 的完整可用示例命令。

### Important

- **I1 录制器先停后拆**：`RosBridge.stop_recorder_if_active()`（best-effort，吞异常）；
  `operator_ui_node` 的 `on_before_stop` 先停录制再 `set_session_mode(None)`；
  supervisor 新增 `on_degraded` 回调，`tick()` 里 `mark_degraded()` 成功后触发
  （节点侧放到后台线程，避免阻塞 asyncio 循环）。
- **I2 会话停止清缓存**：`RosBridge.clear_session_cache()` 复位 `teleop_state`、
  `align_status`、`hitl_mode/source`、`recording_active` 及 `align_stamp` /
  `control_mode_stamp`；由 `set_session_mode(None)` 自动调用。
- **I7 前端急停顺序**：`web/src/lib/commands.js` 将 `emergency_stop` 短路提到
  `pending_op` 闸门之前，与后端 `command_allowed` 一致。
- **I3 cleanup 配置为空**：`run_cleanup_stale()` 返回明确中文
  「未配置清理命令：config/default.yaml 的 cleanup_stale_commands 为空…」；
  yaml 中补注释示例（pkill / docker rm）。
- **I4 日志抽屉层级**：`.topbar` 设 `position: relative; z-index: 200`，高于
  `.drawer-backdrop` 的 100，抽屉不再遮挡急停。
- **I5 tick 容错**：`tick_loop` 对 `supervisor.tick()` 与 pending resolve 分别
  try/except 并 `logger.exception`，单次异常不再终止 10 Hz 循环。
  `supervisor.stop()` 对 `on_before_stop` 同样容错。

### 顺带修复

- `lifespan` 安装 SIGINT handler 时捕获 `ValueError`（非主线程场景），并只在安装
  成功时恢复；此前在 TestClient / 嵌入式线程下会直接抛错。
- 测试假件从 `conftest.py` 移到可导入的 `test/fakes.py`，`test_api_app.py` 自带
  同名 fixture —— 指定的 `--noconftest` 命令此前 7 failed + 7 errors，现已全绿。

### 测试

```
cd skye_ros2_ws && PYTHONPATH=src/skye_operator_ui PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  python3 -m pytest src/skye_operator_ui/test/ -v --noconftest

test_api_app.py::test_command_rejects_unknown PASSED
test_api_app.py::test_session_start_body PASSED
test_api_app.py::test_snapshot_endpoint PASSED
test_api_app.py::test_command_rejects_before_ready PASSED
test_api_app.py::test_emergency_stop_allowed_in_idle_when_bridge_available PASSED
test_api_app.py::test_session_stop PASSED
test_api_app.py::test_logs_endpoint PASSED
test_api_app.py::test_pending_op_set_then_cleared_by_state PASSED
test_api_app.py::test_pending_op_cleared_on_timeout PASSED
test_api_app.py::test_pending_op_not_set_for_unconfirmable_op PASSED
test_api_app.py::test_pending_op_not_set_when_dispatch_fails PASSED
test_api_app.py::test_session_stop_clears_pending PASSED
test_api_app.py::test_tick_loop_survives_supervisor_exception PASSED
test_api_app.py::test_pending_tracker_confirmers[recorder_start-mailbox0-None] PASSED
test_api_app.py::test_pending_tracker_confirmers[recorder_start-mailbox1-recorder_start] PASSED
test_api_app.py::test_pending_tracker_confirmers[takeover-mailbox2-None] PASSED
test_api_app.py::test_pending_tracker_confirmers[takeover-mailbox3-takeover] PASSED
test_ros_bridge.py::test_await_future_returns_true_when_future_completes PASSED
test_ros_bridge.py::test_await_future_times_out PASSED
test_ros_bridge.py::test_clear_session_cache_resets_latched_state PASSED
test_ros_bridge.py::test_set_session_mode_none_clears_cache PASSED
test_ros_bridge.py::test_stop_recorder_if_active_noop_when_not_recording PASSED
test_ros_bridge.py::test_stop_recorder_if_active_dispatches_stop PASSED
test_ros_bridge.py::test_stop_recorder_if_active_swallows_errors PASSED
test_supervisor.py::test_degrade_invokes_callback_once PASSED
test_supervisor.py::test_on_before_stop_failure_does_not_block_stop PASSED
test_supervisor.py::test_cleanup_stale_reports_empty_config PASSED
（其余 commands / hints / process_step / session_state / snapshot / supervisor 用例略）

================== 66 passed, 1 skipped, 1 warning in 19.97s ===================
```

同命令去掉 `--noconftest` 亦为 66 passed, 1 skipped。
`bash -n scripts/run_marvin_m6_impedance.sh` 通过；
`cd src/skye_operator_ui/web && npm run build` 通过（index-CptyS2eM.js / index-DsERsGmS.css）。

### 遗留

- `_PENDING_CONFIRMERS` 的状态字面量（`TELEOP_SYNCING` / `SYNCED` / `TELEOP` /
  `ALIGNING` / `ALIGNED` 等）沿用 `hints.py` 与 `commands.py` 的既有约定，未在真机
  校验；若 FACTR 改字符串需三处同步。确认失败时仍有 8 s 超时兜底。
- `switch_stop` 的确认条件是「teleop_state 不再是 TELEOP/TELEOP_SYNCING」，若停止后
  话题不再更新则依赖超时清除。
- 容器内 `MARVIN_LAUNCH_CMD` 的具体 launch 行仍未在真机跑通（脚本与 yaml 已给出
  可用示例，需现场确认 overlay 路径）。
- `commands.js` 无 JS 单测框架，I7 仅靠与 Python `command_allowed` 的对读保证一致。
