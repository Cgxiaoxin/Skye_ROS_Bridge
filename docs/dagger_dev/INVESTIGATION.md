# DAgger 策略下发链路 — 调查结论（T0.1–T0.9）

> 日期：2026-11-12  
> 范围：本仓本地代码（含 `docs/dagger_dev/client_thor/vendor/remote_inference_async`、`server5090/deploy`、`skye_ros2_ws`）。  
> **未做实机 `ros2 topic echo/hz`**（当前环境无运行中的 ROS 图）；话题字段以 driver/arbiter 源码与文档为准。  
> 原则：找不到写「未找到」；与任务书矛盾处以代码为准。

---

## 总览：与任务书 §1.2 / §2 的关键差异

| 任务书假设 | 本地代码实际 | 影响 |
|---|---|---|
| DAgger 把「反馈→slew 目标」插成 16 步、`dt≈2.08ms`（整段 1/30s）再 pub | **直接把模型原始 `policy_chunk`（绝对 rad）** 经 `dagger_chunk.policy_chunk_to_arbiter_fields` pad/截断到 16 步，**`dt = 1/model_action_hz`（≈1/30s）** 发布 | §2 根因 1/2/3 中「微 chunk 锯齿」**当前不成立**；慢/做不满更可能来自 **硬替换 chunk + rebase-to-feedback + NullGripper + hold tail** |
| slew 参照反馈导致开环丢失 | 本地执行环仍用 **`last_sent_action` 开环 slew**（旧路径一致）；但 **该 command 不发到 arbiter**，只用于 dry_run 轨迹/历史 | P0-a 不能只「改参照」；需要 **改发布内容**，让口 A 与 `last_cmd` 连续 |
| driver 可能未设 impedance vel/acc | `skye_robot.yaml` 已是 **vel/acc ratio = 100**；`connect_and_enable` → `enter_mode` / `SetSpeedRatio` 使用这些值 | P0-b 阻抗补设：**基本无需改**（只核实读回） |
| `max_delta` 主要打策略 | **relative 与 absolute 共用**同一 `max_delta_per_cycle` | P0-b 按话题分档仍然必要 |

---

## T0.1 `/gento/joint_states` 与夹爪反馈

### 要查什么 / 怎么查
- driver 发布维度、夹爪话题；launch remap；文档对照。

### 结论
| 项 | 结论 |
|---|---|
| `/gento/joint_states` | **14 维**：左 7 + 右 7；**不含夹爪**。`DriverNode::publish_state` 拼 `left_position`+`right_position`；launch remap `/joint_states` → `/gento/joint_states`。文档：`skye_robot_driver/Thor大臂消息查看.md`。 |
| 夹爪反馈话题 | **`/left_gripper/state`、`/right_gripper/state`**（**未** remap 到 `/gento/`）。`JointState`：`name=["gripper_joint"]`，`position[0]` = FACTR 归一化 **[0,1]**（0 开 / 1 闭），经 `motor_to_factr_norm`（默认 invert）。频率 ≈ `gripper_rate_hz`（yaml **100 Hz**）。 |
| 夹爪 applied | `/gento/left|right_gripper_action_applied`（电机空间目标）。 |
| DAgger facade | `DaggerRosRobotFacade` 只订 `/gento/joint_states`，取 `position[:14]`；夹爪走 `NullGripper`。 |

**实机 `ros2 topic echo --once` / `hz`：本机未跑，未实测。** 源码与现网文档一致。

### 用于
- P0-c：**方案 A** — 订 `/left_gripper/state` + `/right_gripper/state`。  
- **不做方案 B**（夹爪不在 joint_states 里）。

---

## T0.2 `max_delta_per_cycle`

### 结论
| 项 | 结论 |
|---|---|
| 定义 | 相邻 **成功下发** 的臂目标之间，逐关节 `|Δ|` 上限（rad / **被接收的命令帧**），不是固定 rad/s。 |
| absolute | `handle_absolute_command` → `limit_delta(..., max_delta_per_cycle_)`。 |
| relative | `handle_relative_command` 同样 `limit_delta`；teleop seed ramp 可临时放大 `× teleop_seed_ramp_max_delta_scale`。 |
| 分档 | **无**；双路径共用同一参数（yaml 现网 **0.01**）。 |
| 夹爪 | **`handle_gripper_command` 不走 `limit_delta`**，直接 `set_target`。夹爪**不被**臂的 `max_delta` 限制。 |
| 「命令帧」 | 每条入站 `JointState` 回调成功 `send_position` 算一帧；频率随上游（arbiter abs ≈250Hz / teleop relative）。 |

证据：`driver_node.cpp` ~945–950、1042；`driver_core.cpp::limit_delta`；checklist `phase7_4_left_teleop_checklist.md` §9.2。

### 用于
- P0-b：新增 `max_delta_abs_per_cycle`（建议 0.02）仅 abs；relative **保持 0.01**；夹爪单独配置（见下，当前无臂式 delta，可不加限速除非要显式保护）。

---

## T0.3 driver impedance vel/acc vs 旧 `prepare_arm`

### 结论
| 侧 | 值 |
|---|---|
| 旧 client | `run_robot.sh`：`IMPEDANCE_VELOCITY/ACCELERATION_PERCENT` 默认 **100**；`prepare_arm` → `switch_to_imp_joint_mode` + `runtime_set_vel/acc_ratio`。 |
| driver | yaml：`left/right_velocity_ratio: 100`，`left/right_acceleration_ratio: 100`；连接时写入 `ConnectConfig`，模式进入与 `set_speed_rates` 使用同一套。 |

**结论：与旧链路 100% 对齐，P0-b 不必再补设。** 若实机读回不是 100，再查连接失败/被服务覆盖。

---

## T0.4 client：slew / `last_sent_action` / hard_jump / DAgger chunk

### 位置
| 符号 | 文件 |
|---|---|
| 主循环 / dagger 分支 | `docs/dagger_dev/client_thor/vendor/remote_inference_async/async_robot_client.py` |
| `validate_action` / `prepare_arm` / `execute_action` | `.../robot_control.py` |
| DAgger 打包/发布 | `dagger_chunk.py`、`dagger_publisher.py` |
| ROS facade / NullGripper | `dagger_ros_facade.py` |
| 入口 | `run_robot_dagger.sh` → `--dagger-mode`（强制 dry_run 语义） |

### 旧链路行为（仍在同一文件非 dagger 路径）
1. **`hard_jump`（`validate_action`）**  
   - 比较对象：`arm_reference=last_accepted_action`（**上一次接受的策略目标**），不是反馈。  
   - 超限：**按比例投影整段双臂 delta**（不整块丢弃）。  
   - 夹爪：相对 **当前反馈** 做 `max_gripper_step` slew；不因夹爪大跳拒绝整臂。
2. **stream slew**  
   - 参照 **`last_sent_action`（上一次真正发出的指令）**。  
   - 仅 **臂关节** 按 `interpolation_step_deg`（默认 3°）比例缩放；**夹爪不参与该臂 slew**，用 postprocess/direct 目标。  
   - 注释写明：`interpolation_start="last_sent_action"`。
3. **观测**  
   - `state_from_rt(反馈, gripper.state(...))`；臂度→策略 rad，夹爪保持归一化。

### DAgger 当前行为（关键）
```text
收到 policy_chunk
  → dagger_pub.publish_policy_actions(policy_chunk, dt=1/model_action_hz)   # 原始未来窗
  → 本地仍 merge ActionQueue → validate → slew(last_sent_action) → dry_run（不写 SDK）
```
- **口 A 发布的不是 slewed command**，而是模型原始绝对轨迹（截断/垫到 16）。  
- `NullGripper.state()` 恒 **0**；`diagnostics()` 仅 `{"backend": "dagger_null_gripper"}`。  
- 主循环在 `decision_gripper["left"]["contact_latched"]` 处 **会 KeyError**（除非现场另有补丁）。P0-c 必须一并修。

### 用于
- P0-a：保持 hard_jump = `last_accepted_action`；夹爪不参与臂 slew。  
- **额外必做**：口 A 改为基于 **`last_cmd`→本拍 slewed 目标**（或未来窗衔接 `last_cmd`）的连续指令，而不是只改本地参照。  
- 任务书「16×2.08ms 微 chunk」**不是当前实现**；P1-b 的「30Hz×16 自然窗」更接近现状发布语义，但需解决硬替换/过期步。

---

## T0.5 arbiter ChunkPlayer / rebase

### 结论
| 项 | 行为 |
|---|---|
| 新 chunk | `ChunkPlayer.load` **整块硬替换**（无交叉淡化）。 |
| 时间基准 | `t0 = header.stamp`（ROS clock）；stamp 不可用时回退 `_now_seconds()`（`get_clock()`）。采样用同一 clock，**非**显式 `time.monotonic()` 墙钟。 |
| 采样 | 250Hz；按 `step_t0s` **阶梯取整步**（非步间线性插值）。 |
| 播完 | `holding_tail=True`，继续发末姿态；另有 `_last_target` 兜底。 |
| `chunk_start_max_jump_rad`（默认 0.1） | 比较 **chunk step0** vs **`/gento/joint_states` 反馈**；超限则 `rebase_arm_joints_to_pose`（平移整段保 delta）。**不是** vs 上次输出 cmd。 |

文件：`chunk_player.py`、`chunk_continuity.py`、`control_arbiter_node._policy_callback` / `_joint_timer_callback`。

### 用于
- P1-b：改为墙钟插值 + blend + rebase vs **上次输出**；与 P0-a 互补。  
- 当前 rebase-to-feedback 在臂滞后时会把指令钉回反馈，放大「做不满」。

---

## T0.6 `DualGentoDmGripper.state()` 与 contact latch

### `state()` / `_snapshot`
- 返回：`np.ndarray` shape **(2,)**，`float32`，顺序 **[left, right]**。  
- 单位：**calibrated 归一化 [0,1]**（0 开 / 1 闭）=`(raw - low) / (high - low)` clip。  
- 协议注释：`STATE_GRIPPER_UNIT = calibrated_0_open_1_closed`（`units.py`）。  
- 动作侧：`ACTION_GRIPPER_UNIT = command_0_open_0.9_max_grasp`（`GRIPPER_COMMAND_MAX=0.9`）。

### contact latch（`ContactGripController` in `gento_dm_gripper.py`）
- 闭合推进中：力矩 ≥ `contact_torque_nm` 且速度 ≤ `contact_velocity_rad_s` 连续 `contact_cycles` → `contact_latched=True`，停在当前位置。  
- 打开请求（目标明显小于当前）→ 清 latch。  
- 松开后可用 `regrip_*` 再抓。  
- client 另有 `GripperHoldController` / `GripperCommandProcessor` 用 `diagnostics()["left|right"]["contact_latched"]` 做策略命令保持。

### 与 ROS `/left|right_gripper/state` 对齐注意
- driver 发 FACTR [0,1] + invert；DM4310 电机空间用固定 `pos_min/max`（0–1.6）归一化，**不是** client 启动时实测 low/high 标定。  
- 数值应大致同属 [0,1]，但与旧 `DualGentoDmGripper` 标定曲线可能有偏差 → P0-c 启动时必须与训练 stats 对照。

### 用于
- P0-c：`RosGripperFeedback.state()` 必须 `(2,) float32 [0,1]`；`diagnostics` 至少提供 `left/right.contact_latched`（可先恒 False + 后续复现 latch，或基于 effort/velocity 近似）。

---

## T0.7 基础数采 / 训练标签 / action_history

### 本仓能确认的
| 项 | 结论 |
|---|---|
| 遥操数采真源 | `skye_data_recorder`：action = **`/gento/*_joint_action_applied` + `*_gripper_action_applied`**（driver 成功下发后的指令）；观测 = `/gento/joint_states` + `/left|right_gripper/state`。见 `docs/ros_interfaces.md`、`2026-09-04-applied-action-data-collection-design.md`。 |
| 录制频率 | recorder **事件驱动**（跟消息）；`joint_states` 设计 **250Hz**；applied 随命令帧。离线训到 ~30Hz：**未找到**本仓重采样脚本。 |
| client `action_history` | `ExecutedActionHistory`：**已发布的 robot command**（经 `robot_action_to_policy`），不是反馈。注释：`s(t)` + history 末为 `a(t-1)`，对齐训练 `[idx-K, idx)`。 |
| 本地归一化样例 | `server5090/deploy/downliao_normalization.json`：`fps≈29.98`，`hist_stride=2`，`action_horizon=50`；夹爪 `state_high≈[0.0032, 1.1667]`，`action_high≈[0.0, 0.93]`。这是 **UniJEPA/downliao**，**不是**完整 pi05 训练集 meta。 |

### 未找到（需用户确认 / 另机代码）
1. **pi05 微调** 数据加载与 stats（本仓无 pi05 dataset meta）。  
2. 离线：action 标签是严格 **t 时刻 applied** 还是 **t+k 对齐**。  
3. `hist_stride=2` 与 client `ACTION_HISTORY_HORIZON`（dagger 脚本默认 30，任务书写 8）如何对应。

### 用于
- P1-c / P2：接管片段应用 **`skye_data_recorder` 同口径**（applied + joint_states），不要用 `/skye/teleop_action_*`。  
- action_history 填 **指令**；HUMAN 期间也要按真实执行轨迹更新（当前 client **未订** `/skye/control_mode`，交还重置未做）。

---

## T0.8 `PolicyActionChunk` 消息

### 定义（`skye_hitl_dagger/msg/PolicyActionChunk.msg`）
```
std_msgs/Header header
string policy_version
uint32 chunk_size
float64 dt
float64[] left_joints      # chunk_size * 7
float64[] right_joints
float64[] left_gripper     # chunk_size
float64[] right_gripper
```
- 语义上对应 **[L7, R7, Lg, Rg]** 逐步展开；**无**显式 `positions[N][dims]` 二维字段。  
- 生成：`CMakeLists.txt` → `rosidl_generate_interfaces`；改 msg 后需 `colcon build --packages-select skye_hitl_dagger`。

### 用于
- P1-b：现有字段已够表达「stamp + dt + 16 步」；优先 **不改 msg**，改填值约定（`dt=1/30`、stamp=第一步时刻）。若要二维布局再开 msg 变更（单独 commit）。

---

## T0.9 部署拓扑 / 时钟 / `control_mode`

| 项 | 结论 |
|---|---|
| 正式拓扑 | Thor：driver + arbiter（+ 可选 policy client）；笔记本：teleop/UI；5090：推理。见 `skye-hitl-dagger-refactor-plan.md` §0.1、`hitl_dagger_thor_现场联调步骤.md`。 |
| client 与 arbiter | **同机（Thor）为正式推荐**；同机可用同一 ROS clock。跨机未在脚本里强制 NTP/chrony。 |
| client 订 `/skye/control_mode`？ | **否**。仅 arbiter 发布；`episode_recorder` / UI 订阅。DAgger client **不**根据模式停发 policy_action / 重置 history。 |

### 用于
- P1-b：同机 → stamp/墙钟以 ROS clock 为主即可；跨机再改到达时间+相对偏移。  
- P1-c：client **新增**订阅 `/skye/control_mode`。

---

## 附加发现（任务书未单列）

1. **NullGripper.diagnostics 不完整** → 主循环 KeyError 风险；P0-c 必修。  
2. **口 A 发布原始策略窗 + arbiter rebase-to-feedback + 硬替换**，比「反馈参照 slew」更能解释当前症状。  
3. downliao 归一化里 **左爪 state 几乎恒 0**；若用该文件做 P0-c 范围校验，可能误杀 — 需 pi05 真实 stats。  
4. `run_client_dagger.sh` 默认 `ACTION_HISTORY_HORIZON=30`，与任务书「8」及 pi05_history 配置可能不一致 — **按用户要求不改模型/history 配置**，但交还填充逻辑要对齐训练定义。

---

## 对修改优先级的建议（供 plan 用）

| ID | 建议 |
|---|---|
| P0-a | **确定做**，但实现改为：维护 `last_cmd`；slew 仍相对 `last_cmd`；**发布的 chunk 以 `last_cmd` 为起点衔接**（微窗或本拍单步+pad）；加 `ERR_MAX`；观测仍用反馈。 |
| P0-b | **确定做** abs/teleop 分档；阻抗 **核实即可**；夹爪确认不受臂 delta 误伤（已确认）。 |
| P0-c | **确定做** `RosGripperFeedback`；修 diagnostics；训练范围校验需用户给 pi05 stats 或放宽为「非常数 + 落在 [0,1]」。 |
| P1 | 在 P0 实机/V1 验证后再做；P1-b 与当前「已是 1/30 dt」更合拍。 |
| P2 | 单独一轮；标签口径已有 `skye_data_recorder`。 |

---

## 仍需向用户确认（§7）

1. ~~joint_states 维数 / 夹爪话题~~ → **已由代码确认**（14 + 独立 gripper state）。  
2. **pi05** 训练：action 时间对齐、history 长度/stride、夹爪 state 的 min/max（不要只用 downliao json）。  
3. 实机允许的 `max_vel_abs`（P1-a）；P0 阶段 abs 先用 **0.02 rad/帧** 兜底是否可接受。

---

## T0.10 raw 模式 stamp / 延迟补偿 / 截断 / 过期处理（方案 A 决策补充）

### raw 模式下 `header.stamp` 取什么？
**发布时刻的 ROS clock**（`DaggerPolicyPublisher.publish_policy_actions` → `self._node.get_clock().now()`），**不是** `t_obs`，也不是模型 chunk 内的绝对动作时间。

### 是否做过延迟补偿（跳过已过去的步）？
**没有。** `dagger_chunk.policy_chunk_to_arbiter_fields` 只做形状规范化；不读 now、不丢过期步。推理延迟未在口 A 侧补偿。

### 16 步截断在哪里做？
`policy_chunk_to_arbiter_fields`（`dagger_chunk.py`）：
- `T > 16` → **保留前 16 步**（丢掉更远未来）；
- `T < 16` → **重复最后一姿态** pad 到 16；
- `ChunkPlayer.load` 硬要求 `chunk_size == 16`，否则拒收。

### arbiter 对 stamp 过去 / 将来如何处理？
`control_arbiter_node._policy_callback`：`t0 = header.stamp`（fresh）或回退 `_now_seconds()`。  
`ChunkPlayer.sample(t_now)`：
| stamp 相对 now | 行为 |
|---|---|
| **将来**（`t_now ≤ t0`） | 一直输出 **step0**，直到时钟追上 |
| **过去但仍在窗内** | 直接跳到对应 step index（阶梯，非线性插值） |
| **整窗已过**（`t_now ≥ end_t`） | `holding_tail=True`，锁 **最后一步** |

因此 raw 下若推理慢、仍用「发布时刻」做 `t0`，但内容是「观测时刻起的未来窗」，时间语义本就错位；再叠加硬替换，容易出现 hold-tail 与相位跳跃。  
**setpoint 微窗**（整段 ≈1/30s、stamp=发布时刻）与「从现在起的短程 ramp」更一致；轻微发布延迟只会让采样落在 ramp 中后段，不会像长窗那样整段过期。

### 首个 chunk 为何不受 `chunk_start_max_jump` 误伤？
setpoint 进入 AUTONOMOUS / 启动时 `last_cmd := fb`，故 step0≈反馈，jump≈0 < 阈值。稳态 `last_cmd` 相对 fb 的领先由 `ERR_MAX`（默认 0.15 rad）封顶；把 `chunk_start_max_jump_rad` **配置到 0.3**（> ERR_MAX）即可避免把合法领先 rebase 掉。**不改 rebase 代码。**

---

## 方案 A 决策（相对 Claude 建议的取舍）

**采纳：**
1. `--dagger-publish-mode raw|setpoint`，默认 **setpoint**；raw 作对照。  
2. setpoint：每 30Hz 执行拍，用本地 `validate → slew(last_sent_action)` 得到的 `command` 为终点；起点=上一拍已发布 cmd；16 步线性插值；`dt=1/(model_action_hz×16)`；`stamp=发布时刻`。  
3. `ERR_MAX`（默认 0.15 rad）+ 持续 0.5s 告警。  
4. arbiter **仅改配置** `chunk_start_max_jump_rad: 0.3`（thor/orin profile）。  
5. 本 commit 夹爪字段跟 command 走；P0-c 再换真反馈。  
6. commit 顺序：日志 → A → P0-b → P0-c。

**校正 / 强调：**
- 当前 raw 发布点在 **policy 回包时**，不是每拍；setpoint 必须把发布挪到 **slew 之后的执行拍**（否则仍发原始长窗）。  
- client 臂状态是 **degree**；`ERR_MAX` 按 rad 配置时比较前要 `deg2rad`。  
- ChunkPlayer 是 **阶梯采样** 不是步间线性插值；16×2.08ms 窗在 250Hz 下约命中 8 个阶梯点，对短 ramp 足够。  
- commit0 日志先合入后，**先用 raw+pi05 打基线**再切 setpoint 默认（实现上默认 setpoint，但现场基线用显式 `--dagger-publish-mode raw`）。
