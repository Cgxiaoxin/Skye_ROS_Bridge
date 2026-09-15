# Skye 操作员上位机 UI（遥操数采 / HITL DAgger）设计

**日期：** 2026-09-15  
**状态：** 设计已确认，待实现计划  
**仓库落点：** `skye_ros2_ws/src/skye_operator_ui`（ROS2 包，不另起同级仓库）  
**相关文档：**
- [`docs/Thor_Orin_遥操启动.md`](../../Thor_Orin_遥操启动.md)
- [`docs/Hint_Dagger启动使用说明.md`](../../Hint_Dagger启动使用说明.md)
- [`docs/ros_interfaces.md`](../../ros_interfaces.md)
- [`docs/superpowers/specs/2026-08-21-hitl-dagger-control-arbiter-design.md`](2026-08-21-hitl-dagger-control-arbiter-design.md)

---

## 1. 目标与非目标

### 目标

- 给**数采人员**一套本机上位机 UI，减少多终端 / 键盘焦点依赖，完成：
  - **遥操数采**：会话启动 → sync → 对齐 → 开遥操 → `skye_data_recorder` 起停 → 有序退出
  - **HITL DAgger**：会话启动 → 观察 `control_mode` → takeover / return → HITL recorder 起停 → 有序退出
- 界面美观、主路径清晰；长时间开启不拖垮主机上的 `skye_robot_driver` 控制环。
- 一键会话启停封装现有 `scripts/*.sh`，带预检、逐步健康门闩、可预期退出。

### 非目标（v1 不做）

- 真 VLA / 推理机一键拉起（可选手动；dummy 可选）。
- 改阻抗、限位、标定、串口绑定向导。
- 关节点动、相机墙、实时多曲线示波器、Foxglove 替代。
- 平板触控优先布局（预留 Web，v1 按笔记本鼠标）。
- 局域网多客户端裸奔；Electron 壳。
- 在 UI 进程内重写 FACTR launch 逻辑。

### 约束（已锁定）

| 项 | 选择 |
|----|------|
| 用户 | 数采人员 |
| 设备 | 上位机笔记本浏览器 |
| 模式 | 顶栏切换「遥操数采 / DAgger」，同一套 UI |
| Profile | 启动前必选 thor/orin，**会话锁**；改机台只需结束会话，**不必重启 UI 进程** |
| 进程 | 会话级一键启停（封装现有脚本）+ 健康检查 |
| 技术栈 | **FastAPI + Svelte 静态前端**；本机 `127.0.0.1` |
| 包位置 | `skye_ros2_ws/src/skye_operator_ui` |
| 实现期 skills | 仅 `frontend-design`、`ui-ux-pro-max`（控制 token） |

---

## 2. 架构

### 2.1 运行时拓扑

```text
浏览器 (Svelte，建议 Chrome/Edge --app=http://127.0.0.1:<port>)
    │  REST：会话启停 / 白名单命令
    │  WebSocket：~10 Hz 状态快照 + 按需日志
    ▼
operator_ui 单进程
    ├── FastAPI（主线程或专用线程）
    ├── rclpy spin 线程（订阅状态；发 mode topic / 调 service）
    └── SessionSupervisor（子进程组 + 门闩 + 看门狗）
            │ 仅调用仓库现有 scripts/
            ▼
     skye_robot_driver · Docker FACTR · follower_align
     · skye_data_recorder / HITL arbiter ·（可选）dummy policy
```

### 2.2 包布局

```text
skye_ros2_ws/src/skye_operator_ui/
├── skye_operator_ui/       # Python：API、Supervisor、ROS 桥、快照
├── web/                    # Svelte 源码；构建产物进 share/.../web
├── launch/operator_ui.launch.py
├── config/default.yaml     # 端口、超时、剧本步骤、清理白名单
├── package.xml
└── setup.py
```

### 2.3 硬边界

| 做 | 不做 |
|----|------|
| 会话启停、预检、健康灯、中文下一步提示 | 订阅 `*_action_applied`（拖驱动） |
| sync / 对齐 / 开遥操 / 录制 / takeover·return / 急停 | 从 Web 发 `/gento/*_joint_control` 关节指令 |
| profile 会话锁、模式互斥 | 热切换遥操 launch ↔ HITL launch |
| 步骤环形日志 | 默默全盘 `pkill`（清理须用户确认 + 白名单） |

### 2.4 性能约定

- 状态推送 **约 10 Hz**；命令事件化。
- 与 driver **同机、同 `ROS_DOMAIN_ID=21`**，FastDDS 关 SHM 配置与现网一致。
- 生产只服静态构建，**运行时无 Node**。
- UI 不得进入 4 ms / ~250 Hz 控制环。

---

## 3. Session 状态机与启停

### 3.1 状态

```text
IDLE ──start──► PRECHECK ──ok──► STARTING ──all healthy──► READY
                  │                  │
                  └──── fail ────────┴──► FAILED
READY ──操作/录制──► RUNNING
任意非 IDLE ──end session──► STOPPING ──► IDLE
READY/RUNNING ──看门狗失败──► DEGRADED（禁新录；在录则自动 stop）
```

### 3.2 启动锁

- **Profile**（thor/orin）：仅 `IDLE` 可选；选定后写入会话并顶栏锁定。
- **模式**（`teleop_record` | `dagger`）：仅 `IDLE` 可切；禁止热切换。
- 已有非 IDLE 会话时禁止再次 start。

### 3.3 PRECHECK（失败不拉进程）

1. profile / 模式已选  
2. 控制器可达（`ping 6.6.7.190`，超时可配）  
3. FastDDS xml 存在；环境与现网脚本一致  
4. 无第二个 SDK 客户端；若有 → 提示；仅用户点「清理残留」走白名单清理  
5. 剧本所需脚本路径存在  

### 3.4 STARTING 剧本

只封装现有脚本，逐步超时 + 独立环形日志（默认约 2000 行/步）。成功条件以 **topic/service/节点出现** 为准，不只看进程 exit code。

**遥操数采（`teleop_record`）**

| 步 | 动作 | 成功门闩（可配置） |
|----|------|-------------------|
| 1 | `scripts/start_skye_for_factr.sh`（注入 `ROBOT_PROFILE`） | `/gento/joint_states` 有数据 |
| 2 | `scripts/run_marvin_m6_impedance.sh` + 日常 teleop launch | 小臂相关状态/节点可见 |
| 3 | （可选）`scripts/start_follower_align.sh` | align 节点在 |
| 4 | `skye_data_recorder` launch | `/skye/data_recorder/start\|stop` 可用 |

**DAgger（`dagger`）**

| 步 | 动作 | 成功门闩 |
|----|------|----------|
| 1 | 同大臂 driver | 同上 |
| 2 | Docker + **HITL** launch（禁止与日常 teleop launch 同时） | HITL remap 链路就绪 |
| 3 | `scripts/start_hitl_host.sh --arbiter-only` | `/skye/control_mode` 有数据 |
| 4 | （可选）dummy policy；真 VLA 默认不自动起 | — |
| 5 | （可选）HITL `/skye/recorder/*` | service 可用 |

失败 → `FAILED`：提供 **重试本步** / **结束会话**。

### 3.5 READY / RUNNING 操作（只走 ROS，不起进程）

- 遥操：`/mode/switch_sync|teleop|stop`；`/mode/align_follower|align_cancel`；`/skye/data_recorder/start|stop`
- DAgger：`/skye/intervention_cmd`（`takeover`|`return`）；`/skye/recorder/start|stop`
- 语义：**已发出 → 等状态确认**；超时琥珀提示，不假绿  
  - 例：`return` 仅在 `HUMAN`（及设计允许的状态）下可点，与现网键盘 `w` 行为一致  
  - `HANDOVER_SYNC`：展示对齐等待，不提供第二主按钮

### 3.6 看门狗

关键健康项（driver topic、Docker/arbiter 视模式）消失 → `DEGRADED`：禁止新开录制；若正在录则自动 stop 并红条，避免废 episode。

### 3.7 退出路径

| 入口 | 行为 |
|------|------|
| 结束会话 | 若在录 → stop recorder → 停 Docker/HITL/align → 最后停 driver；进程组 SIGTERM→超时 SIGKILL；完成后 `IDLE` |
| 急停 | 只调 `/gento/emergency_stop`；**不**拆会话 |
| 关浏览器 | 会话继续；重开页显示「会话仍在跑」 |
| 关 UI 进程 | v1：确认后 **结束会话并退出** |
| 模式切换 / 改 profile | 仅 `IDLE` |

### 3.8 明确不做（减 bug 面）

- 不在 UI 内复制一份 launch 参数真相源  
- 不默默全盘杀进程  
- Web 不发关节控制 topic  

---

## 4. 界面信息架构

### 4.1 顶栏（共用）

`SKYE | [THOR|ORIN]锁 | 遥操数采 | DAgger | 会话状态 | 时间 | [结束会话] | [急停]`

- 急停：红色、最大、无确认  
- 结束会话：二次确认  

### 4.2 左栏：步骤 / 健康

- 遥操：预检 → 大臂 → 小臂 → Sync → 对齐 → 遥操 → 数采  
- DAgger：预检 → 大臂 → HITL 小臂 → Arbiter → 策略源 → 控制权 → 录制  
- 每步可展开该步日志尾  

### 4.3 中栏：主状态 + 主操作

- 遥操：向导式，未到步按钮禁用；`TIMEOUT_WARN` 用琥珀文案「仍可开遥操」  
- DAgger：大字 `AUTONOMOUS` / `HANDOVER_SYNC` / `HUMAN`；主按钮随模式切换「接管」「交还」  

### 4.4 右栏：轻量反馈

左右 7 轴细条（近限位醒目）、夹爪开合、一行中文「下一步建议」（raw topic 可折叠详情）。

### 4.5 底栏

单行事件 + 红/琥珀横幅；不弹窗遮挡急停。

### 4.6 视觉

深色操作台、大字号、状态色仅中性/绿/琥珀/红。实现阶段用 `frontend-design` 与 `ui-ux-pro-max` 收紧细节；不做手机优先。

---

## 5. 接口与数据流

### 5.1 前端 ↔ operator_ui（默认只绑 127.0.0.1）

| 方法 | 路径 | 作用 |
|------|------|------|
| WS | `/ws/state` | ~10 Hz 快照 |
| GET | `/api/snapshot` | 调试用 |
| POST | `/api/session/start` | `{profile, mode}` |
| POST | `/api/session/stop` | 有序结束 |
| POST | `/api/session/retry_step` | FAILED 重试当前步 |
| POST | `/api/session/cleanup_stale` | 确认后白名单清理 |
| POST | `/api/command` | 白名单 `op` |
| GET | `/api/logs/{step}` | 日志尾 |

### 5.2 命令白名单 `op`

| op | ROS |
|----|-----|
| `switch_sync` / `switch_teleop` / `switch_stop` | `/mode/switch_*` |
| `align_start` / `align_cancel` | `/mode/align_follower` / `align_cancel` |
| `recorder_start` / `recorder_stop` | 按模式：`/skye/data_recorder/*` 或 `/skye/recorder/*` |
| `takeover` / `return` | `/skye/intervention_cmd` |
| `emergency_stop` | `/gento/emergency_stop` |
| `hold_current` / `stop_motion` | 对应 Trigger（可放「更多」） |

未知 `op` → 400。禁止任何关节 `JointState` 下发入口。

### 5.3 ROS 订阅（UI）

`/gento/joint_states`、`/gento/robot_state`、`/left|right_gripper/state`、`/teleop/state`、`/align/status`、`/skye/control_mode`；另加门闩/看门狗所需的轻量探测。

**禁止：** `*_action_applied`；高频控制 topic 的常驻订阅。

### 5.4 快照（概念字段）

`session`（state/profile/mode/step）、`teleop`、`align`、`hitl`、`recording`、`arms`、`grippers`、`health[]`、`banner`、`next_hint`、`pending_op`。

### 5.5 命令回执

HTTP 接受 ≠ 成功。以状态 topic 变化确认；超时琥珀，不假绿。急停优先，不受向导门禁拦截。

---

## 6. 失败处理

| 类 | 行为 |
|----|------|
| 预检失败 | FAILED，不拉进程，中文原因 + 建议 |
| 启动步失败 | 保留已起进程；重试本步 / 结束会话；展开该步日志 |
| 命令未确认 | 琥珀 banner；可重发或停止 |
| DEGRADED | 禁新录；在录自动 stop |
| 误操作 | 按钮禁用 + API 短路，不发 ROS |

清理残留：仅 config 白名单命令。日志与子进程输出必须有界。

---

## 7. 测试与验收

### 7.1 自动化

- 单元：Session 非法迁移拒绝；命令白名单；快照降采样  
- 集成（无真机）：mock 脚本 + 假 topic，跑通 PRECHECK→STARTING→READY→STOPPING 与超时  
- 契约：HTTP/WS；确认不存在关节下发 API  

### 7.2 真机验收（Thor / Orin 各至少一次）

1. 遥操：启动 → sync → 对齐 → 遥操 → 录约 30s → 停录 → 结束会话，无 SDK 残留  
2. DAgger：启动 → AUTONOMOUS → takeover → HUMAN → return → 结束会话  
3. 急停不拆会话；模式/profile 仅 IDLE 可改  
4. UI 常开约 1h：内存无明显爬升；`/gento/joint_states` hz 不因 UI 明显下降  

### 7.3 成功标准（数采员）

- 不必为找键盘焦点开多个终端，即可完成一条遥操数采  
- DAgger 接管/交还不手打 `ros2 topic pub`  
- 失败时能定位步骤并「结束会话」收场  

---

## 8. 组件边界（实现时拆分）

| 单元 | 职责 | 依赖 |
|------|------|------|
| `SessionSupervisor` | 状态机、剧本、进程组、日志环、看门狗 | 脚本路径 config |
| `RosBridge` | 订阅→邮箱；白名单 publish/service | rclpy |
| `SnapshotBuilder` | 10 Hz 聚合中文 hint / banner | RosBridge + Supervisor |
| `ApiApp` | FastAPI + WS | 以上 |
| `web/` | Svelte 视图与命令按钮 | 仅 HTTP/WS |

各单元可单测；Supervisor 不导入 Svelte；RosBridge 不起子进程。

---

## 9. 延期项

真 VLA 拉起、相机预览、废 episode 删除、平板布局、多客户端鉴权、关 UI 保留孤儿会话高级选项。
