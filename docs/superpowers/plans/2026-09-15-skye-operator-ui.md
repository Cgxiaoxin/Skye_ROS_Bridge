# Skye Operator UI Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 落地本机上位机 UI（FastAPI + Svelte），让数采员用浏览器完成遥操数采与 HITL DAgger 的会话启停、模式操作与录制，而不依赖多终端键盘焦点。

**Architecture:** 新 ament_python 包 `skye_operator_ui`：纯逻辑 Session 状态机 + 进程组 Supervisor + RosBridge + SnapshotBuilder，经 FastAPI/WebSocket 喂给 Svelte 静态前端；只封装现有 `scripts/*.sh`，不进控制环、不订 `*_action_applied`。

**Tech Stack:** ROS2 Humble, Python 3.10, rclpy, FastAPI, uvicorn, websockets, Svelte 4 + Vite, pytest

**Spec:** `docs/superpowers/specs/2026-09-15-skye-operator-ui-design.md`

## Global Constraints

- 包路径：`skye_ros2_ws/src/skye_operator_ui`（不另起同级仓库）
- 绑定：`127.0.0.1` only（v1）
- Profile：`thor`|`orin`，仅 IDLE 可选，会话锁
- 模式：`teleop_record`|`dagger`，仅 IDLE 可切；禁止热切换 launch
- 状态推送 ~10 Hz；禁止订阅 `*_action_applied`；禁止 Web 发 `/gento/*_joint_control`
- 命令白名单见 spec §5.2；急停走 `/gento/emergency_stop`，不拆会话
- 关浏览器不杀会话；关 UI 进程 = 结束会话并退出
- 真 VLA 不自动拉起；dummy policy 可选
- 实现期仅用 skills：`frontend-design`、`ui-ux-pro-max`
- pip 依赖（FastAPI/uvicorn/websockets）写入 `requirements.txt`，README 写明 `pip install -r`

---

## File map

| File | Role |
|------|------|
| `skye_ros2_ws/src/skye_operator_ui/package.xml` | ament_python 元数据 |
| `skye_ros2_ws/src/skye_operator_ui/setup.py` / `setup.cfg` / `resource/skye_operator_ui` | 安装与 entry point |
| `skye_ros2_ws/src/skye_operator_ui/requirements.txt` | fastapi uvicorn websockets |
| `skye_ros2_ws/src/skye_operator_ui/config/default.yaml` | 端口、超时、剧本、清理白名单 |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/session_state.py` | Session 状态机纯逻辑 |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/commands.py` | HTTP op → 内部命令枚举 + 门禁 |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/hints.py` | `next_hint` 中文 |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/process_step.py` | 单步子进程 + 环形日志 |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/playbooks.py` | teleop/dagger 剧本定义 |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/supervisor.py` | SessionSupervisor |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/ros_bridge.py` | 订阅邮箱 + 白名单 pub/srv |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/snapshot.py` | 10 Hz 快照 |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/api_app.py` | FastAPI + WS |
| `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/operator_ui_node.py` | 组装入口 |
| `skye_ros2_ws/src/skye_operator_ui/scripts/operator_ui` | console 包装 |
| `skye_ros2_ws/src/skye_operator_ui/launch/operator_ui.launch.py` | launch |
| `skye_ros2_ws/src/skye_operator_ui/web/` | Svelte 源码 |
| `skye_ros2_ws/src/skye_operator_ui/test/test_session_state.py` | 状态机 |
| `skye_ros2_ws/src/skye_operator_ui/test/test_commands.py` | 白名单/门禁 |
| `skye_ros2_ws/src/skye_operator_ui/test/test_supervisor.py` | mock 剧本启停 |
| `skye_ros2_ws/src/skye_operator_ui/test/test_api_app.py` | HTTP 契约 |
| `scripts/start_operator_ui.sh` | 环境 + 启动 |
| `docs/Operator_UI使用说明.md` | 数采员说明 |
| `docs/ros_interfaces.md` | 补一行 UI 包索引 |

---

### Task 1: 包骨架 + Session 状态机

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/package.xml`
- Create: `skye_ros2_ws/src/skye_operator_ui/setup.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/setup.cfg`
- Create: `skye_ros2_ws/src/skye_operator_ui/resource/skye_operator_ui`
- Create: `skye_ros2_ws/src/skye_operator_ui/requirements.txt`
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/__init__.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/session_state.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/test/test_session_state.py`

**Interfaces:**
- Produces:
  - `SessionState` enum: `IDLE|PRECHECK|STARTING|READY|RUNNING|FAILED|STOPPING|DEGRADED`
  - `UiMode` enum: `teleop_record|dagger`
  - `class SessionLogic`:
    - `state() -> SessionState`
    - `profile() -> str | None` / `mode() -> UiMode | None`
    - `begin_start(profile: str, mode: UiMode) -> bool`  # IDLE only; sets PRECHECK
    - `precheck_ok() -> bool` / `precheck_fail() -> bool`
    - `enter_starting() -> bool` / `mark_ready() -> bool` / `mark_failed() -> bool`
    - `mark_running() -> bool` / `mark_degraded() -> bool`
    - `begin_stop() -> bool` / `mark_idle() -> bool`
    - `can_change_profile_or_mode() -> bool`  # only IDLE

- [ ] **Step 1: Write failing tests**

```python
# test/test_session_state.py
from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode

def test_start_only_from_idle():
    s = SessionLogic()
    assert s.begin_start("thor", UiMode.teleop_record)
    assert s.state() == SessionState.PRECHECK
    assert not s.begin_start("orin", UiMode.dagger)

def test_happy_path_to_ready_and_stop():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    assert s.precheck_ok()
    assert s.state() == SessionState.STARTING
    assert s.mark_ready()
    assert s.state() == SessionState.READY
    assert s.begin_stop()
    assert s.state() == SessionState.STOPPING
    assert s.mark_idle()
    assert s.state() == SessionState.IDLE

def test_profile_locked_until_idle():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    assert not s.can_change_profile_or_mode()
    s.precheck_fail()
    s.begin_stop()
    s.mark_idle()
    assert s.can_change_profile_or_mode()

def test_degraded_from_ready():
    s = SessionLogic()
    s.begin_start("thor", UiMode.teleop_record)
    s.precheck_ok()
    s.mark_ready()
    assert s.mark_degraded()
    assert s.state() == SessionState.DEGRADED
```

- [ ] **Step 2: Run tests — expect FAIL (import error)**

```bash
cd skye_ros2_ws && python3 -m pytest src/skye_operator_ui/test/test_session_state.py -v
```

Expected: FAIL collecting / ModuleNotFoundError

- [ ] **Step 3: Scaffold package + implement `session_state.py`**

`package.xml`：仿 `skye_data_recorder`，`build_type` ament_python；depend `rclpy` `std_msgs` `std_srvs` `sensor_msgs`；test_depend `pytest`。

`requirements.txt`:
```text
fastapi>=0.110
uvicorn[standard]>=0.27
websockets>=12
PyYAML>=6
```

`session_state.py`：按 Interfaces 实现合法迁移表；非法迁移返回 `False` 且不改状态。`begin_start` 校验 `profile in {"thor","orin"}`。

- [ ] **Step 4: Run tests — expect PASS**

```bash
PYTHONPATH=src/skye_operator_ui python3 -m pytest src/skye_operator_ui/test/test_session_state.py -v
```

Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_operator_ui
git commit -m "feat(operator_ui): add package skeleton and session state machine"
```

---

### Task 2: 命令白名单与 hint

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/commands.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/hints.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/test/test_commands.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/test/test_hints.py`

**Interfaces:**
- Produces:
  - `ALLOWED_OPS: frozenset[str]`
  - `def validate_op(op: str) -> bool`
  - `def command_allowed(*, op: str, session: SessionLogic, teleop_state: str | None, hitl_mode: str | None, align_status: str | None) -> tuple[bool, str]`
    - 返回 `(ok, reason_zh)`；`emergency_stop` 任意非 IDLE 会话均可
    - `return` 仅当 `hitl_mode == "HUMAN"`
    - `takeover` 仅当 `hitl_mode == "AUTONOMOUS"`
    - `switch_teleop` 建议要求 teleop 已 `SYNCED`（或 align 结束）；未满足返回 False
  - `def next_hint(...) -> str`

- [ ] **Step 1: Write failing tests**

```python
from skye_operator_ui.commands import validate_op, command_allowed
from skye_operator_ui.session_state import SessionLogic, UiMode

def test_unknown_op_rejected():
    assert not validate_op("joint_control")
    assert validate_op("emergency_stop")

def test_return_only_in_human():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    s.precheck_ok(); s.mark_ready()
    ok, _ = command_allowed(op="return", session=s, teleop_state=None, hitl_mode="AUTONOMOUS", align_status=None)
    assert not ok
    ok, _ = command_allowed(op="return", session=s, teleop_state=None, hitl_mode="HUMAN", align_status=None)
    assert ok

def test_takeover_only_autonomous():
    s = SessionLogic()
    s.begin_start("thor", UiMode.dagger)
    s.precheck_ok(); s.mark_ready()
    ok, _ = command_allowed(op="takeover", session=s, teleop_state=None, hitl_mode="HUMAN", align_status=None)
    assert not ok
```

- [ ] **Step 2: Run — expect FAIL**

```bash
PYTHONPATH=src/skye_operator_ui python3 -m pytest src/skye_operator_ui/test/test_commands.py -v
```

- [ ] **Step 3: Implement `commands.py` + `hints.py`**

`ALLOWED_OPS` 精确包含 spec §5.2 全部 op。`hints.py` 用简单 if/elif 覆盖：IDLE 未选 profile、PRECHECK/STARTING/FAILED、遥操 SYNCED 可对齐、DAgger AUTONOMOUS 可接管等。

- [ ] **Step 4: Run — expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(operator_ui): add command whitelist and Chinese next_hint"
```

---

### Task 3: ProcessStep + playbooks（可 mock）

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/process_step.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/playbooks.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/config/default.yaml`
- Create: `skye_ros2_ws/src/skye_operator_ui/test/test_process_step.py`

**Interfaces:**
- Produces:
  - `class LogRing: append(line: str); tail(n: int) -> list[str]`  # maxlen 可配，默认 2000
  - `class ProcessStep:`
    - `__init__(self, step_id: str, argv: list[str], env: dict[str,str], health_check: Callable[[], bool], timeout_s: float)`
    - `start() -> None`  # start_new_session=True 进程组
    - `poll_health() -> bool`
    - `terminate(grace_s: float = 5.0) -> None`
    - `logs.tail(n: int) -> list[str]`
  - `def playbook_for(mode: UiMode, repo_root: str, profile: str, cfg: dict) -> list[dict]`
    - 每项：`{id, argv, env, health_key, timeout_s, optional: bool}`
  - `default.yaml` 含：`bind_host`, `port`, `step_timeout_s`, `log_ring_size`, `cleanup_stale_commands`（字符串列表白名单）, `playbook` 覆盖点

- [ ] **Step 1: Write failing test with mock argv**

```python
import time
from skye_operator_ui.process_step import ProcessStep, LogRing

def test_log_ring_caps():
    r = LogRing(maxlen=3)
    for i in range(5):
        r.append(str(i))
    assert r.tail(10) == ["2", "3", "4"]

def test_process_step_start_and_terminate():
    step = ProcessStep(
        step_id="sleep",
        argv=["/bin/sleep", "30"],
        env={},
        health_check=lambda: True,
        timeout_s=2.0,
    )
    step.start()
    assert step.poll_health()
    step.terminate(grace_s=1.0)
    # process should be gone
    assert step._proc.poll() is not None  # noqa: private ok in test
```

- [ ] **Step 2: Run — expect FAIL**

- [ ] **Step 3: Implement ProcessStep**

要求：`subprocess.Popen(..., start_new_session=True, stdout=PIPE, stderr=STDOUT, text=True)`；后台线程读 stdout 写入 `LogRing`；`terminate` 先 `os.killpg(pid, SIGTERM)` 再超时 `SIGKILL`。

`playbooks.py`：`teleop_record` / `dagger` 返回步骤列表；argv 指向 `${repo_root}/scripts/start_skye_for_factr.sh` 等；`env` 含 `ROBOT_PROFILE`。Docker 步可先写成「调用 `run_marvin_m6_impedance.sh`」——若脚本是交互进容器，**本任务在 playbook 注释与 yaml 中标明**：v1 对该步使用「可替换的 wrapper 命令」键 `marvin_start_cmd`，默认仍调现有脚本；集成时若脚本只进 shell 不 launch，在 Task 8 文档写清需 `MARVIN_LAUNCH_CMD` 环境变量覆盖为非交互 launch 行（与 Hint 文档容器内命令一致）。

- [ ] **Step 4: Run — expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(operator_ui): add process step runner and playbook defs"
```

---

### Task 4: SessionSupervisor

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/supervisor.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/test/test_supervisor.py`

**Interfaces:**
- Consumes: `SessionLogic`, `playbook_for`, `ProcessStep`
- Produces:
  - `class SessionSupervisor:`
    - `__init__(self, repo_root: str, cfg: dict, health_fn: Callable[[str], bool])`
      - `health_fn(health_key) -> bool` 由外部注入（测试 mock；线上接 RosBridge）
    - `start(profile: str, mode: UiMode) -> tuple[bool, str]`
    - `retry_step() -> tuple[bool, str]`
    - `stop() -> tuple[bool, str]`
    - `tick() -> None`  # STARTING 推进 / 看门狗 / READY→检测
    - `snapshot_fields() -> dict`  # session 段字段
    - `step_logs(step_id: str, n: int = 200) -> list[str]`
    - `run_cleanup_stale() -> tuple[bool, str]`  # 仅执行 cfg 白名单 argv

行为：
1. `start` → `begin_start` → 跑预检回调（注入 `precheck_fn`）→ 失败 `precheck_fail`；成功 `precheck_ok` 后按 playbook 逐步 `start`，每步循环 `tick` 直到 `health_fn` 真或超时 → 超时 `mark_failed`
2. 全部成功 `mark_ready`
3. `stop` → 若提供 `on_before_stop` 可先停录（Task 5 接）；逆序 `terminate` 已起步骤 → `mark_idle`
4. READY/RUNNING 时若 `health_fn("driver")` 假 → `mark_degraded`

- [ ] **Step 1: Write failing integration-style unit test**

```python
from skye_operator_ui.supervisor import SessionSupervisor
from skye_operator_ui.session_state import SessionState, UiMode

def test_supervisor_mock_playbook(tmp_path, monkeypatch):
    cfg = {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": [],
        "precheck": {"skip_ping": True},
        "playbook_override": {
            "teleop_record": [
                {"id": "a", "argv": ["/bin/sleep", "60"], "health_key": "a", "timeout_s": 3.0, "optional": False},
            ]
        },
    }
    healthy = {"a": False}

    def health(key):
        return healthy.get(key, False)

    sup = SessionSupervisor(repo_root=str(tmp_path), cfg=cfg, health_fn=health,
                            precheck_fn=lambda: (True, "ok"))
    ok, _ = sup.start("thor", UiMode.teleop_record)
    assert ok
    # still starting
    for _ in range(5):
        sup.tick()
    healthy["a"] = True
    for _ in range(20):
        sup.tick()
        if sup.logic.state() == SessionState.READY:
            break
    assert sup.logic.state() == SessionState.READY
    ok, _ = sup.stop()
    assert ok
    assert sup.logic.state() == SessionState.IDLE
```

（实现时 `playbook_override` 优先于默认 playbooks。）

- [ ] **Step 2: Run — expect FAIL**

- [ ] **Step 3: Implement supervisor**

预检默认：`skip_ping` 假时 `ping -c 1 -W 1 6.6.7.190`；检查 FastDDS xml 相对 `repo_root/marvin_ws/fastrtps_no_shm.xml`；SDK 残留检测可用 `pgrep -fc skye_robot_driver`（>0 且非本会话拉起前 → 失败并提示 cleanup）。v1 简化：若 `pgrep` 已有 driver 且 playbook 第一步是 driver，可配置 `allow_existing_driver: true`（DAgger arbiter-only 场景）。

- [ ] **Step 4: Run — expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(operator_ui): add SessionSupervisor with mockable playbook"
```

---

### Task 5: RosBridge + SnapshotBuilder

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/ros_bridge.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/snapshot.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/test/test_snapshot.py`

**Interfaces:**
- Produces:
  - `class RosBridge`（可对 Node 组合）:
    - 订阅并缓存最新：`joint_states`, `robot_state`, grippers, `teleop/state`, `align/status`, `control_mode`
    - `health_key` 映射：`driver` ← joint_states 新鲜（`now - stamp < 1.0s`）；`align` ← 节点/话题；`arbiter` ← control_mode 新鲜
    - `dispatch(op: str) -> tuple[bool, str]`：按白名单发 String / 调 Trigger
    - **不**创建 joint_control 发布者
  - `class SnapshotBuilder:`
    - `build(supervisor, bridge, pending_op) -> dict` 符合 spec §5.4

- [ ] **Step 1: Write failing test for snapshot pure merge**（bridge 用假对象）

```python
class FakeBridge:
    def mailbox(self):
        return {
            "teleop_state": "SYNCED",
            "align_status": "IDLE",
            "hitl_mode": None,
            "hitl_source": None,
            "left_joints": [0.0]*7,
            "right_joints": [0.0]*7,
            "left_gripper": 0.0,
            "right_gripper": 0.0,
            "health": {"driver": True},
        }

def test_snapshot_contains_hint_and_session(monkeypatch):
    from skye_operator_ui.snapshot import SnapshotBuilder
    from skye_operator_ui.session_state import SessionLogic, UiMode, SessionState
    logic = SessionLogic()
    logic.begin_start("thor", UiMode.teleop_record)
    logic.precheck_ok(); logic.mark_ready()

    class FakeSup:
        def snapshot_fields(self):
            return {"state": logic.state().name, "profile": "thor", "mode": "teleop_record", "step": None}
        logic = logic

    snap = SnapshotBuilder().build(FakeSup(), FakeBridge(), pending_op=None)
    assert snap["session"]["profile"] == "thor"
    assert "next_hint" in snap
    assert snap["teleop"]["state"] == "SYNCED"
```

- [ ] **Step 2–4: Implement + PASS**

`dispatch` 映射表写死在 `ros_bridge.py`：
- mode topics → `std_msgs/String` data 与现网一致（`switch_sync` 等）
- recorder：`teleop_record` → `/skye/data_recorder/*`；`dagger` → `/skye/recorder/*`
- intervention → `/skye/intervention_cmd`
- e-stop → `/gento/emergency_stop`

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(operator_ui): add RosBridge and snapshot builder"
```

---

### Task 6: FastAPI + WebSocket + 节点入口

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/api_app.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/operator_ui_node.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/scripts/operator_ui`
- Create: `skye_ros2_ws/src/skye_operator_ui/launch/operator_ui.launch.py`
- Create: `skye_ros2_ws/src/skye_operator_ui/test/test_api_app.py`
- Modify: `setup.py`（entry_points + data_files for launch/config/web placeholder）

**Interfaces:**
- Produces HTTP 与 spec §5.1 一致
- `create_app(supervisor, bridge, snapshot_builder) -> FastAPI`
- 节点：`rclpy` 线程 spin；主线程/uvicorn 跑 API；后台 10 Hz `supervisor.tick()` + 广播 WS

- [ ] **Step 1: Write API tests with TestClient + fake supervisor**

```python
from fastapi.testclient import TestClient
from skye_operator_ui.api_app import create_app

def test_command_rejects_unknown(fake_stack):
    app = create_app(*fake_stack)
    c = TestClient(app)
    r = c.post("/api/command", json={"op": "joint_control"})
    assert r.status_code == 400

def test_session_start_body(fake_stack):
    app = create_app(*fake_stack)
    c = TestClient(app)
    r = c.post("/api/session/start", json={"profile": "thor", "mode": "teleop_record"})
    assert r.status_code in (200, 202)
```

- [ ] **Step 2: Run — expect FAIL**

- [ ] **Step 3: Implement `api_app.py`**

- 静态文件：若 `share/.../web` 或源码 `web/dist` 存在则挂载 `/`
- WS `/ws/state`：循环 `await websocket.send_json(snapshot)`，`asyncio.sleep(0.1)`
- `POST /api/command`：先 `validate_op` + `command_allowed`，再 `bridge.dispatch`；急停跳过向导门禁但仍要会话非「完全无机器人」时可在 IDLE 也允许（spec：急停优先——实现为 **只要 bridge 可用即允许**）
- 关闭钩子：进程 SIGINT → `supervisor.stop()`

- [ ] **Step 4: `operator_ui_node.py` + launch + colcon build**

```bash
cd skye_ros2_ws
pip3 install -r src/skye_operator_ui/requirements.txt
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_operator_ui
source install/setup.bash
ros2 run skye_operator_ui operator_ui --ros-args -p repo_root:=$PWD/.. 
```

（参数名以代码为准；默认探测 `Skye_ROS_Bridge` 根。）

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(operator_ui): add FastAPI WebSocket server and ROS entrypoint"
```

---

### Task 7: Svelte 壳 + 顶栏 + 会话控制

**Files:**
- Create: `skye_ros2_ws/src/skye_operator_ui/web/package.json`
- Create: `skye_ros2_ws/src/skye_operator_ui/web/vite.config.js`（`outDir: dist`, base `./`）
- Create: `skye_ros2_ws/src/skye_operator_ui/web/index.html`
- Create: `skye_ros2_ws/src/skye_operator_ui/web/src/main.js`
- Create: `skye_ros2_ws/src/skye_operator_ui/web/src/App.svelte`
- Create: `skye_ros2_ws/src/skye_operator_ui/web/src/lib/api.js`
- Create: `skye_ros2_ws/src/skye_operator_ui/web/src/lib/ws.js`
- Create: `skye_ros2_ws/src/skye_operator_ui/web/src/components/TopBar.svelte`
- Create: `skye_ros2_ws/src/skye_operator_ui/web/src/styles.css`

实现期读取 **frontend-design** 与 **ui-ux-pro-max**：深色操作台、大按钮、状态色仅绿/琥珀/红/中性。

- [ ] **Step 1: Scaffold Vite+Svelte，`api.js` 封装 start/stop/command**

```js
export async function postCommand(op) {
  const r = await fetch('/api/command', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({op}),
  });
  if (!r.ok) throw new Error(await r.text());
  return r.json();
}
```

- [ ] **Step 2: TopBar** — profile 选择（仅 IDLE）、模式切换（仅 IDLE）、会话徽章、结束会话（confirm）、急停（无 confirm，调用 `emergency_stop`）

- [ ] **Step 3: App** — 连接 `/ws/state`，断线重连；展示 `next_hint` 与 banner

- [ ] **Step 4: Build and verify static serve**

```bash
cd skye_ros2_ws/src/skye_operator_ui/web && npm install && npm run build
```

`setup.py` `data_files` 安装 `web/dist` → `share/skye_operator_ui/web`。

- [ ] **Step 5: Commit**

```bash
git commit -m "feat(operator_ui): add Svelte shell with session top bar"
```

---

### Task 8: 遥操数采视图 + DAgger 视图

**Files:**
- Create: `web/src/components/StepRail.svelte`
- Create: `web/src/components/TeleopPanel.svelte`
- Create: `web/src/components/DaggerPanel.svelte`
- Create: `web/src/components/ArmStrip.svelte`
- Create: `web/src/components/LogDrawer.svelte`
- Modify: `App.svelte`

- [ ] **Step 1: StepRail** — 按 `session.mode` 渲染步骤列表；点击打开 `LogDrawer` fetch `/api/logs/{step}`

- [ ] **Step 2: TeleopPanel** — 按钮：`switch_sync`、`align_start`、`align_cancel`、`switch_teleop`、`switch_stop`、`recorder_start`、`recorder_stop`；依 snapshot 禁用

- [ ] **Step 3: DaggerPanel** — 大字 hitl mode；接管/交还；HITL recorder；HANDOVER_SYNC 禁用交还/接管

- [ ] **Step 4: ArmStrip** — 14 值细条；`near_limit` 时可后接（v1 用固定限位表或省略，仅显示数值条）

- [ ] **Step 5: 构建 + 手动点检清单写入 PR 描述**；Commit

```bash
git commit -m "feat(operator_ui): add teleop and DAgger operator panels"
```

---

### Task 9: 启动脚本、文档、验收清单

**Files:**
- Create: `scripts/start_operator_ui.sh`
- Create: `docs/Operator_UI使用说明.md`
- Modify: `docs/ros_interfaces.md`（文末加 Operator UI 索引，链到 spec + 本说明）
- Create: `skye_ros2_ws/scripts/verify_operator_ui_api.sh`（无真机：起节点 + curl start 用 mock cfg）

`start_operator_ui.sh`：
```bash
#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
# shellcheck disable=SC1091
source "${SCRIPT_DIR}/lib/ros_domain_env.sh"
source "${REPO_ROOT}/skye_ros2_ws/install/setup.bash"
exec ros2 run skye_operator_ui operator_ui --ros-args -p repo_root:="${REPO_ROOT}"
```

文档写清：浏览器打开 `http://127.0.0.1:8765`（端口以 yaml 为准）；建议 `--app=`；会话启停与急停区别；Docker 步若需人工确认的已知限制。

- [ ] **Step 1: 写脚本与文档**
- [ ] **Step 2: verify 脚本至少 curl `/api/snapshot` 返回 200**
- [ ] **Step 3: Commit**

```bash
git commit -m "docs: add operator UI usage notes and start script"
```

---

### Task 10: 真机冒烟（手工，可勾选）

不写代码；执行 spec §7.2：

- [ ] Thor 遥操数采全路径
- [ ] Thor DAgger takeover/return
- [ ] 急停不拆会话；结束会话无 SDK 残留
- [ ] 常开 UI 观察 driver hz
- [ ] Orin 至少遥操路径一次（profile 锁）

将结果补记到 `docs/Operator_UI使用说明.md` 顶部「真机状态」一行。

---

## Spec coverage（自检）

| Spec 项 | Task |
|---------|------|
| 包落点 / FastAPI+Svelte | 1, 6, 7 |
| Session 状态机与退出 | 1, 4 |
| Playbook 启停 / 清理白名单 | 3, 4 |
| 命令白名单 / 急停 | 2, 5, 6 |
| 双模式 UI | 7, 8 |
| 10 Hz 快照 / 禁止 applied | 5, 6 |
| 文档与验收 | 9, 10 |

## Placeholder scan

无 TBD/TODO；Docker 非交互 launch 通过 `marvin_start_cmd` / 环境变量覆盖写明，避免假实现。

## Type consistency

- `UiMode.teleop_record` / `UiMode.dagger` 贯穿 Task 1–8
- `SessionState` 名与 snapshot `session.state` 使用 `.name`
- HTTP `mode` 字符串与枚举 value 一致

---

## Execution

Plan 完成后请选择执行方式（见下一条消息交付路径）。
