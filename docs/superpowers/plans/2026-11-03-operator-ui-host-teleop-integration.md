# Operator UI Host Teleop Integration Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Wire Operator UI `teleop_record` sessions to host `scripts/teleop.sh` (unified driver+leaders), remove Docker Marvin / follower_align from that path, keep sync-before leader gate on the host, and apply the startup/teardown efficiency fixes from the spec.

**Architecture:** Session precheck runs `teleop.sh check --robot <profile> --fix-latency`. Playbook becomes two steps: `teleop` (`teleop.sh start --robot … --skip-precheck`) then `recorder`. Health for `teleop` requires fresh `/gento/joint_states` and a latched `/teleop/state`. Leader off/on and teardown disable Dynamixel on the host using profile YAML ports. DAgger playbook stays on the old Docker path this round.

**Tech Stack:** Python 3.10, pytest, FastAPI, Svelte, bash, ROS 2 Humble, `scripts/teleop.py`, `skye_factr_teleop`, `skye_operator_ui`

**Spec:** `docs/superpowers/specs/2026-11-03-operator-ui-host-teleop-integration-design.md`

## Global Constraints

- `teleop_record` only; do not change DAgger playbook steps this round.
- UI exposes only `thor` / `orin` (no A/B device picker).
- Session must never call `teleop.sh build`.
- Default pairing: `thor` → `config/`; `orin` → `config/orin/`; CLI `--device a|b` may override.
- Remove follower_align from teleop_record path.
- Keep leader left/right gate API; implement on host (nodes `skye_factr_teleop_{left,right}`).
- No automatic `bind_leader_arms.py` in session start.

## File map

| File | Responsibility |
|------|----------------|
| `scripts/teleop.py` | robot→config default; `--skip-precheck`; install file check (no `ros2 pkg prefix`) |
| `scripts/tests/test_teleop_entrypoint.py` | Update pairing / skip-precheck tests |
| `scripts/disable_leader_dynamixel.py` | Load ports from profile YAML when env missing |
| `scripts/leader_arm_gate.sh` | Host off/on (pkill + disable / relaunch side) |
| `skye_operator_ui/.../playbooks.py` | `teleop_record` → `teleop` + `recorder` |
| `skye_operator_ui/.../ros_bridge.py` | `health("teleop")`; snapshot key rename |
| `skye_operator_ui/.../supervisor.py` | Mode-aware precheck via `teleop.sh check` |
| `skye_operator_ui/.../teardown.py` | Host disable default; optional docker |
| `skye_operator_ui/.../operator_ui_node.py` | `hold_current` best-effort on stop |
| `skye_operator_ui/.../commands.py` + `hints.py` | Drop align teleop gate; hints |
| `skye_operator_ui/config/default.yaml` | cleanup / teardown defaults |
| `web/src/lib/steps.js`, `commands.js`, `TeleopPanel.svelte` | Steps + remove align UI |
| `docs/Operator_UI使用说明.md` | Host teleop docs |

---

### Task 1: `teleop.py` — robot pairing, `--skip-precheck`, install check

**Files:**
- Modify: `scripts/teleop.py`
- Test: `scripts/tests/test_teleop_entrypoint.py`

**Interfaces:**
- Produces: `device_for_robot(profile: str) -> str` returning `"a"` for thor, `"b"` for orin
- Produces: CLI `start|check|env` with `--skip-precheck` on `start` only
- Produces: when `--device` is `auto` (default), use `device_for_robot(profile)` then `select_leaders(side, device)`
- Consumes: existing `select_leaders`, `check_ports`, `launch_command`

- [ ] **Step 1: Write failing tests for robot→device default and skip-precheck**

Add/replace in `scripts/tests/test_teleop_entrypoint.py`:

```python
def test_device_for_robot():
    assert teleop.device_for_robot("thor") == "a"
    assert teleop.device_for_robot("orin") == "b"


@pytest.mark.parametrize(
    ("robot", "device_dir"),
    [("thor", "config"), ("orin", "config/orin")],
)
def test_start_defaults_device_to_robot_pair(usb, monkeypatch, robot, device_dir):
    # Only the paired USB set is connected.
    device = "a" if robot == "thor" else "b"
    configs = {side: usb(device, side) for side in ("left", "right")}
    captured = []
    monkeypatch.setattr(teleop, "check_ports", lambda *a, **kw: None)
    monkeypatch.setattr(teleop, "ros_environment", lambda: {})
    monkeypatch.setattr(teleop.os, "chdir", lambda *a: None)
    monkeypatch.setattr(Path, "is_file", lambda path: True)
    monkeypatch.setattr(
        teleop.subprocess,
        "run",
        lambda cmd, **kw: subprocess.CompletedProcess(
            cmd, 1 if cmd[0] == "pgrep" else 0
        ),
    )
    monkeypatch.setattr(
        teleop.os, "execvpe", lambda exe, cmd, env: captured.append((cmd, env))
    )
    assert teleop.main(["start", "--robot", robot, "--skip-precheck"]) == 0
    command, env = captured[0]
    assert f"robot_profile:={robot}" in command
    for side, config in configs.items():
        assert f"{side}_config:={config}" in command
        assert device_dir in str(config)


def test_skip_precheck_skips_port_ping_and_install_probe(usb, monkeypatch):
    usb("a", "left")
    usb("a", "right")
    calls = []
    monkeypatch.setattr(
        teleop,
        "check_ports",
        lambda *a, **kw: calls.append("check_ports") or (_ for _ in ()).throw(
            AssertionError("check_ports should not run")
        ),
    )
    monkeypatch.setattr(teleop, "ros_environment", lambda: {})
    monkeypatch.setattr(teleop.os, "chdir", lambda *a: None)
    monkeypatch.setattr(Path, "is_file", lambda path: True)

    def run(cmd, **kw):
        calls.append(cmd[0])
        if cmd[0] == "pgrep":
            return subprocess.CompletedProcess(cmd, 1)
        if cmd[0] == "ping":
            raise AssertionError("ping should not run with --skip-precheck")
        return subprocess.CompletedProcess(cmd, 0)

    monkeypatch.setattr(teleop.subprocess, "run", run)
    captured = []
    monkeypatch.setattr(
        teleop.os, "execvpe", lambda exe, cmd, env: captured.append(cmd)
    )
    assert teleop.main(["start", "--robot", "thor", "--skip-precheck"]) == 0
    assert "ping" not in calls
    assert captured
```

Update existing `test_cross_pairing_and_environment` so `--robot thor` without `--device` expects device **a** ports only (connect `usb("a", …)`). Keep an explicit override test:

```python
def test_explicit_device_overrides_robot_pair(usb, monkeypatch):
    configs = {side: usb("b", side) for side in ("left", "right")}
    captured = []
    monkeypatch.setattr(teleop, "check_ports", lambda *a, **kw: None)
    monkeypatch.setattr(teleop, "ros_environment", lambda: {})
    monkeypatch.setattr(teleop.os, "chdir", lambda *a: None)
    monkeypatch.setattr(Path, "is_file", lambda path: True)
    monkeypatch.setattr(
        teleop.subprocess,
        "run",
        lambda cmd, **kw: subprocess.CompletedProcess(
            cmd, 1 if cmd[0] == "pgrep" else 0
        ),
    )
    monkeypatch.setattr(
        teleop.os, "execvpe", lambda exe, cmd, env: captured.append(cmd)
    )
    assert teleop.main(["start", "--robot", "thor", "--device", "b", "--skip-precheck"]) == 0
    for side, config in configs.items():
        assert f"{side}_config:={config}" in captured[0]
```

- [ ] **Step 2: Run tests — expect FAIL**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge && python3 -m pytest scripts/tests/test_teleop_entrypoint.py -v
```

Expected: FAIL on missing `device_for_robot` and/or old auto cross-pairing behavior.

- [ ] **Step 3: Implement**

In `scripts/teleop.py`:

```python
def device_for_robot(profile: str) -> str:
    key = profile.strip().lower()
    if key == "thor":
        return "a"
    if key == "orin":
        return "b"
    raise RuntimeError(f"unsupported robot profile: {profile}")


def install_executables_present() -> None:
    for package, executable in (
        ("skye_robot_driver", "skye_robot_driver"),
        ("skye_factr_teleop", "skye_factr_teleop_driver"),
    ):
        path = WS / "install" / package / "lib" / package / executable
        if not path.is_file():
            raise RuntimeError(
                f"缺少 {executable}，请先执行: bash scripts/teleop.sh build"
            )
```

In `parser()`, add to `start` only:

```python
command.add_argument(
    "--skip-precheck",
    action="store_true",
    help="跳过 ping/串口/latency/install 全量检查（仍拒绝已有 driver）",
)
```

In `main()` after resolving `profile` / `domain`:

```python
device = args.device
if device == "auto":
    device = device_for_robot(profile)
leaders = select_leaders(args.side, device)
```

For `check` and non-skip `start`: call `install_executables_present()` instead of `ros2 pkg prefix` loops; keep ping + `check_ports`.

For `start` with `--skip-precheck`: skip ping, `check_ports`, and `install_executables_present`; still run existing-driver `pgrep` guard; then `execvpe`.

- [ ] **Step 4: Run tests — expect PASS**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge && python3 -m pytest scripts/tests/test_teleop_entrypoint.py -v
```

- [ ] **Step 5: Commit**

```bash
git add scripts/teleop.py scripts/tests/test_teleop_entrypoint.py
git commit -m "$(cat <<'EOF'
feat(teleop): pair robot to leader config and add --skip-precheck

Default thor/orin to matching YAML dirs; skip redundant host checks when UI already ran check.
EOF
)"
```

---

### Task 2: `teleop_record` playbook → `teleop` + `recorder`

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/playbooks.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_process_step.py`

**Interfaces:**
- Produces: `playbook_for(UiMode.teleop_record, repo, profile, cfg)` → steps `teleop`, `recorder`
- Produces: teleop argv `["bash", f"{repo}/scripts/teleop.sh", "start", "--robot", profile, "--skip-precheck"]`
- Produces: env with `ROBOT_PROFILE`, `ROS_DOMAIN_ID` (`20`/`21`), `ROBOT_IP`; **no** required `FASTRTPS_DEFAULT_PROFILES_FILE`
- Consumes: unchanged `dagger` playbook helpers

- [ ] **Step 1: Rewrite failing playbook assertions**

Replace `test_playbook_for_teleop_record` and remove/replace `test_playbook_marvin_start_cmd_override` (marvin override no longer applies to teleop_record):

```python
def test_playbook_for_teleop_record():
    repo = "/data/repo"
    cfg = {"step_timeout_s": 90.0}
    steps = playbook_for(UiMode.teleop_record, repo, "thor", cfg)
    assert [s["id"] for s in steps] == ["teleop", "recorder"]
    assert steps[0]["argv"] == [
        "bash",
        f"{repo}/scripts/teleop.sh",
        "start",
        "--robot",
        "thor",
        "--skip-precheck",
    ]
    assert steps[0]["health_key"] == "teleop"
    assert steps[0]["env"]["ROBOT_PROFILE"] == "thor"
    assert steps[0]["env"]["ROS_DOMAIN_ID"] == "20"
    assert steps[0]["env"]["ROBOT_IP"] == "6.6.7.191"
    assert "FASTRTPS_DEFAULT_PROFILES_FILE" not in steps[0]["env"]
    assert steps[1]["id"] == "recorder"
    assert steps[1]["health_key"] == "recorder"


def test_playbook_teleop_record_orin_domain():
    steps = playbook_for(UiMode.teleop_record, "/repo", "orin", {})
    assert steps[0]["env"]["ROS_DOMAIN_ID"] == "21"
    assert steps[0]["env"]["ROBOT_IP"] == "6.6.7.190"
```

Keep `test_playbook_for_dagger` unchanged (still Docker).

- [ ] **Step 2: Run — expect FAIL**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws && python3 -m pytest src/skye_operator_ui/test/test_process_step.py -v
```

- [ ] **Step 3: Implement `_teleop_playbook`**

```python
_DOMAIN = {"thor": "20", "orin": "21"}

def _base_env(profile: str, repo_root: str, cfg: dict[str, Any]) -> dict[str, str]:
    key = profile.strip().lower()
    env = {
        "ROBOT_PROFILE": key,
        "ROBOT_IP": _CONTROLLER_IP.get(key, _CONTROLLER_IP["thor"]),
        "ROS_DOMAIN_ID": str(
            cfg.get("ros_domain_id", _DOMAIN.get(key, "21"))
        ),
        "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
    }
    # Do not set FASTRTPS_DEFAULT_PROFILES_FILE for host teleop.
    # Dagger path may still set it in _dagger_playbook if needed.
    return env


def _teleop_playbook(repo_root, profile, cfg):
    timeout_s = _step_timeout(cfg)
    env = _base_env(profile, repo_root, cfg)
    return [
        _make_step(
            "teleop",
            [
                "bash",
                os.path.join(repo_root, "scripts", "teleop.sh"),
                "start",
                "--robot",
                profile.strip().lower(),
                "--skip-precheck",
            ],
            env,
            "teleop",
            timeout_s,
        ),
        _make_step(
            "recorder",
            _recorder_argv(repo_root),
            env,
            "recorder",
            timeout_s,
        ),
    ]
```

Leave `_dagger_playbook` using previous marvin/driver env (including FastDDS xml if still required there). Prefer copying the old `_base_env` FastDDS logic into `_dagger_playbook` only so teleop_record stays clean.

- [ ] **Step 4: Run — expect PASS**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws && python3 -m pytest src/skye_operator_ui/test/test_process_step.py -v
```

- [ ] **Step 5: Commit**

```bash
git add skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/playbooks.py \
  skye_ros2_ws/src/skye_operator_ui/test/test_process_step.py
git commit -m "$(cat <<'EOF'
feat(operator-ui): switch teleop_record playbook to host teleop.sh

Replace Docker driver/marvin/align steps with unified teleop + recorder.
EOF
)"
```

---

### Task 3: `health("teleop")` readiness

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/ros_bridge.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_snapshot.py` (and/or new focused health test)

**Interfaces:**
- Produces: `RosBridge.health("teleop") -> bool` true iff joint_states fresh **and** `_teleop_state is not None`
- Produces: `_health_map()` includes `"teleop"`; keep `"marvin"` as alias returning same as `teleop` **or** remove from map (prefer remove from teleop snapshots; dagger UI may still look at marvin — return `health("teleop")` for key `"marvin"` for one release)

- [ ] **Step 1: Failing test**

```python
def test_health_teleop_requires_joints_and_state():
    bridge = RosBridge.__new__(RosBridge)  # or existing fake helper
    bridge._joint_states_stamp = None
    bridge._teleop_state = None
    assert bridge.health("teleop") is False
    bridge._teleop_state = "IDLE"
    assert bridge.health("teleop") is False
    bridge._joint_states_stamp = time.monotonic()
    assert bridge.health("teleop") is True
```

Use the project’s existing fake/bridge test patterns in `test_ros_bridge.py` / `fakes.py` if `__new__` is awkward — mirror how other health tests stub stamps.

- [ ] **Step 2: Run — expect FAIL**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws && python3 -m pytest src/skye_operator_ui/test/test_ros_bridge.py src/skye_operator_ui/test/test_snapshot.py -v -k teleop
```

- [ ] **Step 3: Implement**

```python
def health(self, key: str) -> bool:
    if key == "driver":
        return self._is_fresh(self._joint_states_stamp, FRESHNESS_S)
    if key in ("teleop", "marvin"):
        return (
            self._teleop_state is not None
            and self._is_fresh(self._joint_states_stamp, FRESHNESS_S)
        )
    # ... existing align/arbiter/recorder/policy ...
```

Update `_health_map` to expose `"teleop"` (and optionally `"marvin": self.health("teleop")` for dagger step rail compatibility).

- [ ] **Step 4: Run — expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(operator-ui): ready when joint_states and teleop/state are live

Host teleop step health no longer waits on a Docker marvin probe alone.
EOF
)"
```

---

### Task 4: Supervisor precheck for `teleop_record`

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/supervisor.py`
- Modify: `skye_ros2_ws/src/skye_operator_ui/config/default.yaml` (cleanup list notes)
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_supervisor.py`

**Interfaces:**
- Produces: for `UiMode.teleop_record`, after residual cleanup, run  
  `["bash", f"{repo}/scripts/teleop.sh", "check", "--robot", profile, "--fix-latency"]`  
  and return stderr on failure
- Produces: teleop_record precheck does **not** require `marvin_ws/fastrtps_no_shm.xml`
- Produces: docker rm cleanup argv skipped when mode is teleop_record
- Consumes: Task 1 `teleop.sh check`

- [ ] **Step 1: Failing tests**

```python
def test_teleop_record_precheck_runs_teleop_check(tmp_path, monkeypatch):
    calls = []
    def run(argv, **kwargs):
        calls.append(argv)
        return subprocess.CompletedProcess(argv, 0, "", "")
    # wire supervisor with teleop_record mode after begin_start
    # assert any call looks like [..., "teleop.sh", "check", "--robot", "orin", "--fix-latency"]
    # assert no requirement that marvin_ws/fastrtps exists


def test_teleop_record_precheck_skips_docker_rm(tmp_path, monkeypatch):
    # cfg cleanup includes docker rm; mode teleop_record
    # assert docker rm not in executed commands
```

Adapt to existing supervisor test helpers (`_make_supervisor`, monkeypatch `subprocess.run`).

- [ ] **Step 2: Run — expect FAIL**

- [ ] **Step 3: Implement `_default_precheck` branches**

Sketch:

```python
mode = self.logic.mode()
profile = self.logic.profile()

# cleanup: filter docker rm when mode == UiMode.teleop_record
# FastDDS xml check: only if mode == UiMode.dagger
# ping: skip for teleop_record (teleop.sh check pings)

if mode == UiMode.teleop_record:
    argv = [
        "bash",
        os.path.join(self.repo_root, "scripts", "teleop.sh"),
        "check",
        "--robot",
        (profile or "thor").strip().lower(),
        "--fix-latency",
    ]
    result = subprocess.run(argv, capture_output=True, text=True, timeout=60.0)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "").strip()
        return False, detail or "teleop.sh check failed"
    return True, "ok"
# else: existing dagger precheck path (ping + xml + cleanup)
```

Also fix `first_is_driver` reuse logic: first step id is now `teleop` for teleop_record — update `keep_driver` skip tokens to include `teleop.sh` / `skye_factr_teleop` as needed.

- [ ] **Step 4: Run — expect PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(operator-ui): precheck teleop_record via teleop.sh check

Drop marvin FastDDS and docker rm from the host teleop session path.
EOF
)"
```

---

### Task 5: Teardown + `hold_current` on stop

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/teardown.py`
- Modify: `skye_ros2_ws/src/skye_operator_ui/config/default.yaml`
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/operator_ui_node.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_teardown.py`

**Interfaces:**
- Produces: `teardown.disable_via_docker` default `false`
- Produces: `run_safe_teardown` skips docker rm when `teardown.skip_docker_rm: true` or when cfg mode flag set — simplest: `default.yaml` sets `teardown.docker_rm: false` for host era; dagger ops can set true later
- Produces: `on_before_stop` calls `hold_current` after `switch_stop` (best-effort)

- [ ] **Step 1: Failing tests**

```python
def test_teardown_skips_docker_rm_when_disabled(tmp_path, monkeypatch):
    cfg = {"teardown": {"docker_rm": False, "disable_leader": False}}
    calls = []
    monkeypatch.setattr(subprocess, "run", lambda *a, **k: calls.append(a[0]) or subprocess.CompletedProcess(a[0], 0))
    run_safe_teardown(str(tmp_path), cfg)
    assert not any(cmd[:3] == ["docker", "rm", "-f"] for cmd in calls)


def test_teardown_host_disable_default(tmp_path, monkeypatch):
    # disable_via_docker false → invokes host python disable script
```

- [ ] **Step 2: Run — expect FAIL**

- [ ] **Step 3: Implement**

`default.yaml`:

```yaml
cleanup_stale_commands:
  - ["pkill", "-f", "skye_robot_driver"]
  - ["pkill", "-f", "skye_factr_teleop"]
  - ["pkill", "-f", "follower_align"]
  - ["pkill", "-f", "host_keyboard_align"]
  - ["pkill", "-f", "skye_data_recorder"]
  # docker rm removed from default; dagger may add via overlay if needed

teardown:
  marvin_container_name: skye_marvin_m6
  disable_leader: true
  disable_via_docker: false
  docker_rm: false
  docker_rm_timeout_s: 20.0
  disable_timeout_s: 60.0
```

`teardown.py`: honor `docker_rm` false; prefer host disable.

`operator_ui_node.on_before_stop`:

```python
bridge.stop_recorder_if_active()
try:
    bridge.dispatch("switch_stop")
except Exception:
    node.get_logger().warning("switch_stop during teardown failed")
try:
    bridge.dispatch("hold_current")
except Exception:
    node.get_logger().warning("hold_current during teardown failed")
bridge.set_session_mode(None)
leader_gate.reset()
```

- [ ] **Step 4: Run teardown + related tests — PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(operator-ui): host-first teardown without docker rm

Prefer host Dynamixel disable and best-effort hold_current on session stop.
EOF
)"
```

---

### Task 6: Host leader arm gate + YAML ports for disable

**Files:**
- Modify: `scripts/disable_leader_dynamixel.py`
- Modify: `scripts/leader_arm_gate.sh`
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/ros_bridge.py` (error strings; pass `ROBOT_PROFILE` in env)
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_disable_leader_side.py`, `test_ros_bridge.py`

**Interfaces:**
- Produces: `disable_leader_dynamixel.py` loads `dynamixel.dynamixel_port` from  
  `skye_factr_teleop/config[/orin]/grav_comp_m6_{side}.yaml` when `ROBOT_PROFILE` set and env ports missing
- Produces: `leader_arm_gate.sh left|right off|on` without requiring a running Marvin container
- Consumes: nodes named `skye_factr_teleop_{side}`

- [ ] **Step 1: Failing tests**

```python
def test_ports_from_profile_yaml(tmp_path, monkeypatch):
    # point repo root; set ROBOT_PROFILE=orin; clear PORT env;
    # assert helper resolves FTAO51EA / FTB8HNOT from yaml (names from files)


def test_run_leader_arm_gate_sets_robot_profile(monkeypatch):
    # dispatch runner env includes ROBOT_PROFILE from session — may require
    # RosBridge to pass profile into run_leader_arm_gate
```

Update docker-centric `test_ros_bridge` tests that assert `MARVIN_CONTAINER_NAME` as the success path — host path should succeed without container.

- [ ] **Step 2: Run — expect FAIL**

- [ ] **Step 3: Implement disable YAML fallback**

```python
def _ports_from_host_yaml(repo: Path, profile: str) -> dict[str, str]:
    pkg = repo / "skye_ros2_ws/src/skye_factr_teleop/config"
    base = pkg / "orin" if profile == "orin" else pkg
    out = {}
    for side in ("left", "right"):
        doc = yaml.safe_load((base / f"grav_comp_m6_{side}.yaml").read_text())
        out[side] = doc["dynamixel"]["dynamixel_port"]
    return out
```

In `main()`, after loading marvin env, if ports still missing and `ROBOT_PROFILE` in `{thor,orin}`, fill from YAML.

Rewrite `leader_arm_gate.sh` outline:

```bash
SIDE=$1; ACTION=$2
PROFILE=${ROBOT_PROFILE:-thor}
NODE="skye_factr_teleop_${SIDE}"
REPO=...
if [[ "$ACTION" == "off" ]]; then
  pkill -f "$NODE" || true
  sleep 1
  python3 "$REPO/scripts/disable_leader_dynamixel.py" --side "$SIDE"
  exit $?
fi
# on: if ros2 node list has /$NODE → exit 0
# else: source setup; ros2 launch skye_factr_teleop start_teleop_m6_${SIDE}.launch.py \
#   robot_profile:=$PROFILE sync_diagnostics:=true &
# wait for node
```

Ensure `run_leader_arm_gate` exports `ROBOT_PROFILE` from `bridge` session profile (add parameter or read `supervisor`/cfg). Minimal approach: store profile on `RosBridge` when session starts (`set_session_profile`) — if missing, check how mode is set today (`set_session_mode`) and extend similarly from `api_app` / supervisor start.

- [ ] **Step 4: Run tests — PASS**

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(teleop): host leader arm gate without Marvin docker

Disable/relaunch skye_factr_teleop sides using profile YAML serial ports.
EOF
)"
```

---

### Task 7: Command gates + hints (drop align requirement)

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/commands.py`
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/hints.py`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_commands.py`, `test_hints.py`

**Interfaces:**
- Produces: `switch_teleop` allowed only when `teleop_state == "SYNCED"` (ignore align_status)
- Produces: hints after SYNCED say open teleop (no “开始对齐”)
- Keep `align_*` ops in whitelist for now but unused by UI (or leave allowed checks intact for dagger-free dead code) — prefer leave backend align ops harmless; UI removes buttons in Task 8

- [ ] **Step 1: Update failing/expected tests**

```python
def test_switch_teleop_requires_synced_only():
    # SYNCED + align IDLE → allowed
    # IDLE → denied
    # SYNCED was previously also OK with ALIGNED alone — remove that path
```

```python
def test_hint_after_synced_prompts_teleop():
    # teleop_state SYNCED → hint mentions 遥操, not 对齐
```

- [ ] **Step 2: Run — expect FAIL on old align-or-synced logic**

- [ ] **Step 3: Implement**

```python
if op == "switch_teleop":
    if teleop_state != "SYNCED":
        return False, "请先完成小臂同步（SYNCED）"
    return True, ""
```

Hints for `teleop_record` when `SYNCED`: `"同步完成，可开启遥操"`.

- [ ] **Step 4: PASS + commit**

```bash
git commit -m "$(cat <<'EOF'
fix(operator-ui): require SYNCED only before teleop

Remove follower-align from the teleop_record operator path.
EOF
)"
```

---

### Task 8: Frontend steps/panel + Operator UI docs

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/web/src/lib/steps.js`
- Modify: `skye_ros2_ws/src/skye_operator_ui/web/src/lib/commands.js`
- Modify: `skye_ros2_ws/src/skye_operator_ui/web/src/components/TeleopPanel.svelte`
- Modify: `docs/Operator_UI使用说明.md`
- Optional: rebuild `web/dist` if the package serves built assets in CI

**Interfaces:**
- Produces: `TELEOP_STEPS` without marvin/align; playbook ids `['teleop','recorder']`
- Produces: TeleopPanel without align buttons/status
- Produces: `commands.js` `switch_teleop` gate matches backend (SYNCED only)

- [ ] **Step 1: Edit `steps.js`**

```javascript
export const TELEOP_STEPS = [
  { id: 'precheck', label: '预检' },
  { id: 'teleop', label: '大小臂 Teleop', logId: 'teleop' },
  { id: 'sync', label: '小臂同步' },
  { id: 'teleop_run', label: '遥操' },
  { id: 'record', label: '数采', logId: 'recorder' },
];

const PLAYBOOK_IDS = {
  teleop_record: ['teleop', 'recorder'],
  dagger: ['driver', 'marvin', 'arbiter', 'dummy_policy'],
};
```

Fix `stepStatus` for runtime teleop step id (`teleop_run` vs playbook `teleop`) so playbook completion and runtime “遥操” don’t collide — use distinct ids as above.

- [ ] **Step 2: Strip align from `TeleopPanel.svelte` and `commands.js`**

Remove align buttons; `switch_teleop` requires `teleopState === 'SYNCED'` only.

- [ ] **Step 3: Update `docs/Operator_UI使用说明.md`**

- Startup still `./scripts/start_operator_ui.sh`
- Session uses host `teleop.sh` (check + start); no Docker small-arm requirement for teleop_record
- Flow: 启动 →（可选关臂）→ 同步 → 遥操 → 录制
- Note: change FTDI via `bind_leader_arms.py` offline; no per-session build

- [ ] **Step 4: Build web if required**

```bash
cd skye_ros2_ws/src/skye_operator_ui/web && npm ci && npm run build
```

- [ ] **Step 5: Commit**

```bash
git commit -m "$(cat <<'EOF'
feat(operator-ui): align frontend steps with host teleop playbook

Drop Marvin/align chrome and document the teleop.sh session path.
EOF
)"
```

---

### Task 9: Regression sweep + smoke checklist

**Files:** none new

- [ ] **Step 1: Run operator_ui + teleop entry tests**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge && python3 -m pytest scripts/tests/test_teleop_entrypoint.py -v
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws && python3 -m pytest src/skye_operator_ui/test -v
```

Expected: PASS (skip ROS-only tests if env lacks rclpy the same way CI does).

- [ ] **Step 2: Manual checklist (document results in PR / Operator_UI使用说明 真机节)**

- [ ] Orin: start session → health READY without waiting full 120s → sync → teleop → stop
- [ ] Thor: same
- [ ] Orin: gate off left → sync right-only → teleop → end session (no leftover `skye_robot_driver`)
- [ ] Confirm no `docker rm` / Marvin container required on teleop_record

- [ ] **Step 3: Final commit only if checklist notes or doc checkboxes updated**

---

## Spec coverage self-check

| Spec section | Task |
|---|---|
| §2 thor/orin pairing, no A/B UI | 1, 8 |
| §3 architecture / READY health | 2, 3, 4 |
| §4 playbook + precheck + teleop.py opts | 1, 2, 4 |
| §5 host leader gate | 6 |
| §6 teardown + hold_current | 5 |
| §7 frontend + hints + commands | 7, 8 |
| §8 errors / no session build | 1, 4 |
| §9 tests + smoke | 9 |
| §11 efficiency (skip-precheck, no pkg prefix, topic ready, no docker rm) | 1, 3, 4, 5 |

## Placeholder / consistency scan

- Health key name: playbook `teleop` ↔ `health("teleop")` ↔ frontend `logId: 'teleop'`.
- Runtime step label id `teleop_run` avoids clashing with playbook step `teleop`.
- DAgger still uses `marvin` health alias via Task 3.
- `--skip-precheck` only on start after UI check (Task 1 + 2 + 4).
