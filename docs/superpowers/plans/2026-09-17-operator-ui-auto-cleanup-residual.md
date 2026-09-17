# Operator UI Auto-Cleanup Residual Driver Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When starting a session, if precheck finds a residual `skye_robot_driver` and reuse is not allowed, automatically run whitelist cleanup, recheck, and continue start on success (fail only if residual remains).

**Architecture:** Extend `SessionSupervisor._default_precheck` so residual detection no longer hard-fails immediately. On residual (and not `allow_existing_driver` + first step is `driver`), call cleanup (cfg `cleanup_stale_commands`, or built-in `pkill -f skye_robot_driver` if empty), log `已自动清理残留 driver`, re-`pgrep`, then pass or fail. Keep `POST /api/session/cleanup_stale` for manual use; no new UI button in this plan.

**Tech Stack:** Python 3, pytest, existing `skye_operator_ui` package.

## Global Constraints

- Only kill via whitelist argv (or the single built-in driver `pkill` fallback when whitelist empty).
- Respect `allow_existing_driver: true` when playbook first step id is `driver` — do not auto-kill.
- `pkill`/`killall` exit code `1` (no matching process) must not be treated as cleanup failure.
- No frontend changes; no new docs beyond this plan unless Operator UI 使用说明 needs one sentence (optional in Task 3).
- Do not commit unless the user asks.

---

### Task 1: Precheck auto-cleanup + pkill exit handling

**Files:**
- Modify: `skye_ros2_ws/src/skye_operator_ui/skye_operator_ui/supervisor.py`
- Modify: `skye_ros2_ws/src/skye_operator_ui/config/default.yaml`
- Test: `skye_ros2_ws/src/skye_operator_ui/test/test_supervisor.py`

**Interfaces:**
- Consumes: existing `run_cleanup_stale() -> tuple[bool, str]`, `playbook_for(...)`, cfg keys `cleanup_stale_commands`, `allow_existing_driver`, `precheck.skip_ping`
- Produces: `_count_skye_robot_driver() -> int`; `_run_cleanup_commands(commands) -> tuple[bool, str]` (shared by `run_cleanup_stale` and auto path); `_default_precheck` auto-cleans then rechecks; default.yaml non-empty `cleanup_stale_commands`

- [ ] **Step 1: Write the failing tests**

Add to `test/test_supervisor.py`:

```python
def test_precheck_auto_cleans_residual_driver(tmp_path, monkeypatch):
    xml = tmp_path / "marvin_ws" / "fastrtps_no_shm.xml"
    xml.parent.mkdir(parents=True)
    xml.write_text("<xml/>")

    calls = {"pgrep": 0, "cleanup": 0}

    def fake_run(argv, **kwargs):
        class R:
            returncode = 0
            stdout = "1"
            stderr = ""

        if argv[:2] == ["pgrep", "-fc"]:
            calls["pgrep"] += 1
            r = R()
            # First count finds residual; after cleanup, gone.
            r.stdout = "1" if calls["pgrep"] == 1 else "0"
            r.returncode = 0 if calls["pgrep"] == 1 else 1
            return r
        if argv[:2] == ["pkill", "-f"]:
            calls["cleanup"] += 1
            return R()
        raise AssertionError(f"unexpected argv: {argv}")

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)

    cfg = {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": [["pkill", "-f", "skye_robot_driver"]],
        "precheck": {"skip_ping": True},
        "playbook_override": {
            "teleop_record": [
                {
                    "id": "driver",
                    "argv": ["/bin/true"],
                    "health_key": "driver",
                    "timeout_s": 3.0,
                    "optional": False,
                }
            ]
        },
    }
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=lambda _k: False,
    )
    ok, reason = sup.start("thor", UiMode.teleop_record)
    assert ok, reason
    assert calls["cleanup"] == 1
    assert calls["pgrep"] >= 2


def test_precheck_fails_if_residual_remains_after_cleanup(tmp_path, monkeypatch):
    xml = tmp_path / "marvin_ws" / "fastrtps_no_shm.xml"
    xml.parent.mkdir(parents=True)
    xml.write_text("<xml/>")

    def fake_run(argv, **kwargs):
        class R:
            returncode = 0
            stdout = "1"
            stderr = ""

        if argv[:2] == ["pgrep", "-fc"]:
            return R()  # always residual
        if argv[:2] == ["pkill", "-f"]:
            return R()
        raise AssertionError(f"unexpected argv: {argv}")

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)

    cfg = {
        "step_timeout_s": 5.0,
        "log_ring_size": 100,
        "cleanup_stale_commands": [["pkill", "-f", "skye_robot_driver"]],
        "precheck": {"skip_ping": True},
        "playbook_override": {
            "teleop_record": [
                {
                    "id": "driver",
                    "argv": ["/bin/true"],
                    "health_key": "driver",
                    "timeout_s": 3.0,
                    "optional": False,
                }
            ]
        },
    }
    sup = SessionSupervisor(
        repo_root=str(tmp_path),
        cfg=cfg,
        health_fn=lambda _k: False,
    )
    ok, reason = sup.start("thor", UiMode.teleop_record)
    assert not ok
    assert "残留" in reason


def test_cleanup_stale_pkill_exit_1_is_ok(tmp_path, monkeypatch):
    def fake_run(argv, **kwargs):
        class R:
            returncode = 1
            stdout = ""
            stderr = ""

        return R()

    monkeypatch.setattr("skye_operator_ui.supervisor.subprocess.run", fake_run)
    sup = _make_supervisor(
        tmp_path,
        {
            **_mock_cfg(),
            "cleanup_stale_commands": [["pkill", "-f", "skye_robot_driver"]],
        },
        {},
    )
    ok, reason = sup.run_cleanup_stale()
    assert ok, reason
```

- [ ] **Step 2: Run tests to verify they fail**

Run:

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws && \
  python3 -m pytest src/skye_operator_ui/test/test_supervisor.py::test_precheck_auto_cleans_residual_driver \
  src/skye_operator_ui/test/test_supervisor.py::test_precheck_fails_if_residual_remains_after_cleanup \
  src/skye_operator_ui/test/test_supervisor.py::test_cleanup_stale_pkill_exit_1_is_ok -v
```

Expected: FAIL (auto-clean not implemented / pkill 1 treated as failure).

- [ ] **Step 3: Implement**

In `supervisor.py`:

1. Add helpers:

```python
_DEFAULT_DRIVER_CLEANUP = [["pkill", "-f", "skye_robot_driver"]]

def _count_skye_robot_driver(self) -> int:
    try:
        result = subprocess.run(
            ["pgrep", "-fc", "skye_robot_driver"],
            capture_output=True,
            text=True,
            timeout=5.0,
        )
        if result.returncode == 0:
            return int(result.stdout.strip())
    except (ValueError, OSError, subprocess.TimeoutExpired):
        return 0
    return 0

@staticmethod
def _cleanup_argv_ok(argv: list, returncode: int) -> bool:
    if argv and argv[0] in ("pkill", "killall") and returncode in (0, 1):
        return True
    return returncode == 0

def _run_cleanup_commands(self, commands: list) -> tuple[bool, str]:
    for argv in commands:
        if not isinstance(argv, list) or not argv:
            return False, "清理命令格式无效"
        try:
            result = subprocess.run(
                argv,
                capture_output=True,
                text=True,
                timeout=120.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            return False, f"清理失败: {exc}"
        if not self._cleanup_argv_ok(argv, result.returncode):
            cmd = " ".join(str(part) for part in argv)
            return False, f"清理命令失败 ({cmd})"
    return True, "清理完成"
```

2. Change `run_cleanup_stale` to use `_run_cleanup_commands` (empty list behavior unchanged).

3. In `_default_precheck`, replace hard-fail residual block with:

```python
count = self._count_skye_robot_driver()
if count > 0:
    profile = self.logic.profile()
    mode = self.logic.mode()
    allow = bool(self.cfg.get("allow_existing_driver", False))
    first_is_driver = False
    if profile is not None and mode is not None:
        steps = playbook_for(mode, self.repo_root, profile, self.cfg)
        first_is_driver = bool(steps) and steps[0].get("id") == "driver"
    if not (allow and first_is_driver):
        commands = self.cfg.get("cleanup_stale_commands") or list(
            self._DEFAULT_DRIVER_CLEANUP
        )
        ok, reason = self._run_cleanup_commands(commands)
        if not ok:
            return False, f"残留 skye_robot_driver 自动清理失败: {reason}"
        time.sleep(0.3)
        if self._count_skye_robot_driver() > 0:
            return False, "检测到残留 skye_robot_driver，自动清理后仍存在"
        logger.warning("已自动清理残留 driver")
```

4. In `config/default.yaml`, set:

```yaml
cleanup_stale_commands:
  - ["pkill", "-f", "skye_robot_driver"]
  - ["pkill", "-f", "factr_teleop"]
  - ["docker", "rm", "-f", "skye_marvin_m6"]
```

Note: `docker rm -f` may return non-zero if container absent — either keep only the two `pkill` lines in default for safety, or extend `_cleanup_argv_ok` to treat `docker rm` exit 1 as ok. Prefer extending:

```python
if argv[:2] == ["docker", "rm"] and returncode in (0, 1):
    return True
```

- [ ] **Step 4: Run tests to verify they pass**

```bash
cd /data/coding/tianji/Skye_ROS_Bridge/skye_ros2_ws && \
  python3 -m pytest src/skye_operator_ui/test/test_supervisor.py -v
```

Expected: all PASS.

- [ ] **Step 5: Commit only if user asks**

Do not commit in this task unless explicitly requested.

---

### Task 2: Optional doc one-liner

**Files:**
- Modify: `docs/Operator_UI使用说明.md` (会话启停 table / 开始会话 row)

- [ ] **Step 1:** Update「开始会话」描述 to mention: 预检若发现残留 `skye_robot_driver` 且未开启复用，会自动执行 `cleanup_stale_commands` 后复检。

- [ ] **Step 2:** No commit unless user asks.

---

## Spec coverage

- Auto-clean on residual → Task 1
- Log `已自动清理残留 driver` → Task 1
- Recheck then fail if still present → Task 1
- Respect `allow_existing_driver` → Task 1
- Fill default cleanup whitelist → Task 1
- Manual UI button → out of scope (API already exists)

## Self-review

- No placeholders left.
- `pkill` exit 1 handled so default whitelist is usable.
- Built-in fallback covers empty cfg during tests / old installs.
