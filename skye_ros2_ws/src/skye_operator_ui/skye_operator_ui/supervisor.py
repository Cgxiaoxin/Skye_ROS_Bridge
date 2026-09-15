"""Session orchestration: precheck, playbook steps, watchdog."""

from __future__ import annotations

import logging
import os
import subprocess
import time
from collections.abc import Callable

from skye_operator_ui.playbooks import playbook_for
from skye_operator_ui.process_step import ProcessStep
from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode

logger = logging.getLogger(__name__)


class SessionSupervisor:
    """Drive SessionLogic through precheck, playbook steps, and teardown."""

    def __init__(
        self,
        repo_root: str,
        cfg: dict,
        health_fn: Callable[[str], bool],
        *,
        precheck_fn: Callable[[], tuple[bool, str]] | None = None,
        on_before_stop: Callable[[], None] | None = None,
        on_degraded: Callable[[], None] | None = None,
    ) -> None:
        self.repo_root = repo_root
        self.cfg = cfg
        self.health_fn = health_fn
        self.precheck_fn = precheck_fn or self._default_precheck
        self.on_before_stop = on_before_stop
        self.on_degraded = on_degraded
        self.logic = SessionLogic()
        self._step_defs: list[dict] = []
        self._steps: list[ProcessStep] = []
        self._current_index = 0
        self._step_deadline: float | None = None
        self._last_fail_msg = ""

    def start(self, profile: str, mode: UiMode) -> tuple[bool, str]:
        if not self.logic.begin_start(profile, mode):
            return False, "无法启动：当前状态不允许开始会话"

        ok, msg = self.precheck_fn()
        if not ok:
            self.logic.precheck_fail()
            self._last_fail_msg = msg
            return False, msg

        if not self.logic.precheck_ok():
            return False, "预检状态异常"

        log_ring_size = int(self.cfg.get("log_ring_size", 2000))
        self._step_defs = playbook_for(mode, self.repo_root, profile, self.cfg)
        self._steps = [
            ProcessStep(
                step_id=spec["id"],
                argv=list(spec["argv"]),
                env=dict(spec.get("env") or {}),
                health_check=lambda key=spec["health_key"]: self.health_fn(key),
                timeout_s=float(
                    spec.get("timeout_s", self.cfg.get("step_timeout_s", 120.0))
                ),
                log_ring_size=log_ring_size,
            )
            for spec in self._step_defs
        ]
        self._current_index = 0
        self._step_deadline = None
        return True, ""

    def retry_step(self) -> tuple[bool, str]:
        if self.logic.state() != SessionState.FAILED:
            return False, "当前状态不可重试"
        if not self._steps or self._current_index >= len(self._steps):
            return False, "无可重试步骤"
        if not self.logic.resume_starting():
            return False, "无法恢复启动状态"

        self._step_deadline = None
        return True, ""

    def stop(self) -> tuple[bool, str]:
        if not self.logic.begin_stop():
            if self.logic.state() == SessionState.IDLE:
                return False, "当前无活跃会话"
            return False, "无法结束会话"

        if self.on_before_stop is not None:
            try:
                self.on_before_stop()
            except Exception:  # noqa: BLE001 - teardown continues regardless
                logger.exception("on_before_stop failed")

        for step in reversed(self._steps):
            step.terminate()

        self._step_defs = []
        self._steps = []
        self._current_index = 0
        self._step_deadline = None

        if not self.logic.mark_idle():
            return False, "结束会话状态异常"
        return True, ""

    def tick(self) -> None:
        state = self.logic.state()
        if state == SessionState.STARTING:
            self._tick_starting()
        elif state in (SessionState.READY, SessionState.RUNNING):
            if not self.health_fn("driver") and self.logic.mark_degraded():
                if self.on_degraded is not None:
                    try:
                        self.on_degraded()
                    except Exception:  # noqa: BLE001 - degrade path must not raise
                        logger.exception("on_degraded failed")

    def snapshot_fields(self) -> dict:
        state = self.logic.state()
        step_id = None
        if state == SessionState.STARTING and self._current_index < len(self._step_defs):
            step_id = self._step_defs[self._current_index]["id"]
        mode = self.logic.mode()
        return {
            "state": state.name,
            "profile": self.logic.profile(),
            "mode": mode.name if mode else None,
            "step": step_id,
        }

    def step_logs(self, step_id: str, n: int = 200) -> list[str]:
        for step in self._steps:
            if step.step_id == step_id:
                return step.logs.tail(n)
        return []

    def run_cleanup_stale(self) -> tuple[bool, str]:
        commands = self.cfg.get("cleanup_stale_commands") or []
        if not commands:
            return True, (
                "未配置清理命令：config/default.yaml 的 cleanup_stale_commands 为空，"
                "请按注释示例填写后重试"
            )

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
            if result.returncode != 0:
                cmd = " ".join(str(part) for part in argv)
                return False, f"清理命令失败 ({cmd})"

        return True, "清理完成"

    def _tick_starting(self) -> None:
        if not self._steps:
            self.logic.mark_ready()
            return

        if self._current_index >= len(self._steps):
            self.logic.mark_ready()
            return

        step = self._steps[self._current_index]
        spec = self._step_defs[self._current_index]

        if self._step_deadline is None:
            step.start()
            self._step_deadline = time.monotonic() + step.timeout_s
            return

        if self.health_fn(spec["health_key"]):
            self._current_index += 1
            self._step_deadline = None
            if self._current_index >= len(self._steps):
                self.logic.mark_ready()
            return

        if time.monotonic() >= self._step_deadline:
            if spec.get("optional"):
                self._current_index += 1
                self._step_deadline = None
                if self._current_index >= len(self._steps):
                    self.logic.mark_ready()
            else:
                self.logic.mark_failed()
                self._last_fail_msg = f"步骤 {spec['id']} 健康检查超时"

    def _default_precheck(self) -> tuple[bool, str]:
        precheck_cfg = self.cfg.get("precheck", {})

        if not precheck_cfg.get("skip_ping", False):
            try:
                result = subprocess.run(
                    ["ping", "-c", "1", "-W", "1", "6.6.7.190"],
                    capture_output=True,
                    text=True,
                    timeout=5.0,
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                return False, f"网络预检失败: {exc}"
            if result.returncode != 0:
                return False, "无法 ping 通 Thor (6.6.7.190)"

        xml_path = os.path.join(
            self.repo_root, "marvin_ws", "fastrtps_no_shm.xml"
        )
        if not os.path.isfile(xml_path):
            return False, f"缺少 FastDDS 配置: {xml_path}"

        count = 0
        try:
            result = subprocess.run(
                ["pgrep", "-fc", "skye_robot_driver"],
                capture_output=True,
                text=True,
                timeout=5.0,
            )
            if result.returncode == 0:
                count = int(result.stdout.strip())
        except (ValueError, OSError, subprocess.TimeoutExpired):
            count = 0

        if count > 0:
            profile = self.logic.profile()
            mode = self.logic.mode()
            allow = bool(self.cfg.get("allow_existing_driver", False))
            first_is_driver = False
            if profile is not None and mode is not None:
                steps = playbook_for(mode, self.repo_root, profile, self.cfg)
                first_is_driver = bool(steps) and steps[0].get("id") == "driver"
            if not (allow and first_is_driver):
                return False, "检测到残留 skye_robot_driver，请先执行 cleanup"

        return True, "ok"
