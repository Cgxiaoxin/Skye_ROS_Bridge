#!/usr/bin/env bash
# Operator UI API verify (no robot; no running operator_ui node required).
# Requires built skye_operator_ui (skye_ros2_ws/install/setup.bash + web dist in share).
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "$WS"

if [[ ! -f install/setup.bash ]]; then
  echo "FAIL: install/setup.bash missing; run skye_ros2_ws/scripts/build.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

pick_python() {
  local candidate
  for candidate in "${PYTHON:-}" python3 /usr/bin/python3.10; do
    [[ -n "${candidate}" ]] || continue
    if "${candidate}" -c "import fastapi" >/dev/null 2>&1; then
      command -v "${candidate}"
      return 0
    fi
  done
  echo "FAIL: fastapi not importable; install skye_operator_ui requirements:" >&2
  echo "  pip install -r skye_ros2_ws/src/skye_operator_ui/requirements.txt" >&2
  exit 1
}

PYTHON="$(pick_python)"
export PATH="/usr/bin:/bin:${PATH}"

echo "== operator_ui: installed package + web assets =="
"$PYTHON" <<'PY'
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

share = Path(get_package_share_directory("skye_operator_ui"))
config = share / "config" / "default.yaml"
web_index = share / "web" / "index.html"
if not config.is_file():
    raise SystemExit(f"FAIL: missing {config}")
if not web_index.is_file():
    raise SystemExit(
        f"FAIL: missing {web_index}; rebuild after `cd web && npm run build`"
    )
print(f"OK: share config + web ({web_index})")
PY

echo "== operator_ui: /api/snapshot via TestClient (no ROS spin) =="
"$PYTHON" <<'PY'
from fastapi.testclient import TestClient

from skye_operator_ui.api_app import create_app
from skye_operator_ui.session_state import SessionLogic, SessionState, UiMode
from skye_operator_ui.snapshot import SnapshotBuilder


class FakeSupervisor:
    def __init__(self) -> None:
        self.logic = SessionLogic()

    def start(self, profile, mode):
        if not self.logic.begin_start(profile, mode):
            return False, "无法启动：当前状态不允许开始会话"
        if not self.logic.precheck_ok():
            return False, "预检状态异常"
        return True, ""

    def stop(self):
        if self.logic.state() == SessionState.IDLE:
            return False, "当前无活跃会话"
        if not self.logic.begin_stop():
            return False, "无法结束会话"
        if not self.logic.mark_idle():
            return False, "结束会话状态异常"
        return True, ""

    def retry_step(self):
        if not self.logic.resume_starting():
            return False, "无法恢复启动状态"
        return True, ""

    def tick(self) -> None:
        pass

    def snapshot_fields(self):
        mode = self.logic.mode()
        return {
            "state": self.logic.state().name,
            "profile": self.logic.profile(),
            "mode": mode.name if mode else None,
            "step": None,
        }

    def run_cleanup_stale(self):
        return True, "清理完成"

    def step_logs(self, step_id, n=200):
        return [f"log:{step_id}"]


class FakeBridge:
    def available(self) -> bool:
        return True

    def set_session_mode(self, mode) -> None:
        pass

    def mailbox(self):
        return {
            "teleop_state": "SYNCED",
            "align_status": "IDLE",
            "hitl_mode": None,
            "hitl_source": None,
            "left_joints": [0.0] * 7,
            "right_joints": [0.0] * 7,
            "left_gripper": 0.0,
            "right_gripper": 0.0,
            "health": {"driver": True},
            "recording_active": False,
        }

    def dispatch(self, op: str):
        return True, ""


app = create_app(FakeSupervisor(), FakeBridge(), SnapshotBuilder())
client = TestClient(app)
response = client.get("/api/snapshot")
if response.status_code != 200:
    raise SystemExit(f"FAIL: /api/snapshot status {response.status_code}")
body = response.json()
if "session" not in body or "next_hint" not in body:
    raise SystemExit(f"FAIL: unexpected snapshot body keys: {list(body)}")
print("OK: GET /api/snapshot -> 200")
PY

echo "PASS: operator UI API verify (package + snapshot contract)"
