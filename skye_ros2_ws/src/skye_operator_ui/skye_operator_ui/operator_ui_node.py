"""ROS2 node entrypoint: rclpy spin thread + FastAPI/uvicorn main thread."""

from __future__ import annotations

import os
import sys
import threading
from pathlib import Path

import rclpy
import yaml
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from skye_operator_ui.api_app import create_app
from skye_operator_ui.ros_bridge import RosBridge
from skye_operator_ui.snapshot import SnapshotBuilder
from skye_operator_ui.supervisor import SessionSupervisor


def _detect_repo_root(start: Path | None = None) -> str:
    markers = (
        "marvin_ws/fastrtps_no_shm.xml",
        "scripts/build.sh",
    )
    for base in (start or Path.cwd(), Path(__file__).resolve().parents[4]):
        current = base.resolve()
        for _ in range(8):
            if any((current / marker).is_file() for marker in markers):
                return str(current)
            if current.parent == current:
                break
            current = current.parent
    return str(Path.cwd().resolve())


def _load_config(node: Node) -> dict:
    cfg_path = node.declare_parameter("config_path", "").value
    if cfg_path:
        path = Path(str(cfg_path))
    else:
        try:
            from ament_index_python.packages import get_package_share_directory

            path = Path(get_package_share_directory("skye_operator_ui")) / "config" / "default.yaml"
        except Exception:
            path = Path(__file__).resolve().parent.parent / "config" / "default.yaml"

    if not path.is_file():
        return {}

    with path.open(encoding="utf-8") as handle:
        data = yaml.safe_load(handle) or {}
    if not isinstance(data, dict):
        return {}
    return data


def main(args=None) -> None:
    rclpy.init(args=args)
    node = rclpy.create_node("operator_ui")

    repo_root_param = node.declare_parameter("repo_root", "").value
    repo_root = str(repo_root_param).strip() or _detect_repo_root()
    cfg = _load_config(node)

    bridge = RosBridge(node)

    def health_fn(key: str) -> bool:
        return bridge.health(key)

    supervisor = SessionSupervisor(
        repo_root=repo_root,
        cfg=cfg,
        health_fn=health_fn,
        on_before_stop=lambda: bridge.set_session_mode(None),
    )
    snapshot_builder = SnapshotBuilder()
    app = create_app(supervisor, bridge, snapshot_builder)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    bind_host = str(cfg.get("bind_host", "127.0.0.1"))
    port = int(cfg.get("port", 8765))

    import uvicorn

    try:
        uvicorn.run(app, host=bind_host, port=port, log_level="info")
    finally:
        supervisor.stop()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main(sys.argv)
