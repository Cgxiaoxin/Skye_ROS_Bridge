# remote_inference_async（Thor DAgger vendor）

HITL DAgger 客户端运行时。入口：上层 `../run_client_dagger.sh` → `./run_robot_dagger.sh`。

**不要**与旧 `run_robot.sh`（直连 `CONTROLLER_IP`）和 `skye_robot_driver` 同时开。

## 目录分类

```text
remote_inference_async/
├── run_robot_dagger.sh     # HITL 入口（推荐）
├── run_robot.sh            # 旧直连 Gento 入口（对照用，勿与 driver 同开）
├── async_robot_client.py   # 主循环：观测 / ZMQ / 融合护栏 / dagger pub
├── robot_control.py        # 关节/回零语义、INITIAL_* 位姿
├── protocol.py / units.py  # 协议与单位
├── action_history.py
├── motion_filter.py        # 速度/加速度限幅、tracking guard
├── ros_camera.py
├── trajectory_recorder.py / trajectory_keys.py
├── runtime_control.py / runtime_ctl.py
├── gento_dm_gripper.py
│
├── dagger_*.py             # HITL 口 A（不写 Gento SDK）
│   chunk / quintic / home / publisher / ros_facade
│
├── tools/                  # 相机 / 探针 / 辅助 shell
└── tests/                  # 单测
```

| 类别 | 文件 | 说明 |
|------|------|------|
| **HITL 入口** | `run_robot_dagger.sh` | Domain 20、`--dagger-mode`、quintic home |
| **DAgger 桥** | `dagger_chunk.py` `dagger_quintic.py` `dagger_home.py` `dagger_publisher.py` `dagger_ros_facade.py` | 平滑后发 `/skye/policy_action` |
| **核心运行** | `async_robot_client.py` `robot_control.py` `protocol.py` … | 与旧 UniJEPA client 同源 |
| **工具** | `tools/` | `ensure_cameras.sh`、probe、runtime 控制 |
| **测试** | `tests/` | `pytest tests/ -v`（在 `vendor/` 下） |
| **勿提交** | `*.bak*` `__pycache__/` | 手工备份 / 字节码，已 gitignore |

## 跑在哪台机器？

| 组件 | 机器 | 要本目录吗？ |
|------|------|----------------|
| `./run_client_dagger.sh` | **Thor（或跑 client 的那台）** | **要** |
| arbiter / driver / teleop | PC 或 Thor | 否（ROS 包） |
| `serve_unijepa_dagger.sh` | **5090** | **不要**；5090 只要 `server5090/deploy/` |

本目录是 **client**，改完在 Thor 上直接跑即可；**不必**同步到 5090。若 Thor 与本机不是同一份盘，把 `client_thor/`（含 vendor）同步到 Thor。

## 单测

```bash
cd docs/dagger_dev/client_thor/vendor
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest remote_inference_async/tests -v
```
