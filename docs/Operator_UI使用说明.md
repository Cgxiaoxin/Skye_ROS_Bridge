# Skye Operator UI 使用说明

## 真机状态

**真机冒烟尚未完成**（Task 10，需现场操作员在 Thor/Orin 上勾选）：

- [ ] Thor 遥操数采全路径（启动 → sync → 对齐 → 遥操 → 录约 30s → 停录 → 结束会话，无 SDK 残留）
- [ ] Thor DAgger takeover/return（AUTONOMOUS → takeover → HUMAN → return → 结束会话）
- [ ] 急停不拆会话；结束会话无 SDK 残留
- [ ] 常开 UI 约 1h，观察 driver `/gento/joint_states` hz 无明显下降
- [ ] Orin 至少遥操路径一次（profile 锁）

完成后请在本节勾选并补记日期/操作员。

---

## 概述

Operator UI 是本机浏览器控制台，封装现有 `scripts/*.sh` 与会话状态机，支持：

- **遥操数采**（`teleop_record`）：driver → Docker 小臂 → follower_align → `skye_data_recorder`
- **HITL DAgger**（`dagger`）：driver → Docker HITL launch → `start_hitl_host.sh`（arbiter）

设计细节见 `[docs/superpowers/specs/2026-09-15-skye-operator-ui-design.md](superpowers/specs/2026-09-15-skye-operator-ui-design.md)`。

## 启动

```bash
# 仓库根目录
./scripts/start_operator_ui.sh
```

脚本会加载 `scripts/lib/ros_domain_env.sh`（默认 `ROS_DOMAIN_ID=21`、FastDDS 无 SHM），并 `ros2 run skye_operator_ui operator_ui`，传入 `repo_root` 参数。

若 `skye_ros2_ws/install` 不存在，先构建：

```bash
./skye_ros2_ws/scripts/build.sh
```

前端静态资源需在 `web/dist` 构建后随包安装（`cd skye_ros2_ws/src/skye_operator_ui/web && npm ci && npm run build`，再 colcon build）。

## 浏览器访问

默认监听 `http://127.0.0.1:8765`（`skye_operator_ui/config/default.yaml` 中 `bind_host` / `port`，以 yaml 为准）。

建议用应用窗口减少误关：

```bash
google-chrome --app=http://127.0.0.1:8765
# 或 Microsoft Edge --app=...
```

顶栏选择 **profile**（thor / orin）与 **模式**（遥操数采 / DAgger），仅在 **IDLE** 可改；点「开始会话」后 profile 与模式锁定至「结束会话」。

## 会话启停 vs 急停


| 操作                        | 作用                                                                                                                   |
| ------------------------- | -------------------------------------------------------------------------------------------------------------------- |
| **开始会话**                  | 预检 → 按 playbook 逐步拉起子进程（driver、Docker 小臂、recorder/arbiter 等）                                                         |
| **结束会话**                  | 有序停止子进程组，**强制** `docker rm -f skye_marvin_m6`，并默认跑主臂 Dynamixel 去使能；回到 IDLE。**结束前请托住小臂**（去使能后会下落）。改 profile 或模式前必须先结束 |
| **急停** (`emergency_stop`) | 调用 `/gento/emergency_stop`，**不结束会话**；会话仍在 READY/RUNNING，可继续操作或再点结束会话                                                 |


急停不受「尚未就绪」等向导门禁拦截。结束会话退场顺序：停录 → `switch_stop` → 倒序 SIGTERM 进程组 → `docker rm -f skye_marvin_m6` → 用 Marvin 镜像一次性容器跑 `scripts/disable_leader_dynamixel.py`（主机无 `dynamixel_sdk` 时也能去使能）。可用 `teardown.disable_leader: false` 关闭去使能。

历史上未命名的残留容器（例如随机名）不会被自动清掉，需一次性：`docker ps` 后 `docker rm -f <id>`。

## 退出机制

```
pkill -f 'skye_operator_ui|operator_ui' || true
# 仍不行再：
kill -9 2400740
```

## Docker 小臂步骤（已知限制）

v1 默认 `scripts/run_marvin_m6_impedance.sh` 使用 **交互式** `docker run -it`，Operator UI 无法在无人工确认的情况下自动完成容器内 launch。

可选绕过方式（任选其一）：

1. 在 `config/default.yaml` 或运行时参数中设置 `playbook.marvin_start_cmd` 为非交互包装脚本（argv 列表）
2. 设置环境变量 `MARVIN_LAUNCH_CMD` 为容器内非交互 launch，例如：
  - 遥操：`ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento.launch.py use_keyboard:=false`
  - HITL：`ros2 launch /marvin_ws/launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py use_keyboard:=false`
3. 在 `playbook.marvin_launch_cmd` 中写入同上 launch 行

详见 `[docs/Hint_Dagger启动使用说明.md](Hint_Dagger启动使用说明.md)` 与 spec §3.4。

## 无真机验收

```bash
./skye_ros2_ws/scripts/verify_operator_ui_api.sh
```

检查包已安装、`web/index.html` 在 share 目录，并用 FastAPI `TestClient` 断言 `GET /api/snapshot` 返回 200（无需启动 `operator_ui` 节点或连接机械臂）。

单元测试：

```bash
cd skye_ros2_ws && colcon test --packages-select skye_operator_ui && colcon test-result
```



## 相关文档

- 设计 spec：`[docs/superpowers/specs/2026-09-15-skye-operator-ui-design.md](superpowers/specs/2026-09-15-skye-operator-ui-design.md)`
- ROS 接口索引：`[docs/ros_interfaces.md](ros_interfaces.md)`（文末 Operator UI 小节）
- Thor/Orin 遥操：`[docs/Thor_Orin_遥操启动.md](Thor_Orin_遥操启动.md)`
- HITL DAgger：`[docs/Hint_Dagger启动使用说明.md](Hint_Dagger启动使用说明.md)`

