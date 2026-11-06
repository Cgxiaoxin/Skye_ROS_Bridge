# UniJEPA × HITL DAgger 实机验证流程（Thor）

配套：[`hitl_dagger_thor_现场联调步骤.md`](hitl_dagger_thor_现场联调步骤.md)、[`unijepa_dagger_接入说明.md`](unijepa_dagger_接入说明.md)。

**禁止：** 旧 `run_robot.sh`（直连 `CONTROLLER_IP`）与 `skye_robot_driver` 同时开。

---

## A. 环境（每个终端）

```bash
export ROBOT_PROFILE=thor
export ROS_DOMAIN_ID=20
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /data/coding/tianji/Skye_ROS_Bridge
source skye_ros2_ws/install/setup.bash
```

---

## B. 启停顺序

| 序 | 终端 / 机 | 命令 | 验收 |
|----|-----------|------|------|
| 1 | Thor A | `./scripts/start_skye_for_factr.sh thor` | Domain=20，链上 `6.6.7.191` |
| 2 | Thor B | `ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py robot_profile:=thor` | 左右 `IDLE` |
| 3 | Thor C | `ros2 launch skye_hitl_dagger hitl_dagger.launch.py robot_profile:=thor` | arbiter 起；`/gento/*_abs` 仅 arbiter 写 |
| 4 | Thor D | `cd docs/dagger_dev/client_thor && bash ensure_cameras.sh` | 三路相机 topic 有图 |
| 5 | 5090 | `bash docs/dagger_dev/server5090/serve_unijepa_dagger.sh`（或现场 `deploy/serve_b_levjepa.sh`，口 **5568**） | 监听 `tcp://*:5568` |
| 6 | Thor E | `cd docs/dagger_dev/client_thor && bash run_client_dagger.sh` | 日志含 `DAGGER mode` / `published PolicyActionChunk`；**无** Gento link |

收工：E→D→C→B→A（先 `return` 再停小臂，见 HITL 文档 §3.4）。

---

## C. 分阶段验收

### C0 — 图测（无策略）

```bash
ros2 topic echo /teleop/state --once
ros2 topic info /gento/left_joint_control_abs   # Publisher=arbiter
ros2 topic hz /gento/joint_states
```

### C1 — HITL 接管链路（可不启 5090 / client）

```bash
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'takeover'}"
# 两侧 SYNCED 后
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'enter_teleop'}"
# 轻动小臂 → 大臂跟随
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'return'}"
```

期望：`HANDOVER_SYNC` → `HUMAN` → `AUTONOMOUS`；同步期间大臂**无**明显微抖。

### C2 — 口 A 冒烟（假策略，可选）

```bash
ros2 run skye_hitl_dagger pub_dummy_policy_chunk
ros2 topic echo /skye/policy_action --once
# control_mode=AUTONOMOUS 时大臂应跟 dummy hold/轨迹
```

### C3 — UniJEPA 闭环（本轮重点）

1. 起 5090 + `run_client_dagger.sh`  
2. 确认：
   - client：`DAGGER published PolicyActionChunk ...`
   - `ros2 topic hz /skye/policy_action` 有输出  
   - `ros2 topic info /gento/left_joint_control_abs` 仍只有 arbiter  
3. `AUTONOMOUS` 下观察大臂是否跟模型 chunk（chunk_size=16，dt≈1/30）  
4. 干预：`takeover` → `enter_teleop` → 人手纠偏 → `return` 交还策略

### C4 — 回归禁区

| 检查 | 期望 |
|------|------|
| UDP 3720–3724 被 driver 占用时 DAgger client 仍能起 | 跳过 SDK link，不报端口冲突退出 |
| 未再起第二路 `skye_robot_driver` / 旧 `run_robot.sh` | 仅 arbiter 写臂 |
| Domain 全链路 20 | 无「半通」 |

---

## D. 现场常改环境变量

| 变量 | 默认 | 说明 |
|------|------|------|
| `INFERENCE_HOST` / `INFERENCE_PORT` | `192.168.201.253` / `5568` | 5090 |
| `SKYE_WS` | 本仓 `skye_ros2_ws` | 需已 install（`PolicyActionChunk`） |
| `CAMERA_ROS_SETUP` | 现场 RealSense ws | 与 `ensure_cameras.sh` 一致 |
| `SKIP_CAMERAS=1` | — | 相机已由别处拉起时 |
| `JOINT_STATES_TOPIC` | `/gento/joint_states` | 观测关节源 |
