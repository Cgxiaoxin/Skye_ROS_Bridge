# UniJEPA × HITL DAgger 实机验证流程

配套：[`hitl_dagger_thor_现场联调步骤.md`](hitl_dagger_thor_现场联调步骤.md)、[`unijepa_dagger_接入说明.md`](unijepa_dagger_接入说明.md)、[`phase1_hitl_isomorphic_startup.md`](phase1_hitl_isomorphic_startup.md)。

---

## 0. 三台机器分别干什么（先看这里）

```text
┌─────────────────────┐     git / scp      ┌──────────────────────────┐
│ 你的本地电脑          │ ───────────────► │ Thor 开发机（机器人侧）     │
│ 改代码、写文档、提交   │                   │ driver / 相机 / DAgger     │
│ 一般不跑 ensure_cameras│                  │ client / 通常也跑 arbiter  │
└─────────────────────┘                   └────────────┬─────────────┘
                                                       │ ZMQ :5568
                                                       ▼
                                              ┌────────────────┐
                                              │ 5090 推理机     │
                                              │ UniJEPA serve  │
                                              └────────────────┘

可选：笔记本 USB 插小臂 → 只跑 HITL teleop（与 Thor 同 ROS_DOMAIN_ID）
```

| 机器 | 是什么 | 本流程里做什么 |
|------|--------|----------------|
| **本地电脑**（你现在这台 `vitai`） | 开发仓 | 改代码、`git commit`；**不要**跑 `ensure_cameras.sh` / driver（没有 `/opt/kernelmind/...`、也连不上 `6.6.7.191`） |
| **Thor** | 机器人侧开发机 | **步骤 1、3、4、6 必须在这里**；正式布局下也常跑 arbiter |
| **笔记本**（可选） | USB 小臂 | **仅步骤 2** 可挪到这里；Domain 必须与 Thor 同为 **20** |
| **5090** | 推理 GPU 机 | **仅步骤 5** |

你终端里这句报错：

```text
[ERROR] ROS setup does not exist: /opt/kernelmind/apex/install/setup.bash
```

说明脚本是按 **Thor 现场路径**写的，在本地跑会失败——应 **SSH 到 Thor** 再执行。

**禁止：** 旧 `run_robot.sh`（直连 `CONTROLLER_IP`）与 `skye_robot_driver` 同时开。

---

## 1. 开跑前：代码放哪、要不要上传

### 1.1 仓库本身（Skye_ROS_Bridge）

| 内容 | 本地 | Thor | 5090 |
|------|------|------|------|
| `skye_ros2_ws`（driver / teleop / hitl_dagger） | 改代码 | **要有同分支 + `colcon build` + `source install`** | 一般不需要 |
| `docs/dagger_dev/unijepa_*`、联调文档 | 改文档 | 可选（方便对着跑） | 可选 |
| `docs/dagger_dev/client_thor/*.sh`、`run_client_dagger.sh` | 改脚本 | **要同步到 Thor** | 否 |
| `docs/dagger_dev/client_thor/vendor/remote_inference_async/` | 本地可有（gitignore，不 push） | **Thor 上必须有**（解压 `async.tar.gz` 或 scp） | 否 |
| `docs/dagger_dev/server5090/deploy/` | 本地可有（gitignore） | 否 | **5090 上必须有** + 权重 ckpt |

同步方式任选：

```bash
# 推荐：Thor / 5090 上 git pull 同分支（vendor/deploy 不会进 git）
git pull

# vendor（Thor，git 不管）：从本地 scp 或现场解压
scp -r docs/dagger_dev/client_thor/vendor/remote_inference_async user@<thor>:<repo>/docs/dagger_dev/client_thor/vendor/

# deploy（5090）
scp -r docs/dagger_dev/server5090/deploy user@<5090>:<path>/
# 权重仍用现场 LEVJEPA_CKPT，本仓不含大模型文件
```

### 1.2 各机「开跑最少集合」

**Thor 上要有：**

1. 本仓已编译：`skye_robot_driver` / `skye_factr_teleop` / `skye_hitl_dagger`
2. `docs/dagger_dev/client_thor/run_client_dagger.sh` + `ensure_cameras.sh`
3. `docs/dagger_dev/client_thor/vendor/remote_inference_async/`（含改过的 `--dagger-mode`）
4. 现场相机 ws：`CAMERA_ROS_SETUP`（默认 `/home/user/tianji_capture_camera_ws/...`）
5. 现场 apex：`ROS_SETUP`（默认 `/opt/kernelmind/apex/install/setup.bash`）

**5090 上要有：**

1. `deploy/`（或现场 UniJEPA 树）+ `LEVJEPA_CKPT`
2. 监听 **5568**（与 Thor client 的 `INFERENCE_HOST/PORT` 一致）

**本地只需：** 能改代码、能 SSH / 能 `git push`；实机图测命令也可在「已进同一 Domain 的笔记本」上发。

---

## 2. 启停顺序（写明机器）

正式推荐（小臂在笔记本）：

| 序 | **在哪台机器** | 命令 | 验收 |
|----|----------------|------|------|
| 1 | **Thor** | `./scripts/start_skye_for_factr.sh thor` | Domain=20，链上 `6.6.7.191` |
| 2 | **笔记本**（或 Thor 同机调试） | `ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py robot_profile:=thor` | 左右 `IDLE` |
| 3 | **Thor** | `ros2 launch skye_hitl_dagger hitl_dagger.launch.py robot_profile:=thor` | arbiter 起；`/gento/*_abs` 仅 arbiter 写 |
| 4 | **Thor** | `cd docs/dagger_dev/client_thor && bash ensure_cameras.sh` | 三路相机有图 |
| 5 | **5090** | `serve_unijepa_dagger.sh` / `deploy/serve_b_levjepa.sh`，口 **5568** | 在听 |
| 6 | **Thor** | `cd docs/dagger_dev/client_thor && bash run_client_dagger.sh` | `DAGGER mode` / `published PolicyActionChunk`；**无** Gento link |

单人调试也可把 1–4、6 全堆在 **Thor 一台**（见 phase1「单人调试」）；**仍然不是你的本地 vitai 笔记本开发环境**（除非那台就是 Thor）。

每个 **ROS 相关终端**（Thor / 笔记本）：

```bash
export ROBOT_PROFILE=thor
export ROS_DOMAIN_ID=20
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
cd /data/coding/tianji/Skye_ROS_Bridge   # 以该机实际路径为准
source skye_ros2_ws/install/setup.bash
```

收工：6→4→3→2→1（先 `return` 再停小臂，见 HITL 文档 §3.4）。

---

## 3. 分阶段验收（命令在「已进 Domain 20」的机器上跑）

图测 / `intervention_cmd`：在 **Thor 或已对齐 Domain 的笔记本** 均可；**不要**在未进机器人网的纯本地瞎跑。

### C0 — 图测（无策略）

```bash
ros2 topic echo /teleop/state --once
ros2 topic info /gento/left_joint_control_abs   # Publisher=arbiter
ros2 topic hz /gento/joint_states
```

### C1 — HITL 接管（可不启 5090 / client）

```bash
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'takeover'}"
# 两侧 SYNCED 后
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'enter_teleop'}"
# 轻动小臂 → 大臂跟随
ros2 topic pub --once /skye/intervention_cmd std_msgs/msg/String "{data: 'return'}"
```

### C2 — 口 A 冒烟（可选，假策略）

```bash
ros2 run skye_hitl_dagger pub_dummy_policy_chunk
ros2 topic echo /skye/policy_action --once
```

### C3 — UniJEPA 闭环

1. **5090** serve + **Thor** `run_client_dagger.sh`
2. Thor 日志：`DAGGER published PolicyActionChunk`；`ros2 topic hz /skye/policy_action`
3. 仍只有 arbiter 写 `/gento/*_abs`
4. `takeover` → `enter_teleop` → 纠偏 → `return`

### C4 — 禁区

| 检查 | 期望 |
|------|------|
| DAgger client 与 driver 同机 | 不抢 UDP 3720、不 link Gento SDK |
| 未再起旧 `run_robot.sh` | 仅 arbiter 写臂 |
| Domain 全链路 20 | 无「半通」 |

---

## 4. 现场常改环境变量

| 变量 | 默认 | 在哪设 |
|------|------|--------|
| `INFERENCE_HOST` / `INFERENCE_PORT` | `192.168.201.253` / `5568` | Thor 跑 client 时 |
| `SKYE_WS` | 该机上的 `skye_ros2_ws` | Thor |
| `ROS_SETUP` / `CAMERA_ROS_SETUP` | Thor 现场 apex / 相机 ws | **仅 Thor** |
| `SKIP_CAMERAS=1` | — | 相机已起时 |
| `LEVJEPA_CKPT` | 现场权重路径 | **仅 5090** |
