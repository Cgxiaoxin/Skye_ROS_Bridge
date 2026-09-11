# 开发笔记

## 项目目标与现状

目标：让“日常遥操”完全脱离 `marvin_ws`，包括替换 Docker 中闭源的 `factr_teleop` 小臂程序；后续应能适配其他从臂机器人。

已确认：现有 `skye_ros2_ws/src/skye_robot_driver` 已能替代旧的 `marvin_ws/gento_ros2_ws/src/gento_robot_driver`，并已在真机成功跑通当前“日常遥操”。不需要重新开发大臂驱动。

当前主线文件：

- `skye_ros2_ws/src/skye_robot_driver/`
- `docs/dev_plan.md`
- `docs/小臂大臂启动步骤.md`
- `docs/小臂rostopic.xlsx`（同事运行旧项目后导出的话题记录）

计划新建的软件包：`skye_leader_teleop`。

职责：控制小臂；执行同步（sync）；在遥操（teleop）时读取小臂关节角并向现有 `skye_robot_driver` 发布大臂目标角。

开发原则：先做单臂、无夹爪的最小真机闭环；真机验证后再加右臂、双臂、夹爪、多从臂适配和稳定性工程。不要先做泛化框架、GUI 或复杂算法。

## 最小闭环的四个步骤

1. **把旧 `factr_teleop` 当作黑盒，记下接口和行为契约。**
   在旧程序仍能正常运行时，确认它的节点、话题、消息内容和模式切换。除关节消息格式外，也要确认它何时认为同步完成，以及从同步切到遥操时第一下会怎样动作。
2. **只实现“单臂、无夹爪”的替代程序。**
   新包 `skye_leader_teleop` 首版只做：连接一只小臂、读取并发布 7 个关节角；sync 时让小臂跟随大臂；teleop 时把小臂角发布到 `/gento/left_joint_control`；stop 时停止发送大臂目标。不要先做双臂、夹爪、GUI、录包、多机器人抽象或复杂算法。
3. **真机验证替代闭环。**
   关闭旧 Docker `factr_teleop`，只启动新节点和现有 `skye_robot_driver`：

   ```text
   新小臂节点 → /gento/left_joint_control → skye_robot_driver → 大臂
   ```

   先单臂、低速、小幅，确认 `sync → teleop → stop` 正常。
4. **逐项替换和工程化。**
   在核心闭环稳定后，按以下顺序扩展：

   ```text
   右臂 → 双臂 → 夹爪 → 启动脚本 / Docker 删除 → 第二种从臂适配
   ```

   第二种从臂真正接入时，再把“大臂输出”拆成可替换的适配层；稳定性和算法改进也以真机实际暴露的问题为准。

## 待办事项

1. 用旧 `factr_teleop` 再做一次观察，确定“对齐成功”的关节误差阈值和持续条件。
2. 确认从 `SYNCED` 切换到 `TELEOP` 的第一帧行为：是否从当前位置作相对跟随，避免大臂突然追赶。
3. 创建 `skye_leader_teleop` ROS 2 包；只实现左臂、无夹爪的 `IDLE → SYNC → TELEOP → STOP`。
4. 关闭旧 Docker `factr_teleop`，在真机上验证新节点与 `skye_robot_driver` 的单臂低速、小幅闭环。
5. 按顺序加入右臂、双臂、夹爪；最后移除 `marvin_ws` 的启动、配置和 Docker 依赖。
6. 第二种从臂机器人实际接入时，再设计从臂适配层。

### 为什么需要补测前两项

1. **“什么时候算对齐成功”需要一个明确数字。**
   按下同步后，小臂要先移动到和大臂差不多的位置。若不知道允许相差多少（例如每个关节相差小于多少 rad），规则太严格会让小臂可能永远无法完成同步；规则太宽松则会在两臂仍相差很大时开始遥操。
2. **需要知道从同步切到遥操时的第一下如何动作。**
   若切换时没有正确记住两臂当前位置，大臂可能会误以为自己需要追赶一个很远的位置，因而突然动作。需要确认旧程序是从当前位置开始一点点相对跟随，还是立刻追赶固定位置；新程序应采用平稳且安全的行为。

## Git 协作方式

- `origin` 是自己的 fork：只向它推送。
- `upstream` 是同事仓库：只拉取，不推送。

首次配置：

```bash
git remote add upstream <同事仓库 Git URL>
git remote set-url --push upstream DISABLED
git fetch upstream
```

日常开始新需求：

```bash
git switch main
git fetch upstream
git merge --ff-only upstream/main
git push origin main
git switch -c feature/<任务名>
```

- `git switch -c feature/<任务名>`：从当前 `main` 新建并切换到一个工作分支。
- 分支名整体是名称；`feature/` 是分类习惯，`/` 不会创建文件夹。
- 本项目分支名可使用 `_`，例如：`feature/decouple_daily_teleop_from_marvin`。
- 开发中需要吸收同事的新提交时：

```bash
git fetch upstream
git switch feature/<任务名>
git merge upstream/main
git push origin feature/<任务名>
```

开发过程中的版本管理：

- 所有开发都在当前功能分支 `feature/<任务名>` 中进行；不要在 `main` 上修改、提交或推送开发代码。
- 每完成一个可独立说明的小步骤就提交一次，例如“创建包骨架”“实现左臂 sync”“接入 teleop”。
- 每次提交前，确认自己仍在正确的功能分支，并检查本次改动：

```bash
git branch --show-current
git status
git diff                 # 尚未 git add 的改动
git diff --staged        # 已 git add、将进入下一次 commit 的改动
```

- 在功能分支提交并推送：

```bash
git add <文件或目录>
git commit -m "feat: <本次完成的事情>"
git push origin feature/<任务名>
```

- 提交信息前缀约定：`feat:` 新功能，`fix:` 修问题，`docs:` 文档，`chore:` 配置或杂务。
- 查看最近历史与某次改动：

```bash
git log --oneline --decorate -10
git show <提交号>
```

- 功能完成后，从自己的 `feature/<任务名>` 向自己的 `main` 发 PR；合并后再同步本地 `main` 并删除功能分支：

```bash
git switch main
git pull origin main
git branch -d feature/<任务名>
git push origin --delete feature/<任务名>
```

- 若 `git status` 有未提交改动，先在当前功能分支提交，再切换分支、同步上游或删除分支。

## 已知配置与启动约定

- 日常遥操 ROS Domain：`ROS_DOMAIN_ID=21`。
- 主机与 Docker 使用 FastDDS 且关闭 SHM：

```bash
export ROS_DOMAIN_ID=21
unset ROS_LOCALHOST_ONLY
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/marvin_ws/fastrtps_no_shm.xml"
```

- 大臂控制器默认 IP：`6.6.7.190`；同一控制器同时只能有一个 SDK 客户端。
- 已有大臂驱动构建命令：

```bash
cd skye_ros2_ws
./scripts/build.sh
source install/setup.bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py
```

- 当前现场流程参考：`docs/小臂大臂启动步骤.md`。

## 新程序接口说明书（v0.1）

每侧小臂可运行一个节点，例如：

```text
skye_leader_teleop_left
skye_leader_teleop_right
```

### 必须兼容的 ROS 2 话题

| 方向 | 话题 | 消息类型 | 约定 |
|---|---|---|---|
| 小臂 → 大臂 | `/gento/left_joint_control` | `sensor_msgs/msg/JointState` | 左臂 `position=[q1..q7]`，固定 7 项，单位 rad；`name`、`velocity`、`effort` 为空。 |
| 小臂 → 大臂 | `/gento/right_joint_control` | 同上 | 右臂，固定 7 项，单位 rad。 |
| 大臂 → 小臂 | `/gento/left_joint_states` | `sensor_msgs/msg/JointState` | 左臂 7 项；`name`、`position`、`velocity`、`effort` 各 7 项；单位 rad、rad/s、N·m。 |
| 大臂 → 小臂 | `/gento/right_joint_states` | 同上 | 右臂，格式同左臂。 |
| 外部 → 小臂 | `/mode/switch_sync` | `std_msgs/msg/String` | 收到消息后请求同步；推荐 `data="switch_sync"`。 |
| 外部 → 小臂 | `/mode/switch_teleop` | 同上 | 同步完成后请求进入遥操；推荐 `data="switch_teleop"`。 |
| 外部 → 小臂 | `/mode/switch_stop` | 同上 | 请求进入空闲；推荐 `data="switch_stop"`。 |
| 小臂 → 外部 | `/teleop/state` | `std_msgs/msg/String` | 状态为 `IDLE`、`TELEOP_SYNCING`、`SYNCED`、`TELEOP`；旧程序为 transient-local QoS。 |

当前项目对外接口应直接用上表 `/gento/*` 名称。旧记录中的 `/<side>_joint_control`、`/<side>_joint_state` 是泛称/内部 remap 形式；不要重新引入单数 `joint_state`。

### v0.1 状态流程

```text
启动 → IDLE
IDLE + /mode/switch_sync → TELEOP_SYNCING
小臂对齐大臂 → SYNCED
SYNCED + /mode/switch_teleop → TELEOP
任意状态 + /mode/switch_stop → IDLE
```

- `IDLE`：不向大臂发送关节控制指令。
- `TELEOP_SYNCING`：小臂向大臂当前关节角靠拢。
- `SYNCED`：小臂已足够接近大臂，等待进入遥操。
- `TELEOP`：以约 125 Hz 向 `/gento/<side>_joint_control` 发布小臂关节角。
- 旧程序的小臂当前状态约以 500 Hz 发布；大臂控制指令名义 125 Hz（500 Hz 循环每 4 次）。

### 暂不纳入 v0.1

- `/mode/switch_telepose`：旧项目中未完成，键盘也不发布。
- 夹爪：`/<side>_teleop_gripper/ctrl`、`/<side>_gripper/state`。
- `/leader_arm/current_state`：旧程序左右臂共用一个 topic，消息无左右标记；不要照搬。若未来确有需求，使用 `/leader/left/joint_states`、`/leader/right/joint_states`。
- 多种从臂机器人适配：首版固定输出 `/gento/*`；第二种从臂实际接入时再抽取适配层。

## 补充：SYNC 判定与新包实施结论（2026-09）

### 对应待办 1：观察旧 `factr_teleop` 的 SYNC 判定

#### 旧 FACTR 的可配置项

- 日常启动所用的 FACTR 配置为 `marvin_ws/configs/<thor|orin>/grav_comp_m6_{left,right}.yaml`。
- `scripts/run_marvin_m6_impedance.sh` 启动前会调用 `scripts/sync_marvin_overlay.sh`，将上述 profile 配置复制到 `marvin_ws/install/share/factr_teleop/configs/`；不要直接修改 `install/` 下的副本。
- 已确认配置中只有 `controller.teleop_sync_timeout: 10.0`，其含义是同步最长等待时间/超时，不是关节误差阈值或误差持续满足时间。
- 现有日常流程未发现可设置“每关节误差阈值”及“连续满足多久”的 FACTR 配置项；这两个规则很可能在闭源 `factr_teleop` 内部实现。真机运行时可复查是否存在隐藏 ROS 参数：

```bash
ros2 param list /factr_teleop_left | rg -i 'sync|threshold|tolerance|error|duration|hold'
ros2 param list /factr_teleop_right | rg -i 'sync|threshold|tolerance|error|duration|hold'
```

#### 旧 FACTR 的节点与业务话题

- `/factr_teleop_left` 和 `/factr_teleop_right` 是旧 FACTR 的 ROS 节点名，不是话题。运行后核查：

```bash
ros2 node list | rg 'factr_teleop_(left|right)'
ros2 node info /factr_teleop_left
```

- FACTR launch 对业务话题作了全局 remap，故它们不带 `/factr_teleop_left` 前缀。左臂使用 `/gento/left_joint_states`（大臂反馈/同步输入）、`/gento/left_joint_control`（遥操输出）和 `/left_leader_arm/current_state`（小臂当前状态）；右臂对应替换为 `right`。

### 对应待办 2：确认 `SYNCED → TELEOP` 的首帧安全行为

#### 当前日常流程的安全映射设置

- 当前日常链路中，`SYNCED → TELEOP` 时“首帧保持、后续相对跟随”的安全行为由主机侧 `skye_robot_driver` 实现，不在 Docker 内 FACTR YAML 中。配置位于 `skye_ros2_ws/src/skye_robot_driver/config/skye_robot.yaml`：

```yaml
teleop_mapping_mode: relative  # relative | absolute
```

- 必须保持 `relative`。它在遥操控制流恢复时重新捕获小臂与大臂参考角，首帧命令为大臂当前角；`absolute` 是旧式按小臂绝对角追赶的行为，不应用于日常遥操。
- `scripts/start_skye_for_factr.sh` 启动 `skye_robot_driver` 时读取该配置。修改 YAML 后需重新构建并重启驱动；启动后可核查实际值：

```bash
ros2 param get /skye_robot_driver teleop_mapping_mode
```

预期为 `relative`。

### 对应待办 3：创建左臂、无夹爪的 `skye_leader_teleop`

#### 新 `skye_leader_teleop` 的暂定同步规则

同事已有实测经验：每个关节在目标角的正负 5° 内持续约 5 秒即可完成同步。v0.1 将此作为可调默认值，而非硬编码：

```yaml
sync_joint_error_threshold_rad: 0.0872665  # 5°
sync_hold_time_s: 5.0
```

- 只有 7 个关节均满足 `abs(q_leader - q_follower) <= threshold`，并连续保持 `sync_hold_time_s`，才从 `TELEOP_SYNCING` 进入 `SYNCED`。
- 任意一个关节越界，连续计时重新开始。
- 后续可用旧 FACTR 真机数据校准参数；这不阻塞创建新 ROS 2 包和实现状态机。

#### 新程序应采用的安全行为

- 新包应采用相对跟随，而非根据小臂绝对角让大臂追赶。
- 进入 TELEOP 的首帧，记录小臂参考关节角 `q_leader_ref` 与大臂当前关节角 `q_follower_ref`；首帧命令为 `q_follower_ref`，故大臂保持当前位置。
- 后续仅跟随小臂相对位移：`q_cmd = q_follower_ref + sign * (q_leader - q_leader_ref)`。因此即使同步仍有残差，也不会在切换时突然追赶。

#### 推荐下一步

先创建 `skye_leader_teleop` 最小 ROS 2 包，限定左臂、无夹爪，并实现 `IDLE → TELEOP_SYNCING → SYNCED → TELEOP → IDLE`。同步阈值和保持时间参数化；TELEOP 实现上述相对映射。至少覆盖以下纯逻辑测试：单轴越界、未满保持时间、连续满足后同步、中途越界重计时、TELEOP 首帧保持、任意状态 STOP 后不再发布控制指令。
