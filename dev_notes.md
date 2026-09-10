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
