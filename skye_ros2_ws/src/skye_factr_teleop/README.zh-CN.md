# skye_factr_teleop

[English README](README.md)。后续修改行为、命令或参数时，请同步更新中英文两版 README。

这是一个基于 ROS 2 Humble `ament_python` 的 Skye/M6 FACTR 风格主臂遥操包。

包内包含经过校验的 YAML 配置、正式使用的 Dynamixel 适配器、不依赖 ROS 的单周期控制核心、单臂 ROS 驱动节点，以及左臂、右臂和双臂 launch 文件。开发计划阶段 7 的低电流硬件冒烟检查仍待完成。

## 配置

Thor M6 未标定的控制模板安装在 `config/grav_comp_m6_left.yaml` 和 `config/grav_comp_m6_right.yaml`。可以用 `skye_factr_teleop.config.load_config` 加载并校验其中任一文件。返回的嵌套 dataclass 和数值元组不可修改。

Thor 搭配遥操作设备 A，Orin 搭配遥操作设备 B；Orin 配置安装在 `config/orin/` 下。两套配置分别使用对应设备的舵机端口和物理方向，原有的各自增益与臂关节限位屏障设置保持不变。新包尚未对这两种配对完成真机验收。这些配置描述主臂硬件，而不是从臂关节方向的语义映射。

在左右臂 YAML 中设置 `controller.joint_gravity_comp.mode`，选择补偿计算方式：

| 模式 | 模型计算 |
|---|---|
| `position_velocity`（默认，省略时也采用此值） | `RNEA(q, dq, 0)`，与 FACTR 一致 |
| `position_only` | `RNEA(q, 0, 0)`，仅计算重力 |

修改模式后，需要重新加载配置并重新创建控制核心。`position_only` 仅将传给逆动力学的速度置零；关节限位阻尼、摩擦补偿的速度判断和零空间阻尼仍使用实测速度。静摩擦补偿的幅值使用所选模式下乘以增益的补偿结果；即使通过 `ControlTerms` 关闭重力输出，也按此计算。

配置中的主臂和从臂 URDF 资源由 `skye_factr_description` 包提供；模型不从 `marvin_ws` 加载。

## 程序入口

```bash
ros2 run skye_factr_teleop skye_factr_teleop_driver --help
```

键盘程序入口有意推迟到基础硬件验收完成后再实现。

## 任务 5.1：ROS 驱动节点

使用可执行程序参数 `--side left|right` 选择侧别，默认 `left`。默认节点名分别为 `skye_factr_teleop_left` 和 `skye_factr_teleop_right`，也支持标准 ROS 节点名重映射。每侧提供一份包含控制配置、标定偏移和运动上限的 YAML。唯一的 ROS 参数是启动后只读的 `config_path`：

| 参数 | 含义／默认值 |
|---|---|
| `config_path` | YAML 绝对路径或相对于包 share 目录的路径；留空时使用 `config/grav_comp_m6_<side>.yaml` |
| `dynamixel.joint_offsets` | 必填：8 个物理舵机的标定偏移，单位 rad，包含夹爪 |
| `dynamixel.current_limit_ma` | 必填：按物理舵机 ID 1..8 排列的 8 个正数电流上限，单位 mA，逐项不得超过对应型号上限 |
| `controller.sync_hold_control.ruckig.max_velocity`、`max_acceleration`、`max_jerk` | 遥操必填：按主臂逻辑关节顺序排列的 8 个正数上限，单位分别为 rad/s、rad/s²、rad/s³ |
| `runtime.state_timeout` | 消息头最大年龄与接收后最长时间，单位秒；默认 `0.25` |
| `runtime.future_tolerance` | 消息头时间戳允许超前的秒数；默认 `0.05` |
| `runtime.allow_unnamed_joint_states` | 默认 `false`；设为 true 后，无名称消息按固定 J1..J7 顺序解释 |

复制对应的 Thor 或 Orin YAML 作为每台设备的独立文件。`joint_offsets` 已预填 8 个 `0.0`；须使用 Dynamixel Wizard 核实主臂 7 轴在 URDF 零姿态、夹爪在其定义零位时的 Present Position。`current_limit_ma` 已按物理舵机 ID 1..8 预填 8 个 `50.0 mA`，仅供低电流验证，正式遥操须逐轴实测后调整。`controller.sync_hold_control.ruckig` 下的三个轨迹上限仍需填写，包内模板不能直接用于遥操启动：

```yaml
dynamixel:
  current_limit_ma: [50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0, 50.0] # 低电流验证值，mA
  joint_offsets: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # 须用 Dynamixel Wizard 核实
controller:
  sync_hold_control:
    ruckig:
      max_velocity: [<8 个正数上限，rad/s>]
      max_acceleration: [<8 个正数上限，rad/s²>]
      max_jerk: [<8 个正数上限，rad/s³>]
```

缺少或无效的值会使节点在打开串口前启动失败。节点通过 `skye_factr_description` 查找主臂 URDF，并使用 `dynamics.py` 中的 Pinocchio 适配器执行 RNEA、局部关节雅可比和正运动学。运行时 Python 依赖包括基线版本的 Pinocchio 和 Ruckig。

通过阶段 7 硬件关卡后，操作员提供合并后 YAML 的左臂启动命令示例如下：

```bash
ros2 run skye_factr_teleop skye_factr_teleop_driver --side left --ros-args \
  -p config_path:=/path/to/left_calibrated.yaml \
  -r /joint_state:=/gento/left_joint_states \
  -r /joint_control:=/gento/left_joint_control \
  -r /joint_move:=/left_joint_move \
  -r /leader_arm/current_state:=/left_leader_arm/current_state \
  -r /leader_arm/target_joint_state:=/left_leader_arm/target_joint_state
```

右臂实例使用 `--side right`、右臂标定 YAML 和右侧话题重映射。模式话题全局共享。节点提供以下基本接口，QoS 遵循阶段 0 冻结的契约：

| 方向 | 话题 | 类型 |
|---|---|---|
| 订阅 | `/joint_state`、`/joint_move` | `sensor_msgs/msg/JointState` |
| 订阅 | `/mode/switch_sync`、`/mode/switch_teleop`、`/mode/switch_stop` | `std_msgs/msg/String` |
| 发布 | `/leader_arm/current_state`、`/leader_arm/target_joint_state` | `sensor_msgs/msg/JointState` |
| 发布 | `/joint_control` | `sensor_msgs/msg/JointState` |
| 发布 | `/teleop/state` | `std_msgs/msg/String` |

状态数据使用 RELIABLE/VOLATILE/KEEP_LAST(1)；关节命令使用 BEST_EFFORT/VOLATILE/KEEP_LAST(1)。模式请求使用 RELIABLE/VOLATILE/depth 10；状态文本使用 RELIABLE/TRANSIENT_LOCAL/depth 1。

输入的 JointState 消息必须包含 7 个有限的 position 值；velocity 和 effort 必须为空，或各包含 7 个有限值。除非显式允许空名称，否则名称及顺序必须严格为 `l_j1..l_j7` 或 `r_j1..r_j7`。消息在节点 ROS 时钟域内必须具有非零且新鲜的时间戳。节点不会从组合的 14 轴消息中截取数据；它还检查自接收以来的单调时间，因此即使 ROS 时钟暂停，旧反馈也不会一直有效。

启动时使用 `DynamixelDriver.open(enable_torque=False)` 打开并配置总线，保持力矩关闭。处于 `IDLE` 时，有效的 `switch_sync` 请求必须先有新鲜的从臂反馈，随后才会使能力矩。Ruckig 完成后进入 `HOLD_SETTLING`；实测位置和速度连续达标后进入 `SYNCED`，此时 `switch_teleop` 才允许发布 7 轴控制命令。在从臂同步、保持或遥操期间，反馈无效或过期会停止运动并返回 `IDLE`。

`/joint_move` 仅在 `IDLE` 接受：它让主臂独立走到消息给出的 7 轴目标并保持，夹爪使用配置中的设定值。稳定达标后报告 `SYNCED`，但节点**不会**授予从臂遥操权限。需要先停止，再重新执行从臂同步，之后才能请求遥操。重复的走位请求不会替换正在运行的轨迹。目标状态话题发布的是运行中的参考值或保持目标；它不是输入话题。

回调在默认的互斥回调组中串行执行。定时器以配置频率使用稳定时钟；控制核心时间戳使用单调时钟。停止操作会关闭力矩并停止发布遥操命令。控制故障会取消定时器、尝试零电流和 Torque Disable，并关闭总线；故障后的节点需要重启。SIGINT 和节点销毁使用同一套可重复调用的清理过程，即使零电流写入失败，也会尝试关闭总线。

任务 5.1 测试通过注入硬件和轨迹，创建真实 ROS 节点及 executor。它们使用独立的本地 ROS domain，不打开设备：

```bash
python3 -m pytest -q test/test_ros_driver.py test/test_dynamics.py test/test_dynamixel_driver.py test/test_control_core.py test/test_state_machine.py test/test_sync_hold.py test/test_state_trajectory_integration.py
```

结果：211 项通过。节点测试需要 ROS Humble，模型测试需要 Pinocchio。跨节点消息与重映射验收属于任务 5.3。

## 任务 5.2：基础 launch

三个入口只启动所选主臂节点，均使用 `skye_factr_teleop_driver` 可执行程序；它们不启动键盘节点或从臂驱动。`robot_profile` 默认 `thor`，仅接受 `thor|orin`。控制 YAML 和相对文件路径从本包 share 目录解析；不会修改环境变量或 Python 模块搜索路径。

| 参数 | 含义 |
|---|---|
| `robot_profile` | `thor`：`config/grav_comp_m6_<side>.yaml`；`orin`：`config/orin/grav_comp_m6_<side>.yaml` |
| `left_config`、`right_config` | 每侧合并后 YAML 的路径；留空时使用 profile 模板，但模板缺少标定，不能直接启动遥操 |

每侧配置路径支持绝对路径或相对于包 share 目录的路径。单臂 launch 只声明该侧配置；双臂 launch 在启动任一节点前检查两侧文件及必填运行值。launch 不会代填未经标定的偏移、电流或运动上限。

完成硬件验收后，使用操作员提供的标定 YAML 启动：

```bash
ros2 launch skye_factr_teleop start_teleop_m6_left.launch.py \
  robot_profile:=thor left_config:=/path/to/left_calibrated.yaml

ros2 launch skye_factr_teleop start_teleop_m6_right.launch.py \
  robot_profile:=orin right_config:=/path/to/right_calibrated.yaml

ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=thor \
  left_config:=/path/to/left_calibrated.yaml \
  right_config:=/path/to/right_calibrated.yaml
```

每侧话题独立重映射：

| 节点接口 | 左臂目标话题 | 右臂目标话题 |
|---|---|---|
| `/joint_state` | `/gento/left_joint_states` | `/gento/right_joint_states` |
| `/joint_control` | `/gento/left_joint_control` | `/gento/right_joint_control` |
| `/joint_move` | `/left_joint_move` | `/right_joint_move` |
| `/leader_arm/current_state` | `/left_leader_arm/current_state` | `/right_leader_arm/current_state` |
| `/leader_arm/target_joint_state` | `/left_leader_arm/target_joint_state` | `/right_leader_arm/target_joint_state` |

模式请求和 `/teleop/state` 保持全局共享名称。基础验收阶段使用 ROS CLI 请求模式；双臂启动时，两侧都会收到这些请求，并各自检查状态和反馈新鲜度：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

阶段 7.2 使用单臂专用的 `gravity_test_m6_left.launch.py` 或
`gravity_test_m6_right.launch.py`，由 `/mode/{side}/switch_gravity_test` 显式进入
`GRAVITY_TEST`。参数、监视话题、停止条件和真机记录格式见
[7.2 验收清单](dev_docs/checklist/phase7_2_gravity_test_checklist.md)。

限位屏障使用独立的 `limit_test_m6_left.launch.py` 或
`limit_test_m6_right.launch.py`；`/mode/{side}/switch_limit_test` 仅输出所选
ID 1--8 的限位项，并限制电流、渐增与通电时间。启动参数、记录器、上下边界
和失能回读步骤见[限位屏障清单](dev_docs/checklist/phase7_2_limit_test_checklist.md)。
软件入口已准备，左右真机限位验收尚未完成。

不启动硬件节点也可查看 launch 参数：

```bash
ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py --show-args
```

`test/test_launch.py` 在拦截进程执行后展开真实 launch 动作，检查生成的命令、参数和重映射。它覆盖三个入口和两种 profile，不打开串口设备：

```bash
python3 -m pytest -q test/test_launch.py test/test_config.py
```

## 阶段 2.5.1：只读总线检查

`skye_factr_dynamixel_readonly_check` 在不启动 ROS 控制节点的情况下调用正式使用的 `DynamixelDriver`。只读过程仅打开串口、设置主机波特率、ping ID 1 至 8、读取 Return Delay Time 和 Torque Enable，以及读取当前位置与速度；它不会写入舵机寄存器。若 Return Delay Time 不为 0、力矩并未关闭、波特率不是 4 Mbps，或 FTDI 延迟定时器不为 1，检查会拒绝继续。

构建并 source 工作空间后，从 `skye_ros2_ws` 目录执行：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --frames 500 --interval 0.02 \
  --output /tmp/skye_phase2_5_left.json
```

检查右侧总线时，改用对应的右臂 YAML 和输出路径。自动报告 `PASS` 表示串口配置、设备身份、力矩关闭状态和采样数值范围通过检查。USB 左右侧映射、实体 ID 顺序、关节方向及连续角度运动仍需操作员核对。将结果用作阶段 2.5.1 验收依据前，应遵循[阶段 2.5.1 只读检查清单](dev_docs/checklist/phase2_5_readonly_checklist.md)。

首次验收 ID 和方向时，使用引导模式。它保持总线只读，提示操作员逐个设备做一次正向运动，自动采集静止、运动、停止宽限期和稳定终点的时间窗口，并为每侧生成一份汇总 JSON 记录：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --guided --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --baseline-frames 50 --movement-frames 200 \
  --settle-frames 50 --endpoint-frames 100 \
  --interval 0.02 --max-abs-velocity 20.0 --max-position-step 0.2 \
  --max-hold-velocity 0.05 --max-hold-position-span 0.01 \
  --output validation_records/phase2_5_1/left_guided.json
```

右侧使用右臂 YAML 重复检查。只有总线检查和 8 个设备的引导式 ID／方向检查全部通过，命令才会成功。

若实体目标识别已通过其他方式完成，可加入 `--direction-only`。在该模式下，非目标设备运动和非目标设备终点不稳定会记为警告，而不是失败。目标设备终点不稳定同样记为警告；目标方向、目标运动、异常采样速度和位置不连续仍判为失败。带警告的成功结果会报告 `PASS_WITH_WARNINGS`；它仅验证关节方向，不验证实体 ID 顺序或终点稳定性。

验收编码器跨界时，每侧运行一次引导式连续检查。对安全的关节执行测试，对不适合运动的关节记录明确的跳过原因：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --guided-continuous --boundary-only --boundary-mode nonzero \
  --boundary-margin 0.05 --ids 1,2,3,4,5,6,7 --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --baseline-frames 50 --continuous-max-frames 750 \
  --settle-frames 50 --endpoint-frames 100 --interval 0.02 \
  --max-position-step 0.2 --motion-reversal-tolerance 0.02 \
  --max-hold-velocity 0.05 --max-hold-position-span 0.01 \
  --output validation_records/phase2_5_1/left_continuous.json
```

`--boundary-mode nonzero` 选择最近的非零 `k * 2π` 边界，因此不会用零附近的小幅跨越作为验收依据。测试不要求转满 `2π`：在安全前提下靠近所选边界开始，并至少跨过 `--boundary-margin` 指定的距离。在 `--boundary-only` 模式中，目标位置不连续仍判为失败；没有跨界则判为无法得出结论。方向、其他关节运动、速度、反向运动和终点稳定性问题均记为警告。工具区分 `PASS`、`PASS_WITH_WARNINGS`、`FAIL`、`INCONCLUSIVE` 和 `SKIPPED_UNSAFE` 五种结果。

## 阶段 2.5.2：单舵机低电流检查

`skye_factr_dynamixel_low_current_check` 仅对一个指定 ID 调用正式使用的 `DynamixelDriver`。任何可写打开操作之前，它会单独执行只读检查，确认型号、Return Delay Time、Torque Disable 状态、波特率和 FTDI 延迟定时器。硬件模式必须提供 `--confirm-single-servo`，并同时施加操作员明确给出的电流上限和 `min(100 mA, 型号额定电流上限的 10%)` 的硬上限。

连接硬件前，先使用 `--prepare-only`。这会校验全部参数、显示 XM430／XC330 寄存器换算，并写入 JSON 准备记录，不会构造驱动或打开串口：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --prepare-only --side left --id 1 \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --output /tmp/skye_phase2_5_2_left_id1_prepared.json
```

移除 `--prepare-only` 前，应遵循完整的[阶段 2.5.2 低电流检查清单](dev_docs/checklist/phase2_5_low_current_checklist.md)。硬件模式依次发送短时正向、零、反向、零电流命令，检查编码器方向，并始终调用正式驱动的关闭流程，写入零电流及 Torque Disable。

## 阶段 3：控制核心（无硬件）

`control_core.py` 包含显式的四分之一圈标定、物理／逻辑关节映射、可分别开关的关节限位、重力、静摩擦和零空间控制项，以及一次非阻塞的 `ControlCore.step()` 调用。运行适配层提供动力学接口（`gravity`、`jacobian`、`forward_kinematics`）和明确的电流上限；构造控制核心不会打开串口或使能力矩。力矩关闭时，`step` 不写电流。读取、计算或写入失败时，它会尝试写零电流并执行 Torque Disable。从臂力矩和夹爪反馈在阶段 8 之前只有接口定义。

固定输入与假设备测试不需要 ROS、串口硬件、Pinocchio 或 Ruckig：

```bash
python3 -m pytest -q test/test_control_core.py
```

## 阶段 4.1／4.2：状态机

轨迹和收敛超时都视为失败：`TELEOP_SYNCING -> IDLE` 或 `HOLD_SETTLING -> IDLE`。超时后不得进入 `SYNCED`，也不得允许 `switch_teleop`。应重新发出 `switch_sync`，等待成功完成（`SYNCED`），再请求 `switch_teleop`。

这改变了早期计划描述的“超时进入 `SYNCED`”行为。来源限制和调用方改动见[迁移说明](dev_docs/skye_factr_teleop_migration.md)。纯 Python 状态机已经实现；任务 4.3 也在 `ControlCore` 中接入轨迹清理与保持。

构造 `StateMachine(now=...)` 后，将显式事件传给 `handle(event, now=...)`。事件可使用 `Event` 成员或字符串名称。时间戳必须有限、来自同一时钟且不递减；状态机内部不读取时钟。控制核心在 Ruckig 完成时发出 `trajectory_complete`，实测收敛后才发出 `sync_complete`；两个阶段到达截止时间时优先处理超时。

不可变的 `TransitionResult` 包含 `previous_state`、`state`、`outcome`、`reason`、有序 `actions` 和 `entered_at`。`accepted` 仅表示事件已处理，因此被接受的超时事件仍表示同步失败。状态机在后续事件间保留 `last_rejection_reason`。`entered_at` 仅在状态改变时更新；重复的同步请求不会重启计时器。

调用方按顺序执行返回的动作：

| 动作 | 控制适配层职责 |
|---|---|
| `stop_motion` | 停止主臂自动运动、保持和遥操命令发布 |
| `clear_sync` | 清除轨迹、保持积分和之前的目标 |
| `start_sync` | 获取新鲜的同步起点和目标 |
| `start_hold` | 保持同步完成时的目标 |
| `start_teleop` | 开始发布主臂到从臂的控制命令 |

停止和超时依次返回 `stop_motion`、`clear_sync`。被拒绝或忽略的事件不返回动作。`TELEPOSE=5` 预留，`switch_telepose` 会被拒绝。此模块不产生 ROS、串口、Ruckig 或硬件副作用。

从包目录运行参数化契约测试：

```bash
python3 -m pytest -q test/test_state_machine.py
```

状态机的 44 项测试使用显式测试时间戳并全部通过；任务 4.1 的预期失败标记已移除。

## 阶段 4.3：Ruckig 同步与保持

`ControlCore.handle()` 同步执行状态机动作。`step(now=...)` 读取一次主臂状态、最多推进一次活动轨迹、计算电流并执行正常的电流写入。内部没有循环、休眠、ROS 调用或自动 Torque Enable。所有核心调用必须在一个控制循环中串行执行。按 `controller.frequency` 调用 `step`，时间戳使用同一个单调时钟的秒数；同步和保持阶段必须提供 `now`。

实际的 `RuckigTrajectory` 适配器会按需导入基线 Python 包 `ruckig==0.19.4`。它遵循官方的[在线更新示例](https://docs.ruckig.com/md_pages_2example__01.html)，每次更新后调用 `pass_to_input`。假轨迹测试不需要安装 Ruckig。

现有 YAML 没有定义轨迹的速度、加速度或加加速度上限。须通过工厂函数明确提供 8 轴正数上限，单位分别为 rad/s、rad/s² 和 rad/s³；不能从 PD 增益推断硬件运动上限：

```python
from skye_factr_teleop.control_core import ControlCore
from skye_factr_teleop.trajectory import RuckigTrajectory

# 调用方针对具体主臂提供 config、hardware、dynamics、offsets、current_limit_ma，
# 以及以下三个上限向量。
def make_trajectory():
    return RuckigTrajectory(
        frequency=config.controller.frequency,
        max_velocity=velocity_limits,
        max_acceleration=acceleration_limits,
        max_jerk=jerk_limits,
    )

core = ControlCore(
    config, hardware, dynamics, offsets, current_limit_ma,
    trajectory_factory=make_trajectory, now=t0,
)
core.handle(
    "switch_sync", now=t0,
    follower_position=follower_q, gripper_setpoint=gripper_q,
)
cycle = core.step(now=tick_time)  # 调用方每个控制周期仅执行一次。
```

被接受的同步请求会记录实测的 8 轴逻辑主臂位置和速度，并复制 7 个从臂位置与夹爪设定值。输入坐标必须事先映射到主臂逻辑坐标系。若省略夹爪目标，则使用 `controller.sync_hold_control.gripper_setpoint`。由于没有加速度测量，起始加速度设为零；目标速度和加速度也为零。每次同步都会新建轨迹。重复的同步请求不会替换目标或重启超时计时。

同步期间，轨迹跟踪力矩为 `ruckig.kp * (q_ref - q) + ruckig.kd * (dq_ref - dq)`。Ruckig 完成后进入 `HOLD_SETTLING`，对记录的最终目标保持。保持力矩为 `hold.kp * error - hold.kd * dq + hold.ki * integral`。对每个轴，仅当 `abs(error) <= hold.i_thresh` 时积分 `error * dt`；否则清零该轴积分。`hold.i_max` 对乘以 `hold.ki` 之前的累计误差限幅，单位为 rad·s。第一个保持周期从零积分开始。所有轴的实测位置误差和速度分别连续满足 `sync_position_tolerance`、`sync_velocity_tolerance` 达 `settle_duration` 后进入 `SYNCED`，并继续保持；任一轴超阈值就重新计时。超过 `settle_timeout` 则返回 `IDLE` 并关闭力矩。这四项配置位于 `controller.sync_hold_control.hold` 下；默认值需在真机上验证和调节。

同步走位和保持阶段分别使用各自的 `gravity_comp_gain`，替换常规重力增益。配置的重力模式和控制项开关仍然有效。关节限位保护和静摩擦补偿仍可使用；同步及保持时关闭零空间调节，以免与各关节完整目标相互冲突。最终力矩仍经过已有的舵机方向换算和电流限幅。

当 `elapsed >= controller.sync_hold_control.ruckig.trajectory_timeout` 时，超时处理先于 Ruckig 更新；收敛超时也先于保持更新。停止、超时以及计算／初始化失败都会清除轨迹、积分和目标，返回 `IDLE`，并尝试写零电流和执行 Torque Disable。显式 `stop()` 不需要时间戳；`handle("switch_stop", now=...)` 即使收到无效时间戳也会停止。其他故障在清理后继续向外抛出。重新尝试同步前，调用方必须有意重新使能力矩。核心 API 禁止外部注入完成／超时事件，因此旧的异步事件无法完成新一轮同步。

进入 `TELEOP` 时会清除自动跟踪与保持输出。常规补偿项仍可使用；ROS 发布由阶段 5 的适配层负责。可观察 `mode`、`last_transition` 和复制得到的 `hold_integral`。`CycleResult` 新增 `tracking_torque`、`mode`、`target_position` 和 `target_velocity`；同步／保持之外，目标字段为 `None`。`wrote_current` 描述正常周期的写入，与清理过程中用于安全停机的零电流写入分开统计。

## 阶段 4.4：状态与轨迹联动验收

`test/test_state_trajectory_integration.py` 使用假时钟、假的 Ruckig Python API 和假的 Dynamixel 硬件，运行正式控制核心、状态机和 Ruckig 适配器。22 个用例覆盖左右两侧配置：完整的同步／保持／遥操／停止流程；重复请求不产生副作用；在截止时间前、当时和之后完成的处理；停止／超时后重新捕获起点；积分清零；非法目标；以及 Ruckig 错误结果。连续几个 TELEOP 周期还验证了自动主臂目标已经清除，而常规补偿仍可继续。

从包目录运行阶段 4 验收测试和控制核心回归测试：

```bash
python3 -m pytest -q test/test_state_trajectory_integration.py test/test_sync_hold.py test/test_state_machine.py test/test_control_core.py
```

120 项检查全部通过：其中 119 项使用假对象或纯逻辑，另保留一项真实 Ruckig 0.19.4 的轨迹上限检查。仅在未安装 Ruckig 时跳过该真实库测试；联动验收使用假模块，不需要安装 Ruckig。任务 4.4 和里程碑 M4 已完成。ROS 集成和硬件验收分别属于阶段 5 和阶段 7。

## 许可证

Apache-2.0。详见 [LICENSE](LICENSE) 和 [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)。
