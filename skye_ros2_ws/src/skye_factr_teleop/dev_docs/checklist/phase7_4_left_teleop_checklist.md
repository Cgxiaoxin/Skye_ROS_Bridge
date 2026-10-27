# 阶段 7.4：左臂关节遥操操作清单

日期：2026-10-26。设备 A / Thor。用于已完成左侧真实大臂反馈同步后的首轮关节遥操。
右侧真实反馈同步调参延期；本清单不启动右侧小臂。

## 1. 本轮基线与接口

直接使用和修改原 YAML，不另建试验配置：

| 文件 | 当前基线 |
|---|---|
| `config/grav_comp_m6_left.yaml` | `joint_gravity_comp.gain=[0.5,0.2,0.7,0.6,1.0,0.8,1.0]`；J1 从 0.4 改为 0.5 |
| 同上 | 同步/保持参数沿用已验证值，含轨迹到保持的 0.5 s 交接 |
| `skye_robot_driver/config/skye_robot.yaml` | 左侧 velocity/acceleration ratio 均为 10；`max_delta_per_cycle=0.001 rad/帧`；relative；timeout=0.50 s |

`max_delta_per_cycle` 是每个被接收命令帧的增量限制，双臂及 relative/absolute 路径共用，
不是固定速度上限。以 500 帧/s 估算，0.001 rad/帧对应 0.5 rad/s 的目标增量预算；实际
频率由运行记录判断，SDK 执行还受控制模式和速率参数影响。若存在跟随积压，先停止并分析，
不在运动中放宽参数。右臂速度/加速度配置未调整，本轮不向右臂发布遥操命令。

大臂驱动共享双臂连接和控制模式，默认仍启用夹爪驱动。新记录器不发送任何控制或服务请求；
小臂本轮只输出七轴关节命令，不输出夹爪遥操命令。

| 数据 | 话题 | 类型 / QoS |
|---|---|---|
| 小臂诊断：八轴位置、速度、各项力矩、裁剪前后电流、实测电流 | `/leader_arm/left/sync_diagnostics` | JSON String / BEST_EFFORT |
| 遥操输入 | `/gento/left_joint_control` | JointState / BEST_EFFORT |
| 驱动接受并成功下发的目标 | `/gento/left_joint_action_applied` | JointState / RELIABLE |
| 大臂真实反馈 | `/gento/left_joint_states` | JointState / RELIABLE，记录器 BEST_EFFORT 订阅 |
| 状态 | `/teleop/state` | String / RELIABLE、TRANSIENT_LOCAL |

七轴名称为 `l_j1..l_j7`，位置 rad、速度 rad/s。`action_applied` 是 SDK 成功下发的目标，
不是大臂实测位置。小臂 `joint_signs` 负责物理舵机标定，大臂 `left_joint_signs` 负责命令映射，
不要再次将小臂 J6/J7 的符号套到大臂命令上。Thor 当前左侧大臂映射为原顺序、全 +1。

## 2. 编译与启动前检查

先退出旧的小臂控制节点和占用串口的工具。编译不连接硬件：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to skye_factr_teleop --symlink-install
colcon build --packages-select skye_robot_driver \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
source install/setup.bash
ros2 run skye_factr_teleop skye_factr_teleop_trial --help
```

- [ ] 原 YAML 的上述参数正确，节点将重新启动加载；`ros2 param set` 不会自动更新驱动缓存的控制参数。
- [ ] 按[同步清单 2.1 节](phase7_3_follower_sync_checklist.md#21-启动前准备编译串口检查与设置)
      检查左侧串口 `FTBEO9L1`、权限、无占用、FTDI `latency_timer=1`，USB 重插后重查。
- [ ] 停止旧 FACTR、右侧小臂、follower_align、策略及其它命令发布者。
- [ ] 大小臂在可行姿态、线缆无牵扯，操作者能持续支撑小臂并操作停止。
- [ ] 首轮使用已验证零位或常用非零目标，远离大小臂关节限位。
      J7 约 `+0.774 rad` 已进入当前小臂屏障区，首轮不使用；J7 安全区为 `[-0.7,+0.7] rad`。

所有终端执行以下环境设置，沿用独立 ROS domain 73：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_left.yaml"
DRIVER_CONFIG="$PWD/src/skye_robot_driver/config/skye_robot.yaml"
```

## 3. 启动与记录（独立终端）

**终端 A：大臂驱动。** 会连接真机并进入配置控制模式；已有驱动须先正常退出再重启以加载新 YAML，避免重复连接。

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=thor "params_file:=$DRIVER_CONFIG"
```

**终端 B：仅左侧小臂。** 启动后保持 torque 关闭。

```bash
ros2 launch skye_factr_teleop start_teleop_m6_left.launch.py \
  robot_profile:=thor "left_config:=$CONFIG" sync_diagnostics:=true
```

**终端 D：启动检查与后续控制。** 核对运行参数、模式及真实七轴反馈：

```bash
ros2 param get /skye_robot_driver teleop_mapping_mode
ros2 param get /skye_robot_driver left_velocity_ratio
ros2 param get /skye_robot_driver left_acceleration_ratio
ros2 param get /skye_robot_driver max_delta_per_cycle
ros2 param get /skye_robot_driver command_timeout_s
ros2 topic echo --once /gento/robot_state std_msgs/msg/Int16MultiArray
ros2 topic echo --once /gento/left_joint_states sensor_msgs/msg/JointState
ros2 topic info --verbose /gento/left_joint_control
ros2 topic info --verbose /gento/right_joint_control
ros2 topic info --verbose /gento/left_joint_control_abs
```

应分别为 relative、10、10、0.001、0.5；预期大臂状态 `[2,2]`（IMP_JOINT）。左侧 control
仅有当前左小臂一个发布者；右侧 control 和左侧 abs 应无发布者。小臂即使处于 IDLE 也会创建
control 发布者，发布者数量不等于正在发送命令。原 YAML 的快照不代表 launch overlay 或服务
修改后的有效值；将本组参数检查输出随本轮记录保留。若调用过 `set_motion_rates`，参数读取
可能仍为启动值，先重启驱动恢复本轮基线。

**终端 C：只读记录。** 每轮选未占用文件名，先运行记录，再请求同步：

```bash
ros2 run skye_factr_teleop skye_factr_teleop_trial \
  --side left --config "$CONFIG" --driver-config "$DRIVER_CONFIG" \
  --operator lyy --output /tmp/left_teleop_01.json \
  --duration 180 --post-stop-seconds 3
```

工具保存 JSON 汇总及同名 `.trace.jsonl`，保留原 YAML 快照，记录正常遥操命令；同时监视右侧
control、左侧 abs 和 sync/teleop/stop 请求。沿用同步阶段的收敛条件提示，显示 WAITING/STALE
时先解决诊断问题。记录器本机运行，避免经 Wi-Fi 远程 RELIABLE 订阅拖慢驱动。

记录器在观察到一次活动后的零输出 IDLE 后继续记录 3 s，或到达 180 s 上限。
**到期、Ctrl+C、关闭记录器均只保存记录，不停止大小臂。** 请求及 stop 始终在终端 D 执行，
不要排在阻塞的记录命令后面。

**终端 E：状态监视。** 本轮只运行左小臂，全局状态对应左侧：

```bash
ros2 topic echo /teleop/state std_msgs/msg/String --qos-durability transient_local
```

## 4. 第一轮：同步与静止切换

确认反馈七轴静止、目标和路径在小臂安全范围内，记录器已收到 IDLE 诊断，再在终端 D 执行：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
```

- [ ] 观察 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED`；同步期间大臂静止、无遥操命令。
- [ ] 必须等待真实反馈同步成功；`joint_move` 得到的 SYNCED 不授予遥操资格。
- [ ] 操作者保持支撑，SYNCED 稳定后再切换：

```bash
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
```

- [ ] 观察 TELEOP，并确认驱动日志 `captured leader/gento refs`。
- [ ] 最初 5～10 s 不主动运动；小臂无明显下落、过度上抬或突动，大臂首帧保持进入时姿态。
- [ ] 记录保持 PID 退出前后电流和关节响应、人工支撑感受。
- [ ] 本轮 J1 保持和遥操重力增益均为 0.5，但保持 PID 及积分仍会退出，不据此认定切换一定平顺。

relative：首帧捕获大小臂参考；随后 `大臂目标 = 大臂参考 + sign × 小臂相对位移`，
经过限位与逐帧增量限制再下发。同步的残差不要求大臂在首帧追赶。小臂 Ruckig 限速只用于
同步，不约束手动 TELEOP。

第一轮先按第 6 节停止并分析，再做逐轴运动。若切换产生明显突动，先分析 tracking、gravity、
friction 和裁剪前后电流，再决定修改；本次软件准备未增加 SYNCED → TELEOP 渐变控制。

## 5. 后续轮次：逐轴与组合运动

每轮更换输出文件、先启动记录，重新执行真实反馈同步，再进入 TELEOP。

1. J1～J7 依次做约 `0.02～0.03 rad` 的缓慢正向、回原处、负向、回原处运动。
   首次发现方向错误、非目标关节明显运动、跟随积压或报警立即停止。
2. 对照输入命令、action_applied、真实反馈，核对方向、幅值、频率、丢帧和跟随延迟。
   不以 `ros2 topic hz` 的单一数值代替实际下发和反馈检查。
3. 逐轴通过后做两三轴小范围组合运动，再检查已验证的另一安全姿态；检查总电流饱和和漂移。
4. 小臂保持人工支撑，区分无接触观察段与人工干预段，记录接触时刻和感受。

真实机械边界不作为首轮动作目标。驱动 clamp/clutch 已有离线测试；后续另选受控试验验证
到限、反向立即恢复以及其余关节继续跟随，不直接冲击硬限位。

## 6. 每轮停止、保持与退出

**终端 D 先停止小臂：** 操作者继续支撑，因为 stop 会关闭 torque。

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

在终端 E 确认 IDLE 后，立即调用大臂受控保持并确认 `success: true`：

```bash
ros2 service call /gento/hold_current std_srvs/srv/Trigger '{}'
```

`hold_current` 对双臂生效并重置两侧会话，右臂须保持静止；它可能产生双侧 applied 保持回执，
不等于右臂发生遥操。先停命令再 hold，避免后续遥操帧重新接管。

原有 `/mode/switch_stop` 不直接调用大臂保持。命令断流后驱动在 `0.50 s` 超时基础上，
通过 `50 ms` 检查定时器执行本侧 hold；响应还包含调度与 SDK 时间，不把 0.50 s 当作硬保证。
若小臂未及时停止、大臂保持失败或出现危险运动，不等待记录器，按现场应急方式处置；现有软件
急停接口为 `/gento/emergency_stop`（Trigger），物理急停由操作者准备。

等待记录器保存停止后 3 s 的数据，再正常退出小臂节点，释放串口后独立回读：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left --config "$CONFIG" --frames 2 --interval 0 \
  --output /tmp/left_teleop_01_torque_disable.json
```

- [ ] ID1..8 的 `torque_enabled=false`，独立回读 PASS；文件名与该轮匹配。
- [ ] 大臂受控保持、无继续追赶；停止后指令及反馈尾段可供核对。
- [ ] 再次进入必须重新同步；不能从 IDLE 直接恢复 TELEOP。

若小臂故障退出导致诊断停止，记录器不会收到完整 IDLE 证据；在处理停止/保持后手动保存，
保留不完整记录及故障日志，不等待 180 s 才处理。

## 7. 异常路径和验收记录

正常切换、逐轴和停止通过后，再安排低风险异常验证：

- [ ] 在静止 TELEOP 中正常退出小臂节点，验证命令断流、大臂驱动 timeout 后保持和会话重置。
- [ ] 反馈陈旧/中断触发小臂停止：离线覆盖后再做受控现场试验，不拔运动中的总线或关闭大臂驱动
      来制造反馈中断。现场故障注入方式另行确定，此项先保持待验收。
- [ ] 恢复后重新同步才能遥操；故障关闭串口的节点需重启。

JSON 中 `RECORDED` 仅表示观察到 TELEOP、零输出停止、停止后记录时窗，以及 command/applied/
feedback 三类流，**不是硬件 PASS**。后 3 s 表示记录时间，不自动证明大臂保持成功。
`receive_hz` 是整段接收平均值（含暂停），判断 TELEOP 频率需截取 trace 的对应时段。
非递增时间戳计数、诊断丢帧、无效包计数和单帧最大增量用于定位问题，不插值补帧。

每轮提供 JSON、trace、独立失能回读、参数检查和 hold 服务输出，并填写：

| 项目 | 记录 |
|---|---|
| 文件名 / 姿态 / 轮次 | 待填写 |
| SYNCED → TELEOP 的小臂响应、人工支撑情况 | 待填写 |
| 首帧、七轴方向与组合运动表现 | 待填写 |
| 电流饱和、漂移、频率与跟随滞后 | 待分析 |
| stop → IDLE、大臂 hold 回执与实际保持 | 待填写 |
| 独立失能回读 | 待填写 |
| 本轮结论与下一步 | 待填写 |

通过条件：切换无明显突动；首帧不追赶同步残差；七轴方向正确；无持续异常饱和或明显漂移；
停止后大臂保持且恢复须重新同步；有记录和操作者观察支持。当前硬件各项均待执行。

## 8. 本次软件准备验证

2026-10-26 已完成：

- `skye_factr_teleop` 与 `skye_robot_driver` 构建成功；安装入口 `skye_factr_teleop_trial --help` 可用。
- 记录器、同步记录回归、配置、launch、同步/保持及状态轨迹联动测试：258 项通过。
- ROS 针对性测试：15 项通过，覆盖遥操准入、反馈失效和实际 DDS 记录链路；使用模拟硬件。
- 大臂 C++ 映射测试：8 项通过，含首帧保持大臂参考位置、clutch 和角度展开。
- 修正配置测试中已过时的首轮电流/轨迹上限断言，保留当前已调通参数；`git diff --check` 通过。

本次验证未连接硬件，尚无 TELEOP 真实切换、跟随或大臂停止保持的现场结论。

## 9. 速度、加速度比例与逐帧增量限制说明

### 9.1 `left_velocity_ratio` 和 `left_acceleration_ratio`

这两个参数设置左侧大臂七个关节整体的速度、加速度比例，驱动允许范围均为 `1～100`，
按百分比使用；当前值均为 `10`。它们不是 rad/s、rad/s²，也不会把小臂位移乘以 0.1。

消费链路如下，源码位置均相对于 `skye_ros2_ws/src/`：

1. `skye_robot_driver/src/driver_node.cpp` 的构造函数读取 ROS 参数，将值写入
   `ConnectConfig.left_vel_ratio`、`left_acc_ratio`。
2. `skye_robot_driver/src/driver_core.cpp` 的 `enter_mode_unlocked()` 在进入当前
   `imp_joint` 模式时，将两者传给 `FX_L1_State_SwitchToImpJointMode()`；左臂对应 `FX_OBJ_ARM0`。
3. 随后通过 `FX_L1_Runtime_SetSpeedRatio(thread_id, FX_OBJ_ARM0, left_vel, left_acc)`
   设置运行时速率。SDK 声明见仓库 `third_party/gento_sdk/include/L1Robot/L1Robot.h`。

运行中 `/gento/set_motion_rates` 服务调用 `DriverCore::set_speed_rates()`，再次通过
`FX_L1_Runtime_SetSpeedRatio()` 更新 SDK 和驱动内部配置。直接 `ros2 param set` 没有对应的
下发回调，不能据此改变已下发速率；服务更新后 `ros2 param get` 也不代表 SDK 的当前值。

源码和 SDK 头文件可确认比例被下发；持续 `SetJointPosCmd` 遥操下，实际速度是否及如何
按比例变化，仍需结合控制器行为和真实反馈验证，不能仅凭参数值换算实际关节速度。

### 9.2 `max_delta_per_cycle` 的消费位置与算法

该参数在 `driver_node.cpp` 中读取并缓存，随后在两条命令路径调用
`DriverCore::limit_delta(mapped, *last_command, max_delta_per_cycle_)`：

- `handle_command()`：普通 `/gento/{left,right}_joint_control` 遥操入口。
- `handle_absolute_command()`：`/gento/{left,right}_joint_control_abs` 绝对命令入口。

逐关节限制算法位于 `driver_core.cpp` 的 `limit_delta()`，相当于：

```text
本次下发目标 = 上次成功下发目标
             + clamp(期望目标 − 上次成功下发目标, −max_delta, +max_delta)
```

处理顺序是：输入目标 → 映射 → 关节限位 → 增量限制 → 再次限位 → SDK 下发。
只有 SDK 下发成功后才更新 `last_command`，并发布 `action_applied`。
该限制参考上一次下发目标，不参考实时实测位置；左右臂共用参数值，但各自保存命令历史。

例如上次下发 `0.100 rad`，新期望目标为 `0.110 rad`，上限为 `0.001 rad/帧`，
这一帧下发 `0.101 rad`。后续持续收到该目标时逐帧靠近；没有新消息时，这段代码不会自动补发。

### 9.3 三者关系与实际速度

| 参数 | 生效层次 | 限制对象 |
|---|---|---|
| `max_delta_per_cycle` | ROS 驱动，SDK 下发之前 | 相邻下发目标的逐关节位置差 |
| `left_velocity_ratio` | SDK / 大臂控制器 | 控制器的速度比例 |
| `left_acceleration_ratio` | SDK / 大臂控制器 | 控制器的加速度比例 |

三者没有直接相乘或换算关系。增量限制不使用时间间隔，也不限制相邻帧增量的变化，
因此不是固定速度限制或加速度限制；它还不同于小臂同步阶段的 Ruckig 速度、加速度约束。

在连续、均匀处理命令时，可估算逐关节目标推进速度上限：

```text
目标推进速度上限 ≈ max_delta_per_cycle × 实际成功下发频率

0.001 rad/帧 × 250 帧/s ≈ 0.25 rad/s
0.001 rad/帧 × 500 帧/s ≈ 0.50 rad/s
```

应使用对应 TELEOP 时段的实际处理/成功下发频率，不直接使用小臂名义发布频率、SDK 配置
周期或整段记录平均频率。以上是均匀时序下的估算；消息突发、丢帧和调度变化会改变时域表现。

如果 SDK 允许的执行速度高于上述目标推进速度，且输入目标持续向前推进、每帧均触发
增量限制，大臂跟随良好时的稳态速度通常接近目标推进速度；输入运动更慢时速度也更慢。
例如目标以约 `0.25 rad/s` 推进、SDK 允许 `1.0 rad/s`，不意味着大臂就以 `1.0 rad/s` 运动，
也不能再将 `0.25 rad/s` 乘速度比例 10% 得出实际速度。

**目标推进速度不是实体关节瞬时速度的硬上限。** 大臂若落后于目标，控制器可能以高于
该推进速度的瞬时速度追赶；加速度、阻抗、负载及跟踪误差都会影响响应。
`max_delta_per_cycle` 也不保证下发目标与实体位置之间的误差大小。
现场应同时分析 `action_applied` 的目标推进和 `joint_states` 的实测位置、速度，
分别判断命令限制效果、跟随滞后及实际运动速度。

## 10. 隔离大臂命令，单独测试小臂切换与补偿

本方法用于先解决小臂切入 TELEOP 时的托力突变、进入后的自行转动和持续支撑不足。
只隔离小臂向大臂发送的命令，保留真实大臂反馈和现有同步准入；大臂驱动继续运行并保持静止，
不是关闭大臂驱动或使大臂失能。重力方向、零位和关节映射按操作者此前已验证的结论处理，
本轮不重复检查；隔离入口本身不改变控制参数，后续逐轴对照的参数变更见第 11 节。

### 10.1 停止旧节点，使用隔离入口启动

所有终端沿用第 2 节的主机环境、工作区、ROS domain 73 和 `CONFIG`。
若原小臂节点正在运行，操作者先持续支撑小臂，在独立控制终端发布：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

确认小臂状态为 IDLE 后，再调用大臂保持，确认 `success: true`：

```bash
ros2 service call /gento/hold_current std_srvs/srv/Trigger '{}'
```

该保持服务对双臂生效，右臂须静止。随后退出原左小臂节点、释放串口。
保留大臂驱动运行，停止其它运动命令发布者。

本轮用下列命令替代第 3 节的左小臂 launch，不同时运行两个小臂控制节点：

```bash
ros2 run skye_factr_teleop skye_factr_teleop_driver \
  --side left --ros-args \
  -p "config_path:=$CONFIG" \
  -p sync_diagnostics:=true \
  -r /joint_state:=/gento/left_joint_states \
  -r /joint_control:=/left_teleop_trial/joint_control \
  -r /joint_move:=/left_joint_move \
  -r /leader_arm/current_state:=/left_leader_arm/current_state \
  -r /leader_arm/target_joint_state:=/left_leader_arm/target_joint_state
```

现有 launch 将 `/joint_control` 固定映射到大臂命令话题，因此本轮直接运行节点并显式重映射。
节点没有固定的启动成功提示；若终端持续运行且无报错，通过状态与诊断判断是否正常，
不要因启动终端安静而重复启动节点。

### 10.2 确认隔离和控制循环

在另一个终端执行：

```bash
ros2 topic info --verbose /gento/left_joint_control
ros2 topic info --verbose /left_teleop_trial/joint_control
ros2 topic info --verbose /gento/left_joint_control_abs
ros2 topic echo --once /teleop/state std_msgs/msg/String \
  --qos-durability transient_local
```

- [ ] `/gento/left_joint_control` 的 `Publisher count: 0`；大臂驱动保留订阅是正常的。
- [ ] `/left_teleop_trial/joint_control` 只有 `skye_factr_teleop_left` 一个发布者，
      `Subscription count: 0`，大臂驱动不订阅该话题。
- [ ] `/gento/left_joint_control_abs` 无发布者，避免其它绝对命令控制大臂。
- [ ] 新启动小臂状态为 IDLE；真实大臂反馈仍有效，大臂静止。

需要确认诊断持续发布时执行：

```bash
ros2 topic hz /leader_arm/left/sync_diagnostics
```

当前配置名义频率为 500 Hz；已有隔离启动检查观察到约 500 Hz。
确认后 Ctrl+C 只退出 `topic hz`，让小臂启动终端继续运行。

### 10.3 记录与简化补偿检查

本轮使用只读同步记录器，保存小臂各项力矩、电流、位置及速度；每轮选未占用的文件名。
以下使用第 02 轮文件名，避免覆盖已有第 01 轮：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side left --config "$CONFIG" --operator lyy \
  --output /tmp/left_isolated_comp_02.json --duration 120
```

当前 `skye_factr_teleop_trial` 期待真实大臂命令和 applied 数据，隔离后不满足其完整记录条件，
因此这里使用 `skye_factr_sync_trial record`。同步记录器也会保存 TELEOP 诊断，
真实 `/gento/left_joint_control` 的命令计数应为 0。

等待记录器收到 IDLE 后，按原方式请求真实反馈同步：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
```

必须等待 `TELEOP_SYNCING → HOLD_SETTLING → SYNCED`。操作者充分支撑小臂，
尤其避免腕部 J6/J7 自由下垂，再请求：

```bash
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
```

采用一次短检查，不松手测试自由下落：

1. 切换前后保持托住，判断托力是否突然增大或方向突变。
2. 在持续支撑下缓慢改变两三个小范围安全姿态，每个姿态停几秒，感受持续压手、向上顶手
   或自行转动的趋势；出现难以控制的运动立即停止，不必做完所有姿态。
3. 反馈“只在切换瞬间突变，还是进入后持续需要明显托力”，以及主要受影响的关节/部位。
   当前现场反馈中，J1/J3/J5 的自行转动是优先观察项，不将自行转动直接等同于欠补偿。

姿态能停住并不自动证明重力补偿合适，尤其须结合记录中的限位屏障力矩判断；
J6/J7 接近安全区边沿时，可能由屏障参与支撑。优先在安全区内部观察。

### 10.4 停止、独立回读与恢复真实遥操

短检查结束即在独立终端发布 stop，操作者继续支撑小臂：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

确认 IDLE，让记录器保存，再正常退出小臂节点。释放串口后独立回读失能：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left --config "$CONFIG" --frames 2 --interval 0 \
  --output /tmp/left_isolated_comp_02_torque_disable.json
```

确认 ID1..8 的 `torque_enabled=false`，并将现场感受、JSON、trace 和失能记录一并保留。
同步记录器首次观察到活动后零输出 IDLE 即结束，不额外保留 3 s 尾段。
到期或 Ctrl+C 只保存记录，不停止小臂；若输出 INCOMPLETE 且没有 IDLE，应先确认小臂状态并停止，
不能把记录器结束当作控制结束。

恢复真实遥操时，先确认已 stop，退出隔离节点，按第 3 节原 launch 启动；
重新核对真实大臂命令话题的发布者和隔离话题不再有小臂发布者，然后重新同步。
话题隔离不会自动恢复，也不把本节的补偿测试结论当作带大臂遥操验收通过。

## 11. J1/J3/J5 姿态相关自行转动：记录与简化对照

### 11.1 第 01 轮实际配置与结果（2026-10-26）

记录 `/tmp/left_isolated_j1_friction0_01.json` 及同名 trace：状态完整经过同步、TELEOP、IDLE，
诊断无丢帧或序号重启，无软件电流裁剪，真实大臂命令计数为 0。
同名前缀的 `_torque_disable.json` 已独立核对 ID1..8 全部失能，`automated_result=PASS`。

虽然文件名为 `friction0`，配置快照仍为 J1 静摩擦增益 0.15；trace 中非零重力项对应的
最大摩擦/重力绝对值比也为 0.15，说明该轮并未关闭 J1 静摩擦。保留原始文件，不把它作为关闭摩擦的结果。
J1/J3/J5 在 TELEOP 的跟踪力矩均为零、限位屏障均未介入。

操作者观察：全部零姿态时 J1 不动；稍向后转动 J4，J1/J3 开始联动；某些姿态下 J3/J5 开始联动。
这些观察按姿态相关的转动趋势记录，不能单凭运动方向判断欠补偿或过补偿；本轮没有同步跟踪项
驱动这三轴，也不能据此排除机械耦合、重力残差和人工支撑的影响。

### 11.2 下一轮：仅关闭 J1 静摩擦，重复熟悉动作

已直接修改原左 YAML：`static_friction_comp.gain[0]: 0.15 → 0.0`，原值保留为注释。
重力增益、J2/J4、J3/J5 静摩擦和其它参数均保持现值。重启隔离小臂节点加载，无需编译。
该静摩擦项也用于同步/保持；仍须正常同步成功后再进入 TELEOP。

沿用第 10.1 节隔离启动命令；确认真实大臂命令发布者为 0、隔离话题订阅者为 0。记录改用新文件名：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side left --config "$CONFIG" --operator lyy \
  --output /tmp/left_isolated_j1_friction0_02.json --duration 180
```

按原流程同步至 SYNCED、支撑小臂后切入 TELEOP。本轮只做熟悉的动作：

1. 从零姿态开始，保持能随时扶住小臂；缓慢向后转动一点 J4。
2. J1/J3 出现自行转动趋势就扶住，不追求转大角度或复现所有姿态。
3. 只比较 J1 自转比上一轮减轻、相近或加重，记录是否仍在转动 J4 后出现。

观察结束立即按第 10.4 节 stop，确认 IDLE、退出节点、独立回读失能。
记录文件和回读文件前缀统一改为 `left_isolated_j1_friction0_02`。
不同时修改 J1 重力或 J3/J5 参数；根据该轮实际诊断和现场趋势再决定重力增益的调整方向。

### 11.3 第 02 轮结果：关闭 J1 摩擦未获现场改善

记录 `/tmp/left_isolated_j1_friction0_02.json` 及同名 trace，TELEOP 约 87.30 s。
配置快照确认 J1 静摩擦增益为 0，实际 TELEOP 诊断的 J1 摩擦力矩全程为零；
J1/J3/J5 的跟踪力矩、限位屏障力矩均为零。无诊断丢帧、序号重启或软件电流裁剪，
真实大臂命令计数为 0。停止后首个 IDLE 诊断确认八轴输出归零、缓存 torque 关闭。
操作者确认本轮已完成失能回读；本次尚未按独立回读文件核对寄存器。

现场反馈：“觉得加重了，正向和反向不同”。方向差异具体指 J4 姿态变化方向还是 J1 本身
转动方向，尚待说明。两轮未标记同姿态、同人工支撑条件的对照片段，不以整轮速度峰值或
现场总体差异直接确定重力增益应增还是减，也不把关闭摩擦认定为改善。
当前参数暂保持第 02 轮设置，不继续试运动，先明确方向差异再选下一项调整。

### 11.4 J1 重力增益 0.45 对照（已撤回，不再执行）

操作者认为细微手感对比难以执行，要求恢复重力及摩擦参数。下列步骤仅保留提案历史，
不作为当前操作指令；参数已按第 11.5 节恢复。

操作者补充：J4 运动时 J2 也会随动，难以分别描述；就 J1 而言，感觉正向阻力增大、
反向运动更容易。将该现象记为正反方向手感不对称，不据此直接认定过补偿。

下一轮只将原左 YAML 的 `joint_gravity_comp.gain[0]: 0.5 → 0.45`，原 0.5 基线保留为注释；
J1 静摩擦继续为 0，其它重力增益、摩擦及同步/保持参数均不变。
因此同步和保持 J1 重力增益仍为 0.5；本轮比较进入 TELEOP 后的持续手感，不评价切换瞬间改善。

重启第 10.1 节隔离节点加载参数，无需编译；确认隔离仍成立后，在独立终端开始记录：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side left --config "$CONFIG" --operator lyy \
  --output /tmp/left_isolated_j1_g045_01.json --duration 180
```

仍须先真实反馈同步至 SYNCED，再支撑小臂切入 TELEOP。将整条小臂缓慢摆到熟悉、容易出现
上述手感的姿态，不要求单独控制 J4 或固定 J2；尽量保持相近整体姿态，轻轻往返转动一点 J1。
全程扶住小臂，出现自行加速或难以控制的趋势立即扶稳并 stop。

只比较：正向阻力是否减小、反向是否仍明显更容易、整体自转趋势减轻/相近/加重。
观察结束按第 10.4 节 stop、确认 IDLE、退出节点和独立失能回读，回读文件名为
`/tmp/left_isolated_j1_g045_01_torque_disable.json`。
该轮结果待现场记录和反馈；不直接继续递减增益，也不同时改 J3/J5。

### 11.5 恢复基线，停止按细微手感调参

按操作者要求，原左 YAML 已恢复 J1 遥操重力增益为 0.5、静摩擦增益为 0.15；
0.45 和 0 的试验值均保留为注释。其它轴及同步/保持参数不变。
已经运行的小臂节点须先正常 stop、退出，再以隔离入口重启加载；修改文件不改变运行中的参数。

后续建议先复用同步/保持诊断做量化筛选：在多个安全姿态下，分析稳定保持所需的额外
PD/PID 跟踪力矩、重力项、摩擦项、误差和电流裁剪，定位姿态相关的补偿残差，减少对细微手感的依赖。
外部手扶力和静摩擦也会影响保持力矩，不能将单次保持积分直接当成真实重力误差。
具体姿态采样流程尚未实现或安排上机；当前不继续执行第 11.4 节试验。
