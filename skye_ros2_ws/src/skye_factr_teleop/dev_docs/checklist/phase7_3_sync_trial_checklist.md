# 阶段 7.3：无大臂时的小臂走位与保持试验

设备 A，先左臂后右臂。软件准备完成后按本清单执行真机验证；真机结果待填写。
左臂限位项按操作者决定为 PASS，沿用开发计划 7.2 的验收结论。
本轮使用 `/left_joint_move`，验证与正式同步共用的 Ruckig、收敛和保持控制。
正式大臂反馈、新鲜度与主从映射的真机验收，待有大臂后补做。

## 1. 本轮配置与软件行为

- 设备 A 的 `config/grav_comp_m6_{left,right}.yaml`：八轴 offset 均为 0，夹爪目标为 `0.4 rad`。
- 八轴轨迹上限分别为 `0.1 rad/s`、`0.2 rad/s²`、`1.0 rad/s³`，是首轮小幅试验初值。
- 八个电流上限仍为 `50 mA`；轨迹超时 `10 s`，收敛超时 `3 s`。
- 首轮保留配置中的重力、静摩擦和限位项；零空间和力反馈关闭。记录全部力矩分项。
- 启动节点时 torque 关闭；有效的 `joint_move` 请求会使能八个舵机，并控制全部八轴。
  单关节小步指仅改变一个臂关节的目标，其余臂关节保持采样位置；夹爪仍走到 `0.4 rad`。
- `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED`。`SYNCED` 后持续保持，
  **不会因记录器退出或其计时结束而停止**。新目标须先 stop 回到 IDLE。
- Ruckig 参考曲线的限制不等于实测运动限制；低电流下是否足以支撑和跟踪须现场判断。

## 2. 构建和离线预览（不打开串口）

在主机执行：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_factr_teleop --symlink-install
source install/setup.bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side left --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --start 0 0 0 0 0 0 0 0 --joint-id 1 --delta 0.05
```

这个虚拟起点示例的完整目标为 `[0.05, 0, 0, 0, 0, 0, 0, 0.4]`，
Ruckig 预计耗时 `4.7 s`。`--start` 只用于离线预览，不能与 `--execute` 同用。
这不代表现场起点或碰撞检查结果。

## 3. 现场准备与启动（终端 A）

只启动左侧，确认无其它程序占用本侧串口或发送模式、走位命令。
操作者支撑小臂，确认机械和线缆路径可行，停止终端和断电开关可立即使用。
起点须在配置的安全范围内，八轴基本静止；夹爪初始位置也需确认。
终端 A、B、C 各执行以下公共设置：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
SIDE=left
CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_${SIDE}.yaml"
```

终端 A 先从操作系统枚举串口，查看 by-id 名称及其指向的 `ttyUSB*`，并与当前侧 YAML 中的 `dynamixel_port` 对照：

```bash
ls -l /dev/serial/by-id/
rg -n 'dynamixel_port:' "$CONFIG"
```

列表只能证明系统识别了串口，不能单凭 `ttyUSB*` 编号确定实体侧别。对照设备或线缆标签核实左臂；若有多个同型号串口且仍无法区分，在节点停止时逐个断开并重新枚举，确认哪个 by-id 条目消失。确认实体对应关系后，将 FTDI `latency_timer` 设为 `1` 并回读确认。USB 重新插拔或重启后需重新检查：

```bash
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0
TTY_PATH=$(readlink -f "$USB_BY_ID")
test -c "$TTY_PATH"
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

回读必须为 `1`；右臂验证时将 `USB_BY_ID` 改为设备 A 右臂的 `FTBEO8WL` 路径。

终端 A 启动：

```bash
ros2 launch skye_factr_teleop start_teleop_m6_left.launch.py \
  left_config:="$CONFIG" sync_diagnostics:=true
```

`sync_diagnostics` 默认关闭。本次开启后，在原状态读取中一并读取 Present Current，
每周期向 `/leader_arm/left/sync_diagnostics` 发布一条 JSON `std_msgs/msg/String`。
节点无磁盘写入；发布使用 BEST_EFFORT / VOLATILE，记录器统计序号缺口。

## 4. 开始记录（终端 B）

每轮选择新的文件名，已有文件不会覆盖：

```bash
OUTPUT="$PWD/src/skye_factr_teleop/validation_records/phase7_3_A/${SIDE}_j1_plus_run1.json"
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side "$SIDE" --config "$CONFIG" --operator lyy --output "$OUTPUT"
```

记录器默认最多运行 120 s；看见运动后回到 IDLE 且软件输出归零、缓存 torque 为关闭时自动保存。
Ctrl+C 或计时结束保存未完成记录，均不发送停止命令。
出现 `IDLE` 后再发运动请求，避免遗漏起始状态。

## 5. 小幅目标预览与执行（终端 C）

先预览当前姿态附近 J1 正向 `0.05 rad`（约 2.86°）目标：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" --joint-id 1 --delta 0.05
```

工具从诊断话题读取新鲜八轴位置和速度，要求该侧 IDLE 且缓存 torque 关闭。
检查起点、目标和规划轨迹的关节范围、起始速度，以及距轨迹超时至少 0.25 s 的余量。
输出完整八轴目标与预计耗时。这里只检查数值范围，不检查实体碰撞。

操作者确认打印的目标和路径后执行：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" --joint-id 1 --delta 0.05 --execute
```

执行时重新采样当前位置，因此相对目标可能与先前预览略有不同。
消息自动带 `l_j1..l_j7`、七轴 position 和发布时的 ROS 时间，只发布一次。
没有额外的 `switch_sync` 请求。命令持续等待同步完成，并在状态变化时打印：

```text
Observed mode: TELEOP_SYNCING
Observed mode: HOLD_SETTLING
Observed mode: SYNCED
Synchronization complete: SYNCED. 同步完成；继续保持目标姿态。 Use /mode/switch_stop to stop.
```

`TELEOP_SYNCING` 只表示轨迹执行中；`HOLD_SETTLING` 表示轨迹已结束，正在等待实测
位置和速度收敛。只有观察到 `SYNCED` 才提示同步完成。若提前回到 IDLE、诊断过期或
等待超时，命令以非零状态退出并报告原因；诊断丢帧可能导致中间状态未打印。
`--wait-seconds` 默认为 5 s，用于连接和启动确认；启动后等待上限为配置中的轨迹超时
加收敛超时，再加该余量。工具退出不发送停止命令，完成后仍需按第 6 节停止。
继续观察终端 B 和实机。

进入 `HOLD_SETTLING` 的条件为 Ruckig 返回 `Finished`，表示参考轨迹结束，并不要求
实测关节此时已经到位。轨迹在 `10 s` 内未完成，控制器以 `sync_timeout` 停止。
进入 `SYNCED` 要求八轴同时满足位置和速度容差，并连续保持 `0.25 s`：
J1..J7 的位置误差不大于 `0.03 rad`、速度绝对值不大于 `0.05 rad/s`；夹爪分别为
`0.05 rad`、`0.10 rad/s`。任一轴超差都会重置连续计时；进入收敛阶段后 `3 s` 内
未满足持续条件，以 `settle_timeout` 停止。

等待期间终端每秒打印当前未满足条件的轴；退出失败时向 stderr 报告超时原因、
最新实测位置/最终目标/位置误差及超限速度。即使最后一帧瞬间达标，也不代表已完成
连续保持。诊断新增 `sync_status`，包含控制器实际的超时原因与连续收敛时长；
更新代码后须重启驱动节点才能获得这些字段。工具不会从丢帧数据推断连续收敛成功。
执行失败时还会打印最后一个同步/收敛周期的实测电流、命令电流裁剪前后值、
运行电流上限及各力矩分项，避免回到 IDLE 后的零输出掩盖失败前的状态。
CLIP 样本数统计本次等待期间实际收到的同步/收敛帧，不能代表丢失的诊断帧。
需要观察原七轴状态时，可另开终端运行：

```bash
ros2 topic echo /left_leader_arm/current_state sensor_msgs/msg/JointState
```

### 多关节相对目标示例

小幅多关节试验使用 `--deltas`，按 J1..J7 顺序填写七个相对增量，单位 rad。
工具从同一帧实测位置计算“当前位置 + 增量”；`0` 表示保留该轴的采样位置，
正负值分别表示沿逻辑关节正负方向移动。不能与 `--target`、`--joint-id` 或
`--delta` 同用。

以下示例让 J4、J6 分别从当前姿态正向移动 `0.05 rad`，其余臂关节保持采样位置，
不要求起点为零位。

先预览：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" \
  --deltas 0 0 0 0.05 0 0.05 0
```

确认完整目标、预计耗时和实体路径后执行：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" \
  --deltas 0 0 0 0.05 0 0.05 0 \
  --execute
```

七个数依次对应 `J1 J2 J3 J4 J5 J6 J7`，不包含夹爪；夹爪仍使用配置中的
`gripper_setpoint`，本轮为 `0.4 rad`。执行时重新采样，因此相对目标可能与预览
略有不同；每次执行都会再次叠加增量。每轮须从 IDLE 开始，完成后的观察、停止和
失能回读沿用第 6 节。

多关节相对试验用于检查同时运动时的动力学耦合、轨迹跟踪、电流限幅和收敛表现。
从已验证的小幅双关节组合开始，再逐步增加参与关节和姿态；保留逐帧记录。

需要复现固定姿态时仍使用 `--target`，七个数均为绝对位置。未参与运动的轴也须
填入当前实测位置；`--target` 中的 `0` 会使该轴向零位运动。

### 三个终端实时查看力矩分项

驱动保持 `sync_diagnostics:=true`。三个新终端各执行第 3 节的公共环境设置，
然后分别执行以下命令（只订阅诊断消息，不打开串口，也不发送控制命令）：

```bash
# 终端 D：重力补偿力矩
ros2 run skye_factr_teleop skye_factr_sync_trial monitor --side "$SIDE" --term gravity

# 终端 E：摩擦力补偿力矩
ros2 run skye_factr_teleop skye_factr_sync_trial monitor --side "$SIDE" --term friction

# 终端 F：限位屏障力矩
ros2 run skye_factr_teleop skye_factr_sync_trial monitor --side "$SIDE" --term limit
```

每个终端原地刷新 J1..J7 和夹爪的瞬时逻辑力矩，单位 N·m，同时显示模式和诊断序号。
默认每秒刷新 5 次，可用 `--rate 10` 调整。显示的是电流裁剪前的控制分项，
不是实测力矩。无新消息超过 1 s 时显示 STALE，并隐藏旧数值；IDLE 时这些分项为零。
Ctrl+C 只退出监视器，不停止驱动。无需传入 `--config`。

### 一个终端实时查看实测电流和限幅

新终端执行第 3 节的公共环境设置后运行：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial monitor --side "$SIDE" --term current
```

按物理舵机 ID 1..8 显示实测电流、`current_limit_ma`、实测电流绝对值占上限比例条、
命令电流裁剪前后值，以及“当前帧 / 刷新间隔”的 CLIP 标记；单位均为 mA。
上限由运行节点直接发布，取该节点的配置值与型号允许上限的较小值。
更新代码后须重启驱动节点，并保持 `sync_diagnostics:=true`；监视器无需传入配置。

CLIP 按 `abs(current_before_clip_ma - current_after_clip_ma) > 1e-9` 判断。
实测电流接近或超过上限与软件命令被裁剪是两个独立观测，不互相替代。
“刷新间隔”标记保留该间隔内收到的任一裁剪帧，下一次刷新后重新统计；
诊断采用 BEST_EFFORT，该标记只覆盖实际收到的数据。默认刷新 5 Hz，
可添加 `--rate 10`。诊断过期时显示 STALE，并隐藏旧数值。

## 6. 每轮观察、停止和失能回读

1. 跟踪方向正确，运动平顺，无机械或线缆干涉；进入 HOLD_SETTLING 时无明显冲击。
2. 达到 SYNCED 后保持观察至少 5 s，位置和速度维持在配置的同步容差内；
   七轴位置容差为 `0.03 rad`，夹爪 `0.05 rad`；速度容差分别 `0.05` 和 `0.1 rad/s`。
3. 发生突动、持续振荡、明显下落或超时时停止本轮，保留记录后分析原因。
4. 正常观察结束或需要提前停止时，终端 C 执行：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

停止会失能，操作者继续支撑小臂。确认记录器显示 IDLE 并保存；终端 A 按 Ctrl+C
正常退出，待串口释放后，保持供电、不要重启总线，用现有只读工具回读：

```bash
READBACK="$PWD/src/skye_factr_teleop/validation_records/phase7_3_A/${SIDE}_j1_plus_run1_torque_disable.json"
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side "$SIDE" --config "$CONFIG" --frames 2 --interval 0 --output "$READBACK"
```

核对回读 `automated_result=PASS`、ID 1..8 的 `torque_enabled=false`。
回读文件与本轮 JSON 同名关联，另记现场方向、平顺性、支撑感受、保持时间和结束方式。
软件缓存中的关闭状态不能代替这个寄存器回读。

## 7. 如何读取数据与调参

输出为本轮 `.json` 汇总与 `.trace.jsonl` 逐帧记录；汇总保存本轮配置内容、侧别和操作者。
节点与工具必须使用同一配置文件，修改参数后重启节点并开始新记录。

| 记录字段 | 含义 |
|---|---|
| `position_rad` / `velocity_rad_s` | 本周期实测八轴状态 |
| `target_position_rad` / `target_velocity_rad_s` | 本周期参考，IDLE 时为 null |
| `*_torque_nm` | 重力、跟踪、静摩擦、限位、零空间及总逻辑指令力矩；不是实测关节力矩 |
| `current_before_clip_ma` / `current_after_clip_ma` | 物理 ID 1..8 顺序的裁剪前后命令电流；裁剪后值尚未包含舵机寄存器量化 |
| `present_current_ma` | 与位置、速度同次总线读取的电流；采样发生在本周期写电流之前 |
| `cycle_dt_s` | 相邻控制周期开始时间差；500 Hz 名义值为 0.002 s |
| `wrote_current` | 本周期是否执行了电流写入；IDLE 为 false |
| `torque_enabled_cached` | 驱动软件缓存状态，不是逐周期寄存器回读 |
| `phases` | 各阶段最大误差、最大速度、电流峰值、逐轴裁剪次数和周期统计 |
| `transitions` | 状态切换前后电流变化和序号间隔；有丢帧时不能当作相邻周期差值 |
| `missing_samples` / `sequence_restarts` | 诊断丢帧和节点序号重启；逐帧证据不完整时复测受影响检查 |
| `follower_command_count` | 记录期间该侧大臂命令数，本轮应为 0 |

`RECORDED` 仅表示观察到运动及软件停止，不等于硬件 PASS；到时或中断未见停止为 INCOMPLETE。
只有人工观察、轨迹/保持数据、无 follower 命令和独立失能回读均满足要求后再记实机 PASS。
轨迹阶段的误差上限先按首轮数据评估；不得把“进入 SYNCED”当作整段轨迹跟踪合格。

调参顺序：先确认电流是否持续裁剪、周期是否明显偏离 2 ms；再结合重力和跟踪分项
调整轨迹阶段的重力增益与 PD，以及保持阶段的重力增益与 PID。
保持不动可能由 PID 抵消重力残差；静摩擦项也参与总输出，需一起核对。
更改任一增益后，复测该阶段及与相邻阶段的切换；不因跟不上而直接提高电流上限。

## 8. 扩展姿态和右臂

按现场可行方向逐个关节测试小幅正负目标，每轮 stop 后再发新目标。
相对负步是相对本轮新采样姿态，不保证精确回到上一轮起点。
如需复现固定姿态，用 `--target q1 q2 q3 q4 q5 q6 q7`。
多个姿态通过后才考虑 `--target 0 0 0 0 0 0 0`；过长轨迹会被预览拒绝。

左臂通过后按相同步骤做右臂：所有终端改为 `SIDE=right` 并重新设置 CONFIG，
启动 `start_teleop_m6_right.launch.py right_config:="$CONFIG" sync_diagnostics:=true`。
工具自动使用 `/right_joint_move`、`r_j1..r_j7` 和右侧诊断话题。

超时和中途 stop 的软件路径由无硬件测试覆盖；现场补做小幅运动中 stop，
超时实测应使用有界小目标与较短的独立试验配置，不通过卡住或阻挡机构制造超时。
待大臂可用，再验证真实 follower 同步及后续遥操资格。

## 9. 软件准备验证记录（2026-10-16）

- 主机 `/usr/bin/python3` 的相关无硬件测试共 374 项通过：配置、launch、控制核心、
  状态机、Ruckig/保持、ROS 驱动、新试验工具及真实 Pinocchio 动力学。
- 左右两侧均通过真实 ROS DDS 联调：假硬件下运行独立的预览、执行和记录进程，
  验证预览不使能、消息被接受、达到 SYNCED、stop 后失能，以及记录文件落盘。
  联调使用假轨迹和注入的实测状态，验证通信与软件流程；真实 Ruckig 另做离线检查。
- `colcon build --packages-select skye_factr_teleop --symlink-install` 通过。
- 安装后的 `move --help`、`record --help` 及左右两侧离线示例通过，预计时长均为 4.7 s。
- 此记录不包含真实串口读写、实体运动、500 Hz 真机周期或同步硬件 PASS。

相关测试复现命令（功能包目录）：

```bash
/usr/bin/python3 -m pytest -q \
  test/test_ros_driver.py test/test_launch.py test/test_sync_trial.py \
  test/test_config.py test/test_control_core.py test/test_sync_hold.py \
  test/test_state_machine.py test/test_state_trajectory_integration.py test/test_dynamics.py
```
