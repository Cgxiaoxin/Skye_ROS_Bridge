# 阶段 7.3：无大臂时的小臂走位与保持试验

配置迁移说明（2026-10-29）：下文历史快照保留当时的 `sync_hold_control.gripper_setpoint`。
用于当前程序前，应将其移到 `controller.limit_barrier_control.gripper.gripper_setpoint`，
放在 `limit_min` 与 `kp` 之间；历史 JSON 记录保持原样。

2026-10-23 补充：操作者确认无大臂同步验收已完成。当前保留参数以第 11、12 节为准：
轨迹上限 `0.8 rad/s、1.0 rad/s²、5.0 rad/s³`。实际采用第 12 节的 `/tmp/` 路径、
`record --duration 180` 及 `move --target 0 0 0 0 0 0 0 --execute` 终端输出进行分析；
第 4 节的 `validation_records/...j1_plus_run1.json` 是早期示例，未实际采用。
关于此前 stop 延迟，操作者补充认为可能是记录命令占用终端、后续 stop 尚未执行；
后续记录器与同步/停止命令使用独立终端，本轮不单独排查 stop 实现。
正式大臂反馈同步继续使用[连接大臂后的单臂同步清单](phase7_3_follower_sync_checklist.md)。
下文保留各次试验的历史范围和结论。

设备 A，先左臂后右臂。2026-10-21 已补充左臂归零同步、重复试验、保持与独立失能回读记录，见第 10 节；
扩展姿态、右臂及完整真机验收项目仍待完成。
左臂限位项按操作者决定为 PASS，沿用开发计划 7.2 的验收结论。
本轮使用 `/left_joint_move`，验证与正式同步共用的 Ruckig、收敛和保持控制。
正式大臂反馈、新鲜度与主从映射的真机验收，待有大臂后补做。

## 1. 本轮配置与软件行为

- 设备 A 的 `config/grav_comp_m6_{left,right}.yaml`：八轴 offset 均为 0，夹爪目标为 `0.4 rad`。
- 八轴轨迹上限分别为 `0.1 rad/s`、`0.2 rad/s²`、`1.0 rad/s³`，是首轮小幅试验初值。
- 当前左臂电流上限按物理 ID 1..8 为 `[300, 300, 100, 200, 50, 50, 50, 50] mA`；
  轨迹超时 `30 s`，收敛超时 `25 s`。本次左臂参数快照见第 10 节；右臂以各自 YAML 为准。
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
实测关节此时已经到位。当前左臂轨迹在 `30 s` 内未完成，控制器以 `sync_timeout` 停止。
进入 `SYNCED` 要求八轴同时满足位置和速度容差，并连续保持 `0.25 s`：
当前左臂 J1..J7 与夹爪的位置误差均不大于 `0.05 rad`、速度绝对值均不大于 `0.05 rad/s`。
任一轴超差都会重置连续计时；进入收敛阶段后 `25 s` 内未满足持续条件，以 `settle_timeout` 停止。

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
   当前左臂八轴位置容差均为 `0.05 rad`，速度容差均为 `0.05 rad/s`。
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
第 10 节已记录指定起点附近的 `--target 0 0 0 0 0 0 0` 归零试验，尚未覆盖其他起点和多姿态。
扩展起点时先逐步验证小幅目标，再安排归零；过长轨迹会被预览拒绝。

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

## 10. 左臂归零同步与保持真机记录（2026-10-21）

### 10.1 范围与结论

设备 A（Thor），操作者 `lyy`，仅左臂；使用原始
`config/grav_comp_m6_left.yaml`，通过 `/left_joint_move` 执行七个臂关节绝对零位目标，
夹爪目标为 `0.4 rad`。以下三轮最终记录的完整配置快照一致，均观察到
`IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`。

- [x] 指定起点附近的归零同步：三轮均在配置的收敛超时内达到连续达标判据。
- [x] 同参数重复归零：前两轮分别在进入保持阶段后约 `17.60 s`、`21.95 s` 达到 `SYNCED`。
- [x] 接触时间标记前的保持数据：第三轮记录 `56.95 s`；最初两秒后约 `54.95 s`，
  八轴位置与速度均在容差内，无命令电流限幅。
- [x] 三轮诊断均无丢帧、无序号重启，`follower_command_count=0`；停止后均观察到
  `IDLE`、软件输出归零及缓存 torque 关闭。
- [x] 补做独立失能寄存器回读：`2026-10-21 15:10:52 +08:00` 的只读检查为
  `automated_result=PASS`，物理 ID 1..8 的 `torque_enabled` 全部为 `false`。

当前参数保留为左臂该归零目标的同步与保持基线。接触标记前实际记录为 `56.95 s`，
不记作完整的 60 秒验证；`RECORDED` 与上述软件停止证据也不代替第 6 节的独立失能寄存器回读。
本轮尚未形成阶段 7.3 的完整硬件 PASS，未覆盖扩展姿态、右臂或正式大臂反馈与遥操资格。

### 10.2 最终参数快照

来源为三轮记录中的 `config` 快照，与当前左臂 YAML 一致。
电流按物理 ID 1..8；控制向量按逻辑 J1..J7、夹爪，重力增益仅含七个臂关节。
八轴 offset 均为 0，`joint_signs=[1, 1, 1, 1, 1, -1, -1, 1]`，控制名义频率为 `500 Hz`。
重力模式为 `position_only`；重力、静摩擦、限位项开启，零空间及力反馈关闭。

```yaml
dynamixel:
  current_limit_ma: [300.0, 300.0, 100.0, 200.0, 50.0, 50.0, 50.0, 50.0]

controller:
  sync_hold_control:
    gripper_setpoint: 0.4
    ruckig:
      gravity_comp_gain: [0.5, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0]
      kp: [0.35, 0.35, 0.35, 0.45, 0.35, 0.35, 0.35, 0.35]
      kd: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
      max_velocity: [0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10, 0.10]
      max_acceleration: [0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20, 0.20]
      max_jerk: [1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00]
      trajectory_timeout: 30.0
    hold:
      gravity_comp_gain: [0.5, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0]
      kp: [0.35, 0.35, 0.35, 0.45, 0.35, 0.35, 0.35, 0.35]
      kd: [0.15, 0.15, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
      ki: [0.02, 0.00, 0.15, 0.15, 0.15, 0.15, 0.15, 0.15]
      i_max: [0.60, 1.60, 0.45, 0.80, 0.10, 0.10, 0.10, 0.10]
      i_thresh: [0.20, 1.00, 0.15, 0.25, 0.15, 0.15, 0.15, 0.15]
      sync_position_tolerance: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
      sync_velocity_tolerance: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
      settle_duration: 0.25
      settle_timeout: 25.0
```

`i_max` 的单位为 rad·s，积分力矩上限为 `ki × i_max`；`i_thresh` 的单位为 rad，
误差超过阈值时该轴积分清零。最终 J1 最大积分力矩为 `0.012 N·m`，J2 积分关闭。
同步与保持阶段使用各自的重力增益，普通模式的 `joint_gravity_comp.gain` 未随之修改。

### 10.3 最终三轮数据与证据路径

证据已归档到 [20261021_left_zero_hold](../../validation_records/phase7_3_A/20261021_left_zero_hold/)。

| 汇总文件 | 进入保持后至 SYNCED | 同步后记录时长 | 主要结果 |
|---|---:|---:|---|
| [sync_j1_imax060_01.json](../../validation_records/phase7_3_A/20261021_left_zero_hold/sync_j1_imax060_01.json) | 17.60 s | 47.52 s | 无电流限幅；同步后有 J1 短暂位置超差、J4 短暂速度超差；最后 30 s 八轴位置与速度均达标 |
| [sync_j1_imax060_repeat02.json](../../validation_records/phase7_3_A/20261021_left_zero_hold/sync_j1_imax060_repeat02.json) | 21.95 s | 42.42 s | 位置误差始终在容差内；存在 J3/J4 短暂速度超差及 J7 电流限幅，停机前是否有接触记不清 |
| [sync_hold60_untouched_01.json](../../validation_records/phase7_3_A/20261021_left_zero_hold/sync_hold60_untouched_01.json) | 20.19 s | 64.42 s | 接触标记前 56.95 s 无速度超差、无电流限幅；最初两秒后约 54.95 s 八轴位置与速度均达标 |

每份汇总均配有同目录、同名的 `.trace.jsonl.gz` 逐帧文件，以 gzip 无损压缩保存。
汇总 JSON 保留采集时 `/tmp/` 下的 `trace_path`，归档分析时使用同目录同名的压缩文件。
第三轮接触标记为 [sync_hold60_touch_marker.txt](../../validation_records/phase7_3_A/20261021_left_zero_hold/sync_hold60_touch_marker.txt)，
记录主机 `time.monotonic()`；分析截取
`SYNCED` 且 `monotonic_s < 接触标记` 的样本，标记之后的数据不计入无接触保持结论。

独立失能回读为 [left_phase73_torque_disable_01.json](../../validation_records/phase7_3_A/20261021_left_zero_hold/left_phase73_torque_disable_01.json)。
回读识别到 ID1..2 为 XM430-W350、ID3..8 为 XC330-T288，八轴 `torque_enabled=false`，
`automated_result=PASS`，`error=null`。该文件补充本次停止后的硬件状态证据；
不追溯为前两轮各自的寄存器回读。三份原始试验汇总中的 `hardware_torque_disable_verified=false`
保留采集时状态，未回写；独立证据以本回读文件为准。回读中的人工检查项仍为 `PENDING`。

第三轮接触标记前，八轴均无速度超差、无命令电流限幅。仅 J1 在同步后的最初两秒内有
17 个位置超差样本，最大误差 `0.05062 rad`；不将这段样本计作全程位置达标。
排除最初两秒后的统计为：

| 轴 | 最大位置误差（rad） | 最大速度绝对值（rad/s） |
|---|---:|---:|
| J1 | 0.04449 | 0.02398 |
| J2 | 0.04295 | 0.00000 |
| J3 | 0.02915 | 0.02398 |
| J4 | 0.00460 | 0.04796 |
| J5 | 0.00460 | 0.02398 |
| J6 | 0.04142 | 0.02398 |
| J7 | 0.00153 | 0.02398 |
| 夹爪 | 0.02724 | 0.04796 |

该无接触区间未复现此前的 J4 速度超差与 J7 限流，当前保留基线参数。
先前记录缺少接触标记，不能据此断定先前所有超差均由人工接触引起。

### 10.4 调参结论与后续复现

- 相近起点仍可能产生不同的收敛过程；先稳定 J1/J2，再复查其余轴，单轮达标不作为可重复通过结论。
- J4、J3 早期分别出现电流限幅与积分到顶。提高电流上限只解除允许输出的限制，
  不会使控制器自动请求该上限，也不能单独消除静态误差。
- J1 重力增益由 `0.4` 调至 `0.5` 后能接近零位；随后启用近目标小积分，
  并将 `i_max` 从 `1.0` 降至 `0.6`，限制越零前的正向积分蓄积。
  当前组合的三轮记录均达到 SYNCED，收敛时间仍有差异。
- `settle_timeout` 从 `20 s` 调为 `25 s`，未改变位置、速度容差或 `0.25 s` 连续达标要求。
- 进入 SYNCED 只证明当时连续达标；保持阶段须继续检查逐帧数据。
  参考轨迹 `0.10 rad/s` 上限不保证实际速度受同一硬限制，整段轨迹跟踪平顺性仍需单独评估。

后续复现沿用第 3 节环境及启动命令，节点和工具使用同一原始左臂 YAML。
先开始记录，再执行与本次相同的绝对目标：

```bash
# 示例文件名须换成未使用的新名称；时长到期不发送停止命令。
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side "$SIDE" --config "$CONFIG" --operator lyy \
  --output /tmp/sync_zero_hold_next01.json --duration 180
```

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" \
  --target 0 0 0 0 0 0 0 --execute
```

进入 SYNCED 后观察保持过程；做无接触保持时，准备扶持前先写入时间标记，再支撑并停止：

```bash
python3 -c 'import time; print(time.monotonic())' \
  > /tmp/sync_zero_hold_next01_touch_marker.txt
```

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

出现异常时直接停止，现场处置优先于写标记。记录器保存后，按第 6 节另做失能寄存器回读。

### 10.5 验收项目状态

- [x] 三轮汇总、对应逐帧压缩文件及接触标记已归档，并关联独立失能寄存器回读。
- [x] 本次停止后八轴失能寄存器回读为 PASS；三份原始试验汇总中的采集时标记保持原值，见第 10.3 节。
- [ ] 补齐现场方向、支撑感受、机械与线缆干涉等人工检查结果，以及整段轨迹跟踪和平顺性评价。
- [ ] 单独评估实际周期与参考轨迹时基：三轮轨迹阶段 `cycle_dt_max_s` 约为 `18.91～19.96 ms`，
  不能将名义 `500 Hz` 记为每周期严格达到 `2 ms` 的真机验收结论。
- [ ] 扩展起点、各轴正负小步、多姿态及多关节组合验证。
- [ ] 现场小幅运动中 stop、受控超时与右臂验证。
- [ ] 有正式大臂后验证 follower 反馈、新鲜度、主从映射与遥操资格。

## 11. 左臂快速归零同步与阻尼回退验证（2026-10-21）

### 11.1 与第 10 节慢速记录的区别

本节记录同日下午继续调参后的结果，设备、操作者和目标沿用第 10 节：设备 A（Thor）、
`lyy`、仅左臂，七个臂关节绝对目标全为 `0 rad`，夹爪目标为 `0.4 rad`。
目标由最初的严格 5 s 调整为“约 5 s、不同起点均能平稳同步”，不再为几百毫秒继续增加增益。

**第 10 节保留为慢速历史参数与验收记录；当前快速同步基线以本节为准。**
第 10 节中的“最终参数”“与当前 YAML 一致”“当前保留基线”等描述，是该阶段采集时的状态，
不表示仍与本节之后的左臂 YAML 一致。两套参数不可混用；旧阶段的保持与失能回读结果也不自动转移到新参数。

| 项目 | 第 10 节慢速历史参数 | 本节快速同步参数 |
|---|---|---|
| 八轴参考速度上限 | 0.10 rad/s | 0.80 rad/s |
| 八轴参考加速度上限 | 0.20 rad/s² | 1.00 rad/s² |
| 八轴参考 jerk 上限 | 1.00 rad/s³ | 5.00 rad/s³ |
| 物理 ID3 电流上限 | 100 mA | 120 mA，其余 ID 不变 |
| 轨迹与保持增益 | 见第 10.2 节 | 见第 11.2 节，不只更改轨迹速度 |
| J3 保持积分 `ki / i_max` | 0.15 / 0.45，积分力矩上限 0.0675 N·m | 0.60 / 0.125，积分力矩上限 0.075 N·m |
| 代表性实测耗时 | 原第三轮约 18.10 s 轨迹 + 20.19 s 收敛，总计 38.29 s | 最终三轮总计 5.65 / 4.98 / 4.89 s |

两阶段的重力增益仍为 `[0.5, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0]`。
八轴位置容差 `0.05 rad`、速度容差 `0.05 rad/s`、连续达标 `0.25 s`、轨迹超时 `30 s`、
收敛超时 `25 s` 均未放宽。名义频率仍为 `500 Hz`；本节不将其记为严格的真机周期保证。
重力模式、offset、方向符号、静摩擦和限位配置沿用当前左臂配置，零空间及力反馈仍关闭。

### 11.2 当前快速同步参数快照

以下为最终三轮记录共同的配置快照，与本次整理时的 `config/grav_comp_m6_left.yaml` 一致。
电流按物理 ID1..8；其余八轴向量按逻辑 J1..J7、夹爪。

```yaml
dynamixel:
  current_limit_ma: [300.0, 300.0, 120.0, 200.0, 50.0, 50.0, 50.0, 50.0]

controller:
  sync_hold_control:
    gripper_setpoint: 0.4
    ruckig:
      gravity_comp_gain: [0.5, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0]
      kp: [0.50, 0.50, 0.80, 0.50, 0.35, 0.35, 0.35, 0.35]
      kd: [0.08, 0.08, 0.10, 0.08, 0.05, 0.05, 0.05, 0.05]
      max_velocity: [0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80, 0.80]
      max_acceleration: [1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00, 1.00]
      max_jerk: [5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00, 5.00]
      trajectory_timeout: 30.0
    hold:
      gravity_comp_gain: [0.5, 0.2, 0.7, 0.6, 1.0, 0.8, 1.0]
      kp: [0.50, 0.35, 0.80, 0.80, 0.35, 0.50, 0.35, 0.35]
      kd: [0.15, 0.15, 0.05, 0.08, 0.05, 0.05, 0.05, 0.05]
      ki: [0.02, 0.00, 0.60, 0.15, 0.15, 0.15, 0.15, 0.15]
      i_max: [0.6, 1.6, 0.125, 0.80, 0.10, 0.10, 0.10, 0.10]
      i_thresh: [0.20, 1.00, 0.15, 0.25, 0.15, 0.15, 0.15, 0.15]
      sync_position_tolerance: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
      sync_velocity_tolerance: [0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]
      settle_duration: 0.25
      settle_timeout: 25.0
```

`i_max` 为位置误差积分上限，单位 rad·s。中途将 J3 的 `ki / i_max` 从 `0.15 / 0.50`
依次调为 `0.30 / 0.25`、`0.60 / 0.125`，使同样误差下的积分力矩建立更快，
同时保持这些步骤的积分力矩上限为 `0.075 N·m`；这不等于整机收敛速度按相同比例提高。
J2 的 `ki=0`，其 `i_max` 不产生积分力矩。同步与保持仍使用各自的增益。

未采用此前仅作候选建议的 `max_velocity=1.00`、`max_acceleration=1.25`、`max_jerk=6.25`；
本节最终三轮实际使用的是 `0.80 / 1.00 / 5.00`。

### 11.3 抖动现象、回退措施与结论

调试中曾将保持阶段 J3、J5 的 `kd` 分别增至 `0.15`、`0.10`。在某一不同起点归零时，
现场观察到零位附近持续明显抖动，25 s 后收敛超时。终端显示 J3 速度约达 `1.15 rad/s`，
J3 命令裁剪样本为 `4698 / 13969`，约 `33.6%`；该失败轮未开启 record，只有终端输出，
没有逐帧证据可用于确定振荡频率及各控制分项的相位关系。

代码直接使用实测速度计算 D 项，没有速度滤波；按当时 J3 的 `kd=0.15` 和上述速度估算，
仅 D 项即对应约 `152 mA`，大于其 `120 mA` 电流上限。
这说明不能把速度超差一律理解为“阻尼不足”，也不能靠继续增加 Kd 或电流上限处理该抖动。
速度反馈增益、反馈延迟、积分和关节耦合的具体贡献尚未分别定位。

本轮仅将保持阶段 **J3、J5 的 Kd 同时回退为 `0.05`**，其余参数保持当时值，
随后进行了第 11.4 节三次不同起点测试。三轮均同步成功，操作者目测无明显抖动。
这一对照支持“此前增大的速度反馈增益是抖动的重要诱因”，但因两轴同时回退、失败轮无逐帧记录，
不将其记为已唯一确定根因，也不外推为任意起点的稳定性保证。

### 11.4 最终三轮不同起点的实测结果

时间统一取驱动诊断中首次 `TELEOP_SYNCING` 到首次 `SYNCED` 的单调时钟差，
包含 `0.25 s` 连续达标时间，不包含 CLI 启动、预览和连接等待时间。
轨迹与收敛时长分别按状态切换时间计算，而非逐秒终端打印次数。

以下文件当前位于本机 `/tmp/`，尚未像第 10 节证据那样归档到仓库；
每份汇总均有同目录同名的 `.trace.jsonl`，完整八轴起点和逐帧数据以该文件为准。
表中的起点取驱动首个轨迹周期，可能与 CLI 预览采样有少量差异。

| 汇总文件 | 起点 J3 / J5 / 夹爪（rad） | 轨迹执行 | 到位收敛 | 总同步时间 |
|---|---|---:|---:|---:|
| [sync_stability_kdrollback_01.json](/tmp/sync_stability_kdrollback_01.json) | -0.0721 / 0.1488 / 0.1887 | 3.154 s | 2.492 s | **5.646 s** |
| [sync_stability_kdrollback_02.json](/tmp/sync_stability_kdrollback_02.json) | -0.7148 / 1.0784 / 0.3651 | 3.079 s | 1.897 s | **4.976 s** |
| [sync_stability_kdrollback_03.json](/tmp/sync_stability_kdrollback_03.json) | 0.0828 / 0.5430 / 0.3651 | 3.071 s | 1.815 s | **4.887 s** |

三轮共同结果：

- [x] 同一套快速参数、三组不同起点均观察到 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`。
- [x] 操作者目测三轮均无明显抖动；同步总时间约 `4.89～5.65 s`，符合本次“约 5 s”的调试目标，
  不记为三轮均严格小于 5 s。
- [x] 三轮 `missing_samples=0`、`sequence_restarts=0`、`follower_command_count=0`，汇总均为 `RECORDED`。
- [x] 三轮轨迹、收敛和所记录的 SYNCED 区间，八轴命令电流裁剪样本均为 0。
- [x] 三轮均观察到停止后的 IDLE、软件输出归零及缓存 torque 关闭。

三轮 SYNCED 后分别记录约 `3.97 / 4.23 / 4.94 s`，不构成本参数下的 60 s 无接触保持验证。
三份汇总的 `hardware_torque_disable_verified` 均仍为 `false`，本轮没有新增独立失能寄存器回读。
此前还观察过一次 stop 未立即生效、第二次发送后才停止的现象，目前未定位或修复；
这三轮的软件停止结果不作为单次 stop 可靠性已经验收的结论。该问题按本轮约定暂时搁置。

### 11.5 当前保留基线与复现

保留第 11.2 节参数，当前不再为缩短几百毫秒继续提速或增加阻尼。
使用第 3 节的环境及启动方法，确保节点和工具指向当前左臂 YAML，
驱动开启 `sync_diagnostics:=true`；修改 YAML 后重启驱动节点使参数生效。

```bash
# 使用未占用的新文件名；到期只保存记录，不发送停止命令。
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side "$SIDE" --config "$CONFIG" --operator lyy \
  --output /tmp/sync_fast_baseline_next01.json --duration 180
```

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side "$SIDE" --config "$CONFIG" \
  --target 0 0 0 0 0 0 0 --execute
```

同步后的观察、支撑与停止沿用第 6 节；若出现明显抖动，应立即停止并保留记录，
不等待收敛超时。本节覆盖三组起点的归零同步，不将第 10.5 节其余未完成的阶段验收项目标为完成。

## 12. 右臂重标零后的归零同步基线与三轮复测（2026-10-21）

### 12.1 设备、零点与参数版本的区分

本节仅记录设备 A（Thor）右臂，操作者 `lyy`。七个臂关节的绝对目标为 `0 rad`，
夹爪目标为 `0.4 rad`，使用 `config/grav_comp_m6_right.yaml`。
第 11 节继续作为左臂快速同步基线，本节不修改或替代左臂的参数及实测结论。

右臂调参期间，操作者重新标定了舵机零点。早期出现过 J1 负向偏移、J3 不到位，
并试调过 J1 轨迹及保持增益、J2 保持增益，以及将 ID3 电流上限提高到 `160 mA`。
重标零后，这些过渡参数曾实现同步，但部分起点仍出现轨迹阶段抖动。
因此，本轮先把右臂的同步控制参数与电流上限完整恢复到第 11.2 节左臂稳定基线，
再以新零点重新建立右臂参数基线。此前过渡试验不与本节最终三轮混为同一配置，
也不据此认定早期全部异常均由零点偏差造成。

恢复基线只涉及同步控制参数和电流上限。右臂串口仍为
`usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0`，
方向符号仍为 `[1, -1, 1, -1, 1, 1, 1, 1]`，动力学模型仍为
`urdf/m6right_isomorphism.urdf`；右臂限位等其余配置保留。
YAML 中八轴 `joint_offsets` 仍为 `0`，本轮未在软件中额外叠加 offset。

### 12.2 新零点下的基线问题与唯一保留调整

完整恢复左臂基线后的首轮记录为
[right_sync_rezero_leftbaseline_01.json](/tmp/right_sync_rezero_leftbaseline_01.json)，
对应逐帧文件为同目录同名 `.trace.jsonl`。
该轮最后仅剩 J2 长期停在约 `+0.060 rad`，超过 `0.05 rad` 位置容差，
25 s 收敛超时后进入 IDLE；轨迹及收敛阶段八轴均无命令电流裁剪。

基线 J2 保持 `ki=0`，没有用于逐步消除该静差的积分项。
本轮只把右臂 J2 保持 `ki` 从 `0.00` 改为 `0.05`，随后连续三轮同步成功。
这支持在本轮工况下保留该小积分，不将单次比较记为静差来源已被唯一定位。

**当前右臂同步控制参数及电流上限相对第 11.2 节左臂基线，唯一差异是 J2 保持 Ki。**
“唯一差异”仅指上述控制参数范围，不表示左右臂的硬件方向、模型和限位配置相同。

| 参数 | 第 11 节左臂稳定基线 | 本节右臂保留值 |
|---|---|---|
| J2 保持 `kp` | 0.35 | 0.35，不保留过渡试验的 0.60 |
| J2 保持 `kd` | 0.15 | 0.15 |
| **J2 保持 `ki`** | **0.00** | **0.05，右臂独立调整** |
| J2 保持 `i_max / i_thresh` | 1.6 / 1.0 | 1.6 / 1.0 |
| J1 轨迹 `kp / kd` | 0.50 / 0.08 | 0.50 / 0.08，已回到左臂基线 |
| J1 保持 `kp / kd / ki` | 0.50 / 0.15 / 0.02 | 0.50 / 0.15 / 0.02 |
| 物理 ID3 电流上限 | 120 mA | 120 mA，不保留过渡试验的 160 mA |
| 八轴参考速度 / 加速度 / jerk 上限 | 0.80 / 1.00 / 5.00 | 与左臂相同 |

右臂保留的积分向量为：

```yaml
controller:
  sync_hold_control:
    hold:
      # 逻辑顺序 J1..J7、夹爪；仅第 2 项与左臂不同。
      ki: [0.02, 0.05, 0.60, 0.15, 0.15, 0.15, 0.15, 0.15]
```

其余完整同步参数见第 11.2 节。右臂 YAML 的 `hold.ki` 上方也已注明此差异及用途。
J2 积分力矩上限为 `0.05 × 1.6 = 0.08 N·m`；这不是保证能输出该力矩的实测结果。
八轴电流上限保留 `[300, 300, 120, 200, 50, 50, 50, 50] mA`。
位置、速度容差仍分别为 `0.05 rad`、`0.05 rad/s`，连续达标时间仍为 `0.25 s`，
轨迹及收敛超时仍为 `30 s / 25 s`，没有为达标放宽判据。

### 12.3 最终三轮不同起点的记录结果

以下三份记录的同步配置与当前右臂 YAML 一致。文件目前保存在本机 `/tmp/`，尚未归档到仓库；
每份汇总对应同目录同名 `.trace.jsonl`，完整八轴起点及逐帧数据以该文件为准。
起点取首个轨迹周期，时长按驱动首次进入各状态的单调时钟差计算，包含 `0.25 s` 连续达标时间，
不包含 CLI 启动、预览及连接等待时间。

| 汇总文件 | 起点 J3 / J5（rad） | 轨迹执行 | 到位收敛 | 总同步时间 |
|---|---|---:|---:|---:|
| [right_sync_leftbaseline_j2ki005_01.json](/tmp/right_sync_leftbaseline_j2ki005_01.json) | 0.0568 / -0.2148 | 3.016 s | 4.655 s | **7.671 s** |
| [right_sync_leftbaseline_j2ki005_02.json](/tmp/right_sync_leftbaseline_j2ki005_02.json) | 0.1457 / 0.3513 | 3.116 s | 2.402 s | **5.518 s** |
| [right_sync_leftbaseline_j2ki005_03.json](/tmp/right_sync_leftbaseline_j2ki005_03.json) | 0.7961 / 0.5077 | 3.056 s | 2.677 s | **5.733 s** |

三轮共同结果：

- [x] 同一套右臂配置、三组起点均观察到 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`。
- [x] J2 不再因持续静差导致收敛超时；最近两轮总同步时间约 `5.5～5.7 s`。
- [x] 三轮轨迹、收敛及所记录的 SYNCED 区间，八轴命令电流裁剪样本均为 0。
- [x] 三轮 `missing_samples=0`、`sequence_restarts=0`、`follower_command_count=0`，汇总均为 `RECORDED`。
- [x] 三轮均观察到停止后的 IDLE、软件输出归零及缓存 torque 关闭。

三轮总耗时为 `5.52～7.67 s`，不能记为三轮均约 5 s 或均严格小于 5 s。
SYNCED 后分别记录约 `6.16 / 9.31 / 5.85 s`；这些区间的八轴最大位置误差均未超过 `0.05 rad`，
但存在瞬时速度超过 `0.05 rad/s` 的样本，因此进入 SYNCED 不等于其后每帧均满足同步入口判据。
三份汇总的 `hardware_torque_disable_verified` 均为 `false`，本节没有新增独立失能寄存器回读。
上述软件停止结果不作为单次 stop 可靠性已完成验收的结论。

### 12.4 本轮收尾范围与后续复现

本轮右臂归零调参先收尾，保留“第 11.2 节左臂基线 + 右臂 J2 保持 `ki=0.05`”，
不继续为缩短收敛时间增加增益。左臂 J2 保持 `ki=0.00` 不变。

操作者明确选择跳过额外的 60 s 保持测试，本轮不再将其作为收尾前置条件；
该项未执行，不记为已通过。前述过渡试验中的轨迹抖动、异常长控制周期间隔及 stop 偶发现象
未在本轮定位或修复，也未新增分段耗时记录。这些问题与本节三轮归零成功结论分别保留。
本节不将第 10.5 节其余未完成的阶段验收项目标为完成。

复现时设置 `SIDE=right`，`CONFIG` 指向当前 `config/grav_comp_m6_right.yaml`，
使用 `start_teleop_m6_right.launch.py right_config:="$CONFIG" sync_diagnostics:=true` 启动。
修改 YAML 后须重启驱动，节点和记录工具使用同一份右臂配置。先开始记录，再执行归零：

```bash
# 使用未占用的新文件名；到期只保存记录，不发送停止命令。
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side right --config "$CONFIG" --operator lyy \
  --output /tmp/right_sync_leftbaseline_j2ki005_next01.json --duration 180
```

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial move \
  --side right --config "$CONFIG" \
  --target 0 0 0 0 0 0 0 --execute
```

## 13. 双 USB 设置与双臂并行归零命令（2026-10-21）

本节给出双臂测试的操作命令，不代表双臂并行测试已经通过。
左臂沿用第 11 节基线，右臂沿用第 12 节基线，仅右臂 J2 保持 `ki=0.05`。
先退出此前分别启动的单臂驱动，再启动双臂 launch，避免重复占用同一串口。

双臂归零可以在一个终端发起：并行启动两个现有 `move` 工具，分别发布到
`/left_joint_move` 和 `/right_joint_move`，各自等待对应侧的 SYNCED。
此前 `> /tmp/…log 2>&1 &` 加 `wait`、`tail` 的写法也已经在一个终端并行发送，
但所有进度先进入日志，终端须等两侧工具结束后才能看到最后几行，不便现场观察。
第 13.5 节改为按行实时标注左右臂、显示并保存全部输出，再分别汇总结果。
两个 CLI 各自连接和校验状态，实际开始运动的时间可能有差异，不保证同一控制周期起动。

### 13.1 每个终端的环境准备

使用 Bash；每个终端先执行以下命令。修改配置后须重启驱动，保证驱动与工具使用同一份文件。

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export LEFT_CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_left.yaml"
export RIGHT_CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_right.yaml"
```

### 13.2 终端 1：设置两个 FTDI USB 的 latency_timer

在启动驱动前执行。按 by-id 分别找到对应串口，无需固定 `ttyUSB0 / ttyUSB1` 的分配顺序。
`FTBEO9L1` 对应左臂，`FTBEO8WL` 对应右臂。重新插拔 USB 或重启后再次设置并确认。

```bash
sudo -v

for USB_BY_ID in \
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0 \
  /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0
do
  TTY_PATH=$(readlink -f "$USB_BY_ID")
  if ! test -c "$TTY_PATH"; then
    echo "串口不存在：$USB_BY_ID" >&2
    continue
  fi

  TTY_NAME=$(basename "$TTY_PATH")
  LATENCY_PATH="/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
  echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
  if ! printf '1\n' | sudo tee "$LATENCY_PATH" > /dev/null; then
    echo "设置失败：$LATENCY_PATH" >&2
    continue
  fi
  echo "latency_timer 读取确认："
  cat "$LATENCY_PATH"
done
```

两侧的读取确认均应为 `1`；设置失败或串口缺失时，处理对应问题后再启动。

### 13.3 终端 1：启动双臂驱动

```bash
ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  left_config:="$LEFT_CONFIG" \
  right_config:="$RIGHT_CONFIG" \
  sync_diagnostics:=true
```

双臂使用独立节点、串口、归零话题及诊断话题；启动后确认两侧驱动均正常运行。

### 13.4 终端 2、3：分别开启左右臂记录

先启动记录，再发送目标。下面的输出文件名必须未被使用；重复测试时两侧一起换成新的轮次。
每份汇总旁会生成同名 `.trace.jsonl`。记录到期只保存数据，不发送停止命令。

终端 2，左臂：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side left --config "$LEFT_CONFIG" --operator lyy \
  --output /tmp/dual_sync_left_next01.json --duration 180
```

终端 3，右臂：

```bash
ros2 run skye_factr_teleop skye_factr_sync_trial record \
  --side right --config "$RIGHT_CONFIG" --operator lyy \
  --output /tmp/dual_sync_right_next01.json --duration 180
```

### 13.5 终端 4：一个终端并行归零，实时显示两侧进度

执行第 13.1 节的环境准备后，将下面整块复制到同一个 Bash 终端执行。
两侧均须处于可接受新目标的 IDLE 状态。
`PYTHONUNBUFFERED=1` 与 `sed -u` 用于及时输出；每行会带 `[左臂]` 或 `[右臂]` 标识。
`tee` 同时保存日志，日志目录每次自动创建，不覆盖前一轮。
输出管道启用 `pipefail`，以保留工具失败状态，不把日志保存成功误记为同步成功。

```bash
DUAL_MOVE_LOG_DIR=$(mktemp -d /tmp/dual_sync_move_XXXXXX)
echo "本轮归零日志目录：$DUAL_MOVE_LOG_DIR"

run_arm_zero() (
  set -o pipefail
  arm_side="$1"
  arm_config="$2"
  arm_label="$3"
  arm_log="$4"

  printf '[%s] 开始归零，等待对应侧 SYNCED。\n' "$arm_label"
  if PYTHONUNBUFFERED=1 ros2 run skye_factr_teleop skye_factr_sync_trial move \
    --side "$arm_side" --config "$arm_config" \
    --target 0 0 0 0 0 0 0 --execute \
    2>&1 | sed -u "s/^/[$arm_label] /" | tee "$arm_log"
  then
    printf '[%s] 本侧归零工具正常结束，已确认 SYNCED；机械臂继续保持目标姿态。\n' "$arm_label"
  else
    arm_result=$?
    printf '[%s] 本侧归零工具异常结束（退出码 %s），请查看上方原因；未自动发送停止命令。\n' \
      "$arm_label" "$arm_result"
    exit "$arm_result"
  fi
)

run_arm_zero left "$LEFT_CONFIG" 左臂 "$DUAL_MOVE_LOG_DIR/left.log" &
LEFT_MOVE_PID=$!
run_arm_zero right "$RIGHT_CONFIG" 右臂 "$DUAL_MOVE_LOG_DIR/right.log" &
RIGHT_MOVE_PID=$!

if wait "$LEFT_MOVE_PID"; then LEFT_MOVE_RC=0; else LEFT_MOVE_RC=$?; fi
if wait "$RIGHT_MOVE_PID"; then RIGHT_MOVE_RC=0; else RIGHT_MOVE_RC=$?; fi

if [ "$LEFT_MOVE_RC" -eq 0 ] && [ "$RIGHT_MOVE_RC" -eq 0 ]; then
  echo "双臂结果：左右臂均已确认 SYNCED；继续保持目标姿态。"
else
  printf '双臂结果：未能确认两侧全部成功，左臂退出码=%s，右臂退出码=%s。\n' \
    "$LEFT_MOVE_RC" "$RIGHT_MOVE_RC"
fi
echo "完整日志目录：$DUAL_MOVE_LOG_DIR"
```

运行中可直接看到各侧的起点、参考轨迹时长、`Observed mode`、超差关节和连续收敛时间，
成功时出现带对应侧标识的 `Synchronization complete: SYNCED`。
某侧提前结束时会立即显示该侧结果，不必等另一侧结束。
只有两侧工具均正常结束，最后才输出“双臂均已确认 SYNCED”。
该命令不调整任何控制参数，也不会因某侧工具报错或等待结束而自动停止机械臂。

### 13.6 停止双臂

观察完成后，或运动中出现异常时，在另一个已准备好 ROS 环境的终端执行：

```bash
ros2 topic pub --once /mode/switch_stop \
  std_msgs/msg/String '{data: switch_stop}'
```

`/mode/switch_stop` 是两侧共同订阅的全局停止话题。应确认左右臂均停止并退出活动状态，
不能仅凭发布端的 `publishing #1` 判断实际停止已完成。
归零工具结束、记录时长到期，以及终端 4 中断进度显示，都不等同于停止双臂。

## 14. 双臂首次并行归零结果：文本日志记录（2026-10-21）

### 14.1 本轮操作与证据

操作者 `lyy` 按第 13 节启动双臂驱动，并在同一个终端并行运行左右臂 `move` 工具。
配置沿用第 11 节左臂基线和第 12 节右臂基线，未为本次双臂测试调整参数；
右臂 J2 保持 `ki=0.05`，左臂为 `0.00`。
两侧七个臂关节目标均为 `0 rad`，夹爪目标均为 `0.4 rad`。

本轮没有启动第 13.4 节的逐帧 `record`，没有对应的汇总 JSON、`.trace.jsonl` 或配置快照。
但第 13.5 节命令中的 `tee` 已保存左右臂工具的文本输出，文件已核对存在：

- 左臂：[left.log](/tmp/dual_sync_move_R5pbHC/left.log)。
- 右臂：[right.log](/tmp/dual_sync_move_R5pbHC/right.log)。

这些文件目前位于本机 `/tmp/dual_sync_move_R5pbHC/`，尚未归档到仓库。
终端显示每侧工具均正常结束，最后汇总为“左右臂均已确认 SYNCED；继续保持目标姿态”。

### 14.2 双侧结果与收敛差异

| 项目 | 左臂 | 右臂 |
|---|---|---|
| CLI 预览起点 J3 / J5（rad） | -0.3298 / 0.3835 | 0.2117 / -0.3666 |
| CLI 预览参考轨迹时长 | 3.115 s | 3.034 s |
| 工具观察到的状态顺序 | TELEOP_SYNCING → HOLD_SETTLING → SYNCED | TELEOP_SYNCING → HOLD_SETTLING → SYNCED |
| 同步结果 | 成功，先进入 SYNCED | 成功，稍后进入 SYNCED |

右臂进入 HOLD_SETTLING 时，J2 位置误差约为 `+0.105845 rad`，随后降至 `+0.058291 rad`，
较长时间停留在约 `+0.055223 rad`，再逐步降到 `+0.050621 rad` 附近后进入 SYNCED。
后半段仅 J2 持续被列为超差关节，因此右臂本轮主要等待 J2 收敛。
`0.050621 rad` 是成功前的超差采样值，不是最终达标值，也不表示容差已放宽。

表中的时长来自 CLI 轨迹预览，不是实测轨迹执行时间或总同步时间。
文本日志没有逐帧单调时钟，不能按逐秒打印次数计算精确同步耗时，
也不能由本轮两侧均成功推断双侧总时间均约 5 s。

### 14.3 当前结论与保留范围

- [x] 同一终端并行发起双臂归零，左右臂各自确认进入 SYNCED；本轮可记为首次双臂并行归零成功。
- [x] 带左右臂标识的进度、超差条件和成功信息已实时显示，并分别保存为文本日志。
- [x] 当前控制参数继续保留，不因本轮右臂收敛较慢而立即增加增益。

本轮文本输出不足以统计电流裁剪、诊断丢样和完整保持区间，也不能据此判定全程运动平稳、
双侧起动时间一致或 stop 可靠性已通过。用户提供的本轮输出截至同步成功，未补记停止后的状态或失能回读。
额外 60 s 保持测试仍按第 12.4 节约定跳过，不作为本轮收尾前置条件。
后续如继续双臂测试，先按第 13.4 节分别启动两侧记录器，再执行第 13.5 节归零命令。
