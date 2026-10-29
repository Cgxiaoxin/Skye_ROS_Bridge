# 阶段 7.5：连接大臂的双臂同步与关节遥操 checklist

日期：2026-10-27。设备 A / Thor，操作者 lyy。
本清单独立覆盖编译、环境、双串口、大小臂启动、双侧记录、正式同步、遥操、停止及失能回读。
当前已测双臂正常同步、遥操及停止保持流程验收完成，左右独立失能回读 PASS，详见第 10.2 节。
前文步骤保留供后续复测使用。

## 1. 当前范围与参数

- [ ] 左臂全流程已由操作者确认完成；右臂已有非零、近零同步及人工支撑下遥操、停止保持记录。
- [ ] 按操作者决定，不再安排右臂逐轴专项试验，直接进入双臂正常流程联调。
- [x] **J6/J7 方向问题已由操作者自行解决并确认关闭，不再作为待办。大臂命令符号及 profile 映射由操作者维护。**
- [ ] 右臂少许自行转动、同步/保持中的小幅补动已有现场观察。本轮记录其影响，保持当前共用参数。
- [ ] 操作者能支撑两侧小臂并操作独立停止终端；小臂 stop 后会关闭 torque。

使用原 YAML，显式传入路径；两侧共用控制参数，各自保留串口、方向、零点和模型。

| 项目 | 当前基线 |
|---|---|
| 左小臂配置 | [grav_comp_m6_left.yaml](../../config/grav_comp_m6_left.yaml) |
| 右小臂配置 | [grav_comp_m6_right.yaml](../../config/grav_comp_m6_right.yaml) |
| 大臂配置 | [skye_robot.yaml](../../../skye_robot_driver/config/skye_robot.yaml)，`robot_profile:=thor` |
| 控制器 IP | `6.6.7.191` |
| ROS 环境 | Humble，domain `73`，`rmw_fastrtps_cpp` |
| 小臂名义控制频率 | `500 Hz`，实际周期由记录判断 |
| 电流上限，物理 ID1..8 | `[500,500,450,250,180,180,180,50] mA` |
| 同步参考速度 / 加速度 / jerk 上限 | `0.8 rad/s` / `1.0 rad/s²` / `5.0 rad/s³` |
| 同步夹爪目标 | `0.4 rad` |
| 七轴位置容差 | `0.0873 rad`，约 5°，**不是 `0.87 rad`** |
| 夹爪位置容差 | `0.05 rad` |
| 八轴速度容差 / 连续达标时间 | `0.05 rad/s` / `0.25 s` |
| 轨迹 / 收敛超时 | `30 s` / `25 s` |
| 轨迹到保持的平滑交接 | `0.5 s`，共用控制代码 |
| 大臂模式 | `imp_joint`，预期状态 `[2,2]` |
| 两侧大臂速度 / 加速度比例 | 均为 `10 / 10` |
| 大臂遥操模式 | `relative` |
| 共用逐帧增量上限 / 命令超时 | `0.001 rad/命令帧` / `0.50 s` |

位置容差只决定何时进入 SYNCED；之后仍执行保持 PID 和积分，补动不会因状态变成 SYNCED 而自动停止。
Ruckig 上限作用于同步参考轨迹，不是实测速度上限，也不限制手动遥操速度。
大臂逐帧增量上限不是固定速度上限。当前小臂只发布七轴遥操命令，不发布夹爪遥操命令；
同步仍包含小臂夹爪 ID8，大臂驱动按原配置启用自身夹爪。

## 2. 编译与离线检查

### 2.1 先退出旧节点

- [ ] 先正常 stop；进入过遥操时，确认小臂停止后调用大臂 hold。
- [ ] 正常退出旧单臂/双臂小臂 launch、串口检查工具，避免两个进程访问同一条总线。
- [ ] 停止旧 FACTR、follower_align、策略或其它运动命令发布者。
- [ ] 修改配置后重启相应节点；已经运行的节点不会自动采用磁盘上的新值。

### 2.2 编译

在一个 Bash 终端执行。以下命令不启动硬件：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to skye_factr_teleop --symlink-install
```

确认成功后，再编译大臂驱动：

```bash
colcon build --packages-select skye_robot_driver \
  --cmake-args -DAMENT_CMAKE_SYMLINK_INSTALL=OFF
```

大臂驱动沿用当前工作区的普通安装方式，显式关闭 CMake 符号链接安装，避免与已有安装目录冲突。
首次运行，或源码、launch、安装资源发生变化时执行编译；只修改本清单显式传入的源 YAML 时，重启节点即可。

```bash
source install/setup.bash
ros2 run skye_factr_teleop skye_factr_sync_trial --help
ros2 run skye_factr_teleop skye_factr_teleop_trial --help
ros2 run skye_factr_teleop skye_factr_dual_trial --help
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- --help
```

- [ ] 两个包编译成功，四个工具帮助可用。双臂记录器是新增入口，首次使用须完成小臂包编译。

## 3. 每个终端的公共环境

后文每个新终端先执行本段一次；这些环境及配置路径不随轮次变化，不用每轮重新设置。

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=73
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
LEFT_CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_left.yaml"
RIGHT_CONFIG="$PWD/src/skye_factr_teleop/config/grav_comp_m6_right.yaml"
DRIVER_CONFIG="$PWD/src/skye_robot_driver/config/skye_robot.yaml"
```

无需设置或同步 `RUN_ID`。记录器每次自动在 `/tmp/skye_factr_trials/` 下创建新的会话目录，
目录名包含阶段、时间和唯一后缀；重复执行同一条记录命令即可开始新一轮，不覆盖旧记录。
可选用 `--output-dir /path/to/records` 指定会话父目录；仍支持原来的成对 `--left-output/--right-output`，
显式文件模式不能与 `--output-dir` 混用。
所有程序在同一主机、同一 domain 73 运行；domain 21 中历史节点不用于本轮。

可在公共环境加载后，离线核对当前配置：

```bash
/usr/bin/python3 - <<'PY'
from skye_factr_teleop.config import load_config

left = load_config('src/skye_factr_teleop/config/grav_comp_m6_left.yaml', require_runtime=True)
right = load_config('src/skye_factr_teleop/config/grav_comp_m6_right.yaml', require_runtime=True)
assert left.controller.sync_hold_control == right.controller.sync_hold_control
assert left.dynamixel.current_limit_ma == right.dynamixel.current_limit_ma
for side, config in (('left', left), ('right', right)):
    hold = config.controller.sync_hold_control.hold
    assert list(hold.sync_position_tolerance) == [0.0873] * 7 + [0.05]
    assert list(hold.sync_velocity_tolerance) == [0.05] * 8
    assert hold.settle_duration == 0.25
    print(side, config.dynamixel.dynamixel_port)
    print('position tolerance:', hold.sync_position_tolerance)
print('Shared sync parameters and current limits checked.')
PY
```

本检查只读取配置，不打开串口；左右硬件专属标定不要求相同。

## 4. 双串口配置：侧别、权限、占用和 FTDI 延迟

### 4.1 确认 by-id 与实体接线

```bash
ls -l /dev/serial/by-id/
rg -n 'dynamixel_port:|baudrate:' "$LEFT_CONFIG" "$RIGHT_CONFIG"
```

| 侧别 | 设备 A USB by-id | 舵机 ID / 波特率 |
|---|---|---|
| 左 | `usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0` | ID1..8 / `4000000` |
| 右 | `usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0` | ID1..8 / `4000000` |

- [ ] by-id、YAML 和实体线缆侧别一致。
- [ ] 两侧总线分别有 ID1..8；两条独立总线使用相同 ID 是正常的。
- [ ] 不用 `ttyUSB0/ttyUSB1` 编号判定侧别，USB 拔插后编号可能变化。

### 4.2 逐一检查并设置 latency_timer

此时小臂驱动和串口检查工具均应已退出。以下在子 shell 中执行，任一侧失败即停止本段，
修复后重跑；不要跳过失败侧继续启动。

```bash
(
  sudo -v || exit 1
  for USB_BY_ID in \
    /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0 \
    /dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0
  do
    TTY_PATH=$(readlink -f "$USB_BY_ID")
    if ! test -c "$TTY_PATH"; then
      echo "串口不存在：$USB_BY_ID" >&2
      exit 1
    fi
    if ! test -r "$TTY_PATH" || ! test -w "$TTY_PATH"; then
      echo "当前用户缺少串口读写权限：$TTY_PATH" >&2
      stat -c '%A %U %G %n' "$TTY_PATH"
      id
      exit 1
    fi
    echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
    if fuser -v "$TTY_PATH"; then
      echo "串口仍被占用，请先正常退出对应程序" >&2
      exit 1
    fi
    TTY_NAME=$(basename "$TTY_PATH")
    LATENCY_PATH="/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
    printf '1\n' | sudo tee "$LATENCY_PATH" > /dev/null || exit 1
    echo "latency_timer 读取确认（应为 1）："
    cat "$LATENCY_PATH" || exit 1
    test "$(cat "$LATENCY_PATH")" = 1 || exit 1
  done
)
```

`fuser` 无占用时通常无输出、返回非零；显示 PID 时先正常退出对应程序。
如权限不足，先根据 `stat`、`id` 核对设备所属组。设备组为 `dialout` 且用户尚未加入时可执行：

```bash
sudo usermod -aG dialout "$USER"
```

之后注销并重新登录，再重新加载环境并复查权限。
两侧 `latency_timer` 都必须读回 `1`；主机重启或 USB 重插后重新设置。
总线波特率由驱动按 YAML 设置，不另用 `stty` 改写。

### 4.3 启动前只读检查

驱动仍未启动时，在同一个终端创建一次启动前检查目录，再分别检查两条总线。
该目录变量只供本步骤使用，不需要同步到其它终端：

```bash
PREFLIGHT_DIR=$(mktemp -d /tmp/dual_preflight_XXXXXXXX)
echo "$PREFLIGHT_DIR"
```

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left --config "$LEFT_CONFIG" --frames 2 --interval 0 \
  --output "$PREFLIGHT_DIR/left.json"
```

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side right --config "$RIGHT_CONFIG" --frames 2 --interval 0 \
  --output "$PREFLIGHT_DIR/right.json"
```

- [ ] 两侧均 `automated_result=PASS`，各 ID1..8 都有结果且 `torque_enabled=false`。
- [ ] 核对两侧位置与实际姿态，夹爪读数与配置范围一致；此前右 ID8 曾因位置坐标异常顶住止挡。
- [ ] 无当前硬件报警。右 ID3 历史上出现过 `0x20 OVERLOAD`，处理后须重新检查，不沿用旧 PASS。

任一侧检查失败时先处理对应错误；不边运行小臂驱动边用工具竞争串口。

## 5. 网络、大小臂启动及运行检查

### 5.1 控制器网络

```bash
ip route get 6.6.7.191
ping -c 3 6.6.7.191
```

- [ ] 路由走连接控制器的网口，控制器通信可用；异常时先处理现有网络配置。
- [ ] 大臂两侧保持已测近零或已验证的安全姿态；小臂起点、运动路径、线缆和两臂间距可行。

### 5.2 终端分工

| 终端 | 用途 |
|---|---|
| A | 一份大臂驱动，管理左右大臂 |
| B | 一份双臂小臂 launch，内部创建左、右两个节点 |
| C | 一个双臂记录器，同时接收左右侧数据，分别保存左右文件 |
| D | 参数/反馈检查、sync/teleop/stop 请求及 hold 服务 |

记录器会占用终端。终端 D 独立保留，不把停止命令排在阻塞的记录命令后面。

### 5.3 终端 A：大臂驱动

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=thor "params_file:=$DRIVER_CONFIG"
```

该命令会连接真机并进入原配置的控制模式，也会启动配置中的大臂夹爪。
已有同一驱动运行且已加载本轮参数时沿用；需换参数时先正常退出再启动，避免重复连接。

### 5.4 终端 B：双臂小臂驱动

```bash
ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=thor sync_diagnostics:=true \
  "left_config:=$LEFT_CONFIG" "right_config:=$RIGHT_CONFIG"
```

- [ ] 两个小臂节点均正常启动，初始为 IDLE、torque 关闭。
- [ ] 不再同时运行单臂 launch。

### 5.5 终端 D：有效参数、反馈和发布者

```bash
ros2 node list
ros2 param get /skye_robot_driver teleop_mapping_mode
ros2 param get /skye_robot_driver left_velocity_ratio
ros2 param get /skye_robot_driver right_velocity_ratio
ros2 param get /skye_robot_driver left_acceleration_ratio
ros2 param get /skye_robot_driver right_acceleration_ratio
ros2 param get /skye_robot_driver max_delta_per_cycle
ros2 param get /skye_robot_driver command_timeout_s
ros2 param get /skye_factr_teleop_left config_path
ros2 param get /skye_factr_teleop_right config_path
ros2 topic echo --once /gento/robot_state std_msgs/msg/Int16MultiArray
ros2 topic echo --once /gento/left_joint_states sensor_msgs/msg/JointState
ros2 topic echo --once /gento/right_joint_states sensor_msgs/msg/JointState
ros2 topic info --verbose /gento/left_joint_control
ros2 topic info --verbose /gento/right_joint_control
ros2 topic info --verbose /gento/left_joint_control_abs
ros2 topic info --verbose /gento/right_joint_control_abs
```

- [ ] 大臂参数依次为 relative、四个 10、0.001、0.5；小臂配置路径各自正确。
- [ ] 大臂状态 `[2,2]`；左反馈名 `l_j1..l_j7`，右反馈名 `r_j1..r_j7`。
- [ ] 反馈时间戳非零且新鲜，大小臂实体姿态与反馈一致，目标在小臂安全范围内。
- [ ] 左右 control 各只有对应小臂一个发布者，两侧 abs 均无发布者。

源 YAML 快照不等于运行时有效值；若此前用服务改过 motion rates，先重启大臂驱动恢复基线。
小臂在 IDLE 时也创建 control 发布者，发布者存在不表示正在发送命令。
临时 echo 的一次 `A message was lost` 提示本身不等于控制节点反馈丢失，结合实际诊断判断。
本段不检查或修改大臂符号映射。

当前两侧小臂七轴安全区相同，包含 `0.1 rad` 安全裕量：

| 关节 | 安全区间，rad |
|---|---|
| J1 / J3 / J5 | `[-2.9,2.9]` |
| J2 | `[-1.9,1.9]` |
| J4 | `[-2.3,0.9]` |
| J6 | `[-0.8,0.8]` |
| J7 | `[-0.7,0.7]` |

正式 sync 在请求时截取两侧各自的大臂七轴反馈，小臂分别走向该目标；不是持续追随大臂新姿态。
同步及保持期间大臂保持静止；首轮优先使用已测近零目标。

## 6. 第一轮：只验证双臂正式同步

### 6.1 终端 C：一个双臂同步记录器

一条命令启动一个 ROS 记录节点，终端每行带 `[left]` 或 `[right]`。
每轮自动生成会话目录，左右仍分别保存 JSON 和 trace：

```bash
ros2 run skye_factr_teleop skye_factr_dual_trial \
  --phase sync --left-config "$LEFT_CONFIG" --right-config "$RIGHT_CONFIG" \
  --operator lyy --duration 180
```

- [ ] 同一终端分别收到 `[left] IDLE` 和 `[right] IDLE`，两侧未显示 WAITING/STALE。
- [ ] 已看到本轮 `Session directory:`，以及 `[left] Recording ...`、`[right] Recording ...` 路径。

### 6.2 终端 D：一次请求，两侧分别确认

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
```

- [ ] 终端 C 的 `[left]`、`[right]` 各自出现 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED`。
- [ ] 两侧轨迹、交接和保持表现可接受，无报警、机械干涉或异常扩大。
- [ ] **必须分别确认 `[left]`、`[right]` 均为 SYNCED。** 全局 `/teleop/state` 有两个发布者，一条 SYNCED 不能证明双侧成功。
- [ ] 两侧均同步后保持约 5～10 s，记录补动及人工支撑情况。

全局请求会被两侧各自处理，不保证同一控制周期起动或同一时刻完成。
若一侧失败或超时，停止本轮；共同请求不代表存在“两侧全部就绪”的自动门禁。
本轮仅验证同步，不发 TELEOP 请求；两侧大臂 control 命令计数均应为 0。

### 6.3 停止并退出

终端 D 发一次共同 stop，继续支撑两侧小臂：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

记录器须等两侧都经历活动并返回零输出 IDLE 后才自动结束，不因一侧先停止而截断另一侧。
确认终端 C 的 `[left]`、`[right]` 都收到 IDLE、完成保存。终端 B 用 Ctrl+C 正常退出双臂小臂节点，
确认串口释放；保持舵机供电、不重启总线，执行第 8 节中的同步回读。
本轮大臂始终静止；如实体状态需要受控保持，确认小臂停止后调用第 7.4 节 hold。

记录器到期、Ctrl+C 或关闭窗口只保存记录，均不停止大小臂；出现异常不等待 180 s。

## 7. 后续轮次：双臂遥操

第一轮双臂同步及停止结果确认后执行。若已退出终端 B，按第 5.4 节重新启动双臂小臂节点；
大臂驱动保持一份。重新从 IDLE 正式同步，不使用 `joint_move` 代替。

### 7.1 终端 C：一个双臂遥操记录器

使用同一个工具的 teleop 模式，显式传入大臂配置快照：

```bash
ros2 run skye_factr_teleop skye_factr_dual_trial \
  --phase teleop --left-config "$LEFT_CONFIG" --right-config "$RIGHT_CONFIG" \
  --driver-config "$DRIVER_CONFIG" \
  --operator lyy --duration 180 --post-stop-seconds 3
```

工具只读记录，不发送同步、遥操或停止命令。双臂遥操时，每侧文件的 `other_command`
表示另一侧正常命令，不能按单臂测试要求其为零；两侧 abs 仍应无命令。
`action_applied` 表示 SDK 成功下发的目标，不是实测位置。
记录器分别计时两侧的停止尾段，须两侧都返回零输出 IDLE 且各自经过约 3 s 后才自动结束。
尚未收到某侧诊断或某侧仍在运行时不会提前自动结束；总时长到期或 Ctrl+C 会分别保存已有数据。
不要把结束录制当作停止硬件。

### 7.2 第一轮遥操：静止切换

终端 C 的左右记录均就绪后，终端 D 请求：

```bash
ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
```

分别确认左右两侧均为 SYNCED，操作者支撑好两侧小臂，再执行：

```bash
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
```

- [ ] 终端 C 的 `[left]`、`[right]` 均出现 TELEOP；若只有一侧进入，停止本轮。
- [ ] 最初约 5～10 s 保持支撑、暂不主动拖动，观察自行转动、下落、上抬或切换突动。
- [ ] 大臂各自捕获会话参考，首帧保持进入时姿态，未明显追赶同步残差。
- [ ] 记录支撑方式；有手扶时不把“未下落”解释为无支撑悬停通过。

relative 模式会捕获大小臂参考，随后跟随小臂相对位移；保持 PID 在切入 TELEOP 后退出，
所以同步保持正常不能代替遥操切换验收。首轮静止切换后按第 7.4、8 节停止及回读，分析后再做运动轮。

### 7.3 后续遥操轮：左右隔离及共同运动

每轮重复执行第 7.1 节记录命令，自动生成新会话；重新正式同步到双侧 SYNCED 后切入 TELEOP。
按操作者决定，不重新安排右臂逐轴专项试验；本节检查双臂同时运行的行为。

1. 两侧都保持支撑，左侧做小幅、缓慢拖动，右侧尽量保持静止，观察大臂是否串侧运动。
2. 右侧做小幅、缓慢拖动，左侧尽量保持静止，观察对应跟随及非目标侧状态。
3. 两侧分别验证后，再做双侧同时小范围动作，保持两臂及线缆无干涉。
4. 对照两侧 command、applied、feedback，观察跟随、积压、停止响应及电流裁剪。
   J6/J7 方向问题已由操作者解决，本流程沿用当前映射。

每轮结束执行共同 stop、hold 和两侧失能回读。主动拖动、自行转动、补动和人工支撑片段分别记录。

### 7.4 每轮停止及大臂保持

终端 D 发共同 stop，操作者继续支撑两侧小臂：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

**分别确认终端 C 的 `[left]`、`[right]` 都回到 IDLE 后**，调用一次大臂保持：

```bash
ros2 service call /gento/hold_current std_srvs/srv/Trigger '{}'
```

- [ ] hold 返回 `success: true`，左右大臂实际保持、不再追赶。
- [ ] 双臂记录器已保存两侧各自停止后约 3 s 的尾段，并分别输出保存结果。
- [ ] 正常退出终端 B 的双臂小臂节点，释放两个串口，再执行第 8 节遥操回读。

`hold_current` 对双臂生效并重置两侧会话，可能产生双侧 applied 保持回执。
`switch_stop` 本身不直接调用大臂 hold；命令断流后还有驱动超时保持路径，
但 `0.50 s` 超时还包含检查定时器、调度和 SDK 时间，不能当作硬实时停止保证。
一侧故障不代表另一侧一定自动停止；先停止整个试验并确认两侧状态。
若软件停止或 hold 失败，按现场应急方式处理，不等记录器结束；已有软件急停接口为：

```bash
ros2 service call /gento/emergency_stop std_srvs/srv/Trigger '{}'
```

恢复测试时需重新正式同步；故障后关闭串口的小臂节点需重启。

## 8. 退出小臂节点后的双侧独立失能回读

前提：先完成 stop，遥操轮还需完成 hold；记录已保存，终端 B 已正常退出，两个串口无占用。
两侧均保持供电，不在回读前重启总线。

### 8.1 使用记录器打印的本轮命令

同步和遥操轮均采用相同步骤：记录器结束时，在终端 C 自动打印两条完整的
`ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check ...` 命令，
其中已经填好左右配置和本次会话对应的输出路径。

1. 在终端 D 完成 stop，遥操轮还需确认 hold 成功及大臂实际保持。
2. 在终端 B 正常退出双臂小臂节点，确认两个串口已释放。
3. 回到终端 C，复制该终端刚打印的两条回读命令，按左、右顺序执行。

不需要在任何终端设置轮次编号或手动拼接路径。打印命令不会自动打开串口，必须等退出小臂节点后再执行。
中途结束或不完整记录也会打印命令；先完成实际停止和退出，再回读。

### 8.2 本轮文件对应关系

以记录器打印的会话目录为准，例如 `/tmp/skye_factr_trials/dual_sync_20261027_160000_abcd1234/`。

| 文件 | 用途 |
|---|---|
| `left.json` / `right.json` | 左右记录汇总及配置快照 |
| `left.trace.jsonl` / `right.trace.jsonl` | 左右逐帧记录 |
| `left_torque_disable.json` / `right_torque_disable.json` | 执行打印命令后产生的独立失能回读 |

- [ ] 左右均 `automated_result=PASS`，各自 ID1..8 的 `torque_enabled` 全为 `false`。
- [ ] 回读文件与左右记录位于同一会话目录；串口占用或错误导致的 FAIL 保留，不改写为 PASS。

## 9. 每轮记录与结论

每侧保存汇总 JSON、同名 trace、匹配的独立失能回读；同时保存运行参数输出、hold 返回及现场观察。
文件目前写入本机 `/tmp/`，需要长期留存时归档这些关联文件，避免只保留汇总 JSON。

| 项目 | 本轮填写 |
|---|---|
| 阶段 / 会话目录 / 日期 / 操作者 | 待填写 |
| 两侧目标与起点 / 是否人工支撑 | 待填写 |
| 左侧轨迹、收敛耗时 / 保持表现 | 待填写 |
| 右侧轨迹、收敛耗时 / 保持表现 | 待填写 |
| 双侧进入 SYNCED / TELEOP 的情况 | 待填写 |
| 左动右静 / 右动左静 / 共同运动 | 待填写 |
| 补动、自行转动、积压、饱和或报警 | 待填写 |
| 共同 stop 后两侧 IDLE / 大臂 hold 返回及实际保持 | 待填写 |
| 左右记录及独立回读文件 | 待填写 |
| 本轮结论与下一步 | 待填写 |
| 已知现场现象 | 右臂少许自行转动及同步/保持补动；右夹爪短时边界限流见第 10.2 节 |

正常流程验收逐项确认：

- [ ] 两侧 USB、节点、配置和话题对应正确，无重复驱动或额外命令源。
- [x] 双侧真实反馈同步成功，目标在安全范围，现场响应可接受；同步期间无大臂 control 命令。
- [x] 双侧切入 TELEOP 的响应现场可接受，大臂首帧保持进入时姿态。
- [x] 左右分别运动及共同运动可控，无串侧运动；J6/J7 方向问题由操作者确认已解决。
- [x] 共同 stop 后两侧 IDLE、软件输出归零；大臂 hold 成功且实体保持。
- [x] 两侧独立失能回读 PASS；停止后再次遥操前仍须重新同步。

`RECORDED` 表示记录流程满足工具条件，不自动等于硬件 PASS。
5° 为到位判据，不等于保持期间每帧速度均达标或补动已消除；判断结合实际误差、速度、电流和现场表现。
结论限于实测姿态和动作范围，并列明现场现象；J6/J7 方向问题按操作者确认关闭。
受控超时、反馈中断和单侧故障等异常路径另选具体试验方式安排，不因正常流程完成而自动标记通过。

历史参数及单臂记录见[右臂同步调参与遥操记录](phase7_3_7_4_right_checklist.md)。

## 10. 首批双臂真机记录（2026-10-27）

### 10.1 首个会话：只执行同步，遥操模式汇总为 INCOMPLETE

会话 `/tmp/skye_factr_trials/dual_teleop_20261027_172832_4mhvfcpn`。
两侧状态均为 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → IDLE`，未进入 TELEOP。
两侧诊断丢帧、序号重启及大臂 control 命令数均为 0，八轴所有阶段均无软件电流裁剪。

因为使用 `--phase teleop`，工具按遥操完成条件将左右汇总记为 `INCOMPLETE`；
这不表示同步失败，也不是双臂记录器异常。会话保留为双臂同步与停止的软件记录，
无该会话独立失能回读，不能据此补写硬件失能已验证。
启动时短暂 `[right] WAITING` 随后恢复收到右侧 IDLE，后续诊断无丢帧。

### 10.2 第二个会话：双臂正常同步、遥操及停止保持验收完成

会话 `/tmp/skye_factr_trials/dual_teleop_20261027_172941_ljcp6cga`。
两侧完整经历 `IDLE → TELEOP_SYNCING → HOLD_SETTLING → SYNCED → TELEOP → IDLE`，
工具均为 `RECORDED`，停止后 3 s 时窗均已保存。

| 项目 | 左臂 | 右臂 |
|---|---:|---:|
| 同步总耗时 | 3.7463 s | 3.8561 s |
| 保持收敛耗时 | 0.6900 s | 0.9160 s |
| TELEOP 诊断首末帧间隔 | 94.4200 s | 94.4211 s |
| 本侧命令帧数 | 46801 | 46777 |
| 命令段平均接收频率 | 495.66 Hz | 495.40 Hz |
| 反馈全段平均接收频率 | 249.99 Hz | 249.99 Hz |
| 诊断丢帧 / 序号重启 | 0 / 0 | 0 / 0 |
| 七轴各阶段软件电流裁剪 | 无 | 无 |
| 独立失能回读 | PASS，ID1..8 全部失能 | PASS，ID1..8 全部失能 |

操作者确认：左右分别拖动、同时拖动都可控，无明显突动、串侧运动或跟随积压；
停止后已调用 `/gento/hold_current`，返回 `success: true`，左右大臂保持、不再追赶。

数据补充：

- 首个 applied 相对临近前一帧 feedback，各轴差异最大左侧约 `4.79e-7 rad`、右侧约 `3.50e-7 rad`，
  支持首帧保持进入时姿态；停止前正常 applied 逐帧增量两侧均不超过约 `0.001 rad`。
- 无无效消息、非递增时间戳或两侧 abs 命令。单节点记录器同时接收并保存两侧完整流。
- stop 后首个 IDLE 诊断确认各侧八轴输出归零并缓存失能。
  stop 后 1～3 s 的反馈位置跨度，左侧各轴最大约 `0.000178 rad`，右侧约 `0.000287 rad`，
  与操作者确认的大臂保持表现一致。
- 右小臂 ID8 在 TELEOP 阶段有 96 帧软件裁剪，累计约 `0.1921 s`，请求峰值约 `67.23 mA`，
  上限为 `50 mA`。这些样本的夹爪位置约 `-0.17794～-0.01841 rad`，低于配置下限 `-0.0175 rad`，
  有限位屏障输出。单独保留该边界事件，不写成八轴全程无裁剪，不调整当前参数。

本轮正常双臂同步、遥操及停止保持已有数据和现场观察支持，独立失能回读已补齐并核对：

- 左侧 [left_torque_disable.json](/tmp/skye_factr_trials/dual_teleop_20261027_172941_ljcp6cga/left_torque_disable.json)：
  `2026-10-27 17:37:12 +08:00`，`automated_result=PASS`，ID1..8 全部 `torque_enabled=false`，无错误或分析问题。
- 右侧 [right_torque_disable.json](/tmp/skye_factr_trials/dual_teleop_20261027_172941_ljcp6cga/right_torque_disable.json)：
  `2026-10-27 17:37:19 +08:00`，`automated_result=PASS`，ID1..8 全部 `torque_enabled=false`，无错误或分析问题。
- 操作者确认 J6/J7 方向问题已自行解决，按此确认关闭，助手不再安排该问题的检查或修改。

本轮已测范围内的双臂正常同步、遥操及停止保持验收记为 `PASS`，当前共用控制参数作为已验收基线保留。
少许自行转动、补动及上述夹爪边界事件作为现场现象保留，不再作为本轮待办。
异常路径状态仍按第 9 节分别登记，不由正常流程结论自动覆盖。
