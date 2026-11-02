# 阶段 7.2：单臂重力补偿真机验收清单

状态：`PASS`。设备 A 左、右臂 J1--J7 的 `position_only` 重力补偿检查与停止路径已验收通过；右臂提前 `switch_stop` 依据操作者现场确认，未保存独立记录。
先完成[阶段 7.1](../skye_factr_teleop_development_plan.md#71-完成阶段-252-真机验证与-253-冒烟验收记录)。

本入口只启动一侧 leader，使用生产 `DynamixelDriver`、`ControlCore` 和动力学模型。
`GRAVITY_TEST` 不订阅从臂反馈、同步或走位请求，也不发布 follower 关节命令。
验收模式只输出所选关节的重力项；其余控制项均为零。
本轮设备 A 配置使用 `position_only`，动力学计算为 `RNEA(q, 0, 0)`；
本清单不验收 `position_velocity` 的速度相关补偿效果。

## 1. 操作前确认

- [ ] 7.1 的该侧低电流真机验收有效。USB、接线、标定或机械装配变更时，先按
  [2.5.1 只读清单](phase2_5_readonly_checklist.md)重验受影响项目。
- [ ] 小臂无负载、由操作者支撑，周围人员与线缆已清场，可立即切断舵机电源。
- [ ] 只启动目标侧；没有其他程序占用该侧串口或自动发布模式请求。
- [ ] 核对实体标签、USB by-id、ID 1--8、该侧 `joint_offsets`、`joint_signs`、
  URDF、配置中的 `current_limit_ma` 和 `joint_gravity_comp.mode: position_only`。
  配置须是现场核准的标定文件。
- [ ] 核准本轮 `joint_id`、`gain_scale`、电流上限、渐增时间和最长通电时间。
  `gain_scale` 在 `(0, 1]`，重力检查以配置中的该关节 gravity gain 为基数。
  电流上限不超过 50 mA，且不能高于配置中任一舵机的上限；渐增时间至少 1 秒，
  最长运行时间不超过 30 秒。当前设备 A 模板电流上限均为 50 mA。

本入口的 `set_torque_enabled(True)` 会使能总线全部 8 个舵机，但只给所选关节
非零电流命令；其余 7 个 ID 命令为零。零电流不等于机械制动，操作者须持续支撑小臂。

## 2. 启动与监视

首次测试或更新源码、YAML 后，从工作区目录构建遥操包及其 URDF 依赖，并加载工作区：

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws

source /opt/ros/humble/setup.bash
colcon build --packages-up-to skye_factr_teleop --symlink-install

source install/setup.bash
ros2 pkg executables skye_factr_teleop
```

确认最后一条命令列出 `skye_factr_teleop_driver`。

新开的启动或监视终端也须执行：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/install/setup.bash
```

启动前检查是否仍有程序占用 leader 串口：

```bash
pgrep -af 'factr_teleop|skye_factr_teleop|dynamixel' || true
```

每轮只测一个 ID；切换 ID、参数或 YAML 模式时，先停止节点，再重新启动。以下命令使用
设备 A 的左右臂配置文件；执行前按第 1 节核准对应配置和硬件。
节点启动后应处于 `IDLE`、Torque Disable。两个电流话题均按物理 ID 1--8
排列、单位 mA：`gravity_test_current_ma` 是限幅后的**软件命令值**，
`gravity_test_present_current_ma` 是舵机寄存器 `Present Current(126)` 的
**实际电流回读**，不是关节力矩。驱动在同一次同步读取中取得电流、速度和位置，
命令电流随后才写入；比较两者时应使用渐增结束、姿态静止后的连续样本，
不要逐条当作同一瞬间的目标与反馈。XC330 的电流回读来自电源输入侧，
不能直接按电机相电流换算关节力矩。当前状态话题应只报告该侧 7 个关节的
位置、速度。开始前确认监视端能收到消息。

### 只读自动记录（可选）

在**启动测试节点之前**，另开终端启动记录器。它只订阅四个监视话题，
不发布模式请求、不操作舵机；现有的启动、`switch_gravity_test`、`switch_stop`
和测试后 Torque Disable 回读步骤仍按本清单执行。每轮一个 ID 和一个输出文件；
命令参数必须与该轮 launch 参数一致。

左臂 ID 1 示例：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/install/setup.bash

ros2 run skye_factr_teleop skye_factr_gravity_test_recorder -- \
  --side left --joint-id 1 --operator 'lyy' \
  --config /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --gain-scale 0.1 --current-limit-ma 20 \
  --ramp-seconds 2 --max-duration-seconds 10 \
  --output /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/validation_records/phase7_2_1_A/left_id1_run1.json
```

右臂 ID 1 示例：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/install/setup.bash

ros2 run skye_factr_teleop skye_factr_gravity_test_recorder -- \
  --side right --joint-id 1 --operator 'lyy' \
  --config /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --gain-scale 0.1 --current-limit-ma 20 \
  --ramp-seconds 2 --max-duration-seconds 10 \
  --output /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/validation_records/phase7_2_1_A/right_id1_run1.json
```

返回 `IDLE` 后，记录器自动结束采样，只问操作者三个短答案：
同姿态支撑力 `less/unclear/more`、异常运动 `none/sudden/reverse/other`、
本轮结束方式 `stop/timeout/other`。输出 JSON 是汇总，旁边的 `.trace.jsonl`
保留收到的原始话题消息；另有一项可跳过的现象备注。JSON 中
`present_current_samples` 与目标 ID 回读电流最小／最大值用于核对回读是否存在；
其它 ID 的回读电流可能非零，不使用它代替“非目标 ID 命令电流为零”的检查。
没有实际电流回读样本时，记录器标为 `INCOMPLETE`。JSON 中
`PENDING_SHUTDOWN_CHECK` 仅表示已收到的
话题和人工观察符合前半段要求，**不是**最终 `PASS`：还须确认节点正常退出，
并按第 4 节取得本轮 ID 1--8 Torque Disable 只读回读文件。若未看到完整状态
过程、末尾零命令或必要话题，记录器标为 `INCOMPLETE`；不补填为通过。
`sudden` 指持续轻扶时出现不受手部控制的速度突增或甩动；`reverse` 指同姿态
下补偿明显加重下坠、出现反向牵引；`other` 指非目标关节出现异常动作。
起步阻力大、克服静摩擦后阻力降低，若仍平顺且由手控制，不单凭阻力变化选择
`sudden`。记录器保存每轴最大速度和相邻位置变化，供事后核对，但不以自造的
数值阈值代替现场判断。
力感答案以**同一姿态、同一扶持位置**在进入测试前与渐增结束后的比较为准；
完成这一比较后，仍按第 3 节轻缓改变目标关节姿态并观察运动，记录器会保存
起止位置。若姿态无法保持或力感分辨不出，选 `unclear`。

### 2.1 左臂

先列出 USB by-id：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
```

核对左臂实体标签与 YAML 中的 `dynamixel_port` 均对应
`FTBEO9L1`，再解析实际 tty：

```bash
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0
TTY_PATH=$(readlink -f "$USB_BY_ID")
test -c "$TTY_PATH"
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
```

确认 `test -c` 成功且 tty 对应左臂，然后设置并回读 FTDI latency timer：

```bash
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

确认回读为 `1`，再启动左臂 ID 1、20 mA 上限的测试：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/install/setup.bash

ros2 launch skye_factr_teleop gravity_test_m6_left.launch.py \
  config_path:=/home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  joint_id:=1 \
  gain_scale:=0.1 \
  ramp_seconds:=2.0 \
  current_limit_ma:=20.0 \
  max_duration_seconds:=10.0
```

在三个终端分别监视 当前操作状态、目标电流、当前关节状态：

```bash
ros2 topic echo /teleop/state std_msgs/msg/String
```

```bash
ros2 topic echo /leader_arm/left/gravity_test_current_ma std_msgs/msg/Float64MultiArray
```

```bash
ros2 topic echo /leader_arm/current_state sensor_msgs/msg/JointState
```

若要现场查看实际电流时，另开终端：

```bash
ros2 topic echo /leader_arm/left/gravity_test_present_current_ma std_msgs/msg/Float64MultiArray
```

### 2.2 右臂

先列出 USB by-id：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
```

核对右臂实体标签与 YAML 中的 `dynamixel_port` 均对应
`FTBEO8WL`，再解析实际 tty：

```bash
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0
TTY_PATH=$(readlink -f "$USB_BY_ID")
test -c "$TTY_PATH"
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
```

确认 `test -c` 成功且 tty 对应右臂，然后设置并回读 FTDI latency timer：

```bash
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

确认回读为 `1`，再启动右臂 ID 1、20 mA 上限的测试：

```bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/install/setup.bash

ros2 launch skye_factr_teleop gravity_test_m6_right.launch.py \
  config_path:=/home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  joint_id:=1 \
  gain_scale:=0.1 \
  ramp_seconds:=2.0 \
  current_limit_ma:=20.0 \
  max_duration_seconds:=10.0
```

在三个终端分别监视：

```bash
ros2 topic echo /teleop/state std_msgs/msg/String
```

```bash
ros2 topic echo /leader_arm/right/gravity_test_current_ma std_msgs/msg/Float64MultiArray
```

```bash
ros2 topic echo /leader_arm/current_state sensor_msgs/msg/JointState
```

需要现场查看实际电流时，另开终端：

```bash
ros2 topic echo /leader_arm/right/gravity_test_present_current_ma std_msgs/msg/Float64MultiArray
```

## 3. 逐关节重力补偿检查

确认姿态处于关节安全范围并扶稳小臂后，仅向当前启动的一侧发送一次显式请求。

左臂：

```bash
ros2 topic pub --once /mode/left/switch_gravity_test std_msgs/msg/String '{data: switch_gravity_test}'
```

右臂：

```bash
ros2 topic pub --once /mode/right/switch_gravity_test std_msgs/msg/String '{data: switch_gravity_test}'
```

观察并记录：

1. `/teleop/state` 从 `IDLE` 变为 `GRAVITY_TEST`；没有同步轨迹或 follower 命令。
2. 所选 ID 的补偿增益在 `ramp_seconds` 内从零渐增；命令电流随姿态变化，
   不要求单调增加，但绝对值不得超过本轮上限；其余 ID 的命令电流为零。
3. 扶稳小臂，轻缓改变所选关节姿态后停住；在相同姿态下比较进入测试前后所需的
   手部支撑力，确认补偿辅助抵消重力，而不是加重下坠趋势。不得出现突然加速、
   反向牵引或其它关节异常动作。2.5.1 的只读 `joint_signs` 结果沿用，
   不重复手动方向引导。本轮不做变速移动测试。
4. `max_duration_seconds` 到期后状态回到 `IDLE`，命令电流归零、Torque Disable。

在任一时刻均可先发送 `switch_stop` 结束本轮：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

从较低 `gain_scale` 起逐级提高；每一级都须重新启动节点并保存参数、结果。
某一关节在低上限下无明显辅助力时，记录为 `INCONCLUSIVE`，不得据此判断方向正确，
也不得未经现场批准直接提高电流。按 ID 1--7 逐项完成左右两侧。

### 节点启动后的现场顺序

1. 确认测试节点处于 `IDLE`，记录器已显示 `状态: IDLE`。若记录器尚未启动，
   先保持节点在 `IDLE` 并启动第 2 节的记录器，再发送测试请求。
2. 扶稳小臂，在安全姿态用固定的扶持位置感受测试前维持姿态所需的防下坠支撑力。
3. 用本节上方对应侧的命令发送**一次** `switch_gravity_test`。渐增期间继续扶稳；
   渐增结束后仍保持同一姿态，比较支撑力是减轻、加重还是分辨不出。
4. 完成同姿态比较后，轻缓改变目标关节姿态并停住，观察目标关节是否突动、
   补偿是否反向牵引，以及其它关节是否有异常动作。出现停止条件时立即按第 4 节处理。
5. 可在 `max_duration_seconds` 到期时等待自动返回 `IDLE`，或用上方命令提前
   发送 `switch_stop`。确认状态为 `IDLE`、命令电流全零；记录器结束采样后，
   回答支撑力 `less/unclear/more`、异常运动 `none/sudden/reverse/other`、
   结束方式 `timeout/stop/other`。只有实际提前发送并生效时才填 `stop`。
6. 确认测试节点正常退出；保持舵机供电和小臂支撑，立即按第 4 节对本侧
   ID 1--8 做 Torque Disable 只读回读，保存文件。换 ID 或参数时重新启动节点
   和记录器，并使用新的输出文件名。

## 4. 停止条件与结束检查

方向不符、其它关节动作、超出预期的速度、机械或线缆干涉、通信报错、状态未按时
返回 `IDLE` 时，立即使用现场断电手段并记录为 `FAIL`。正常停止时先发布
`/mode/switch_stop`，确认电流消息为全零、状态为 `IDLE`，再结束进程。Ctrl+C 和
控制异常会调用现有零电流与 Torque Disable 清理路径；进程被强制终止或总线失联时
不能仅凭软件消息断言舵机已失能。退出节点后，按[2.5.2 第 10 节](phase2_5_low_current_checklist.md#10-测试结束后复核-torque-disable)
对本侧 ID 1--8 做只读 Torque Disable 回读，保存独立结果文件。

以下是设备 A **左臂 ID 1、run1** 的可复制回读命令。先确认测试节点已退出，
小臂仍被支撑；保持本轮的舵机供电，不断电重启、不拔插 USB，也不要启动其它
占用串口的控制程序。随后立即执行：

左臂：

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --frames 2 --interval 0 \
  --output src/skye_factr_teleop/validation_records/phase7_2_1_A/left_id1_run1_torque_disable.json
```

右臂：

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side right \
  --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --frames 2 --interval 0 \
  --output src/skye_factr_teleop/validation_records/phase7_2_1_A/right_id1_run1_torque_disable.json
```

终端应显示 `automated result: PASS`。这步中，程序会把 torque_enabled 写入 JSON，但它的 PASS 判定没有检查这些值，因此不保证八个舵机都已失能。

可再用下面的命令核对 JSON 确实包含 ID 1--8，且每个 `torque_enabled` 均为 `false`：

```bash
python3 - <<'PY'
import json
from pathlib import Path

path = Path('src/skye_factr_teleop/validation_records/phase7_2_1_A/left_id1_run1_torque_disable.json')
record = json.loads(path.read_text())
devices = record['devices']
assert record['automated_result'] == 'PASS', record['error']
assert [device['id'] for device in devices] == list(range(1, 9))
assert all(device['torque_enabled'] is False for device in devices)
print(f'ID 1--8 Torque Disable PASS: {path}')
PY
```

其它轮次改用对应侧的 `--side`、配置文件和**独立输出文件名**。
若从测试结束到回读之间已断电重启，本次回读不能证明原轮次的停止结果；
需重做该轮并在同一供电周期立即回读。将节点退出结果和回读文件路径写入该轮记录。

## 5. 验收记录与判定

每一轮至少保存：日期、操作者、侧别、配置文件、补偿模式、USB by-id、URDF、ID、型号、
`gain_scale`、该 ID 配置 gravity gain、电流上限、渐增时间、最长时间、
起止姿态、命令电流峰值、实际施力方向、`switch_stop` 或超时结果、进程退出结果及
测试后 Torque Disable 回读路径。

单轮 `PASS` 要求状态切换正确、仅目标 ID 非零、命令电流不越界、施力方向正确，
且停止和失能回读通过。`INCONCLUSIVE` 不计为通过。左右两侧全部关节的重力检查
和停止路径均通过，才可将本清单标为 `PASS`。本清单通过仅表示
`position_only` 下的重力补偿验收通过；切换到 `position_velocity` 后须另行验收速度相关效果。
7.2 的其他待办以开发计划为准。7.2 完成前不得进入 7.3 真机同步。

## 6. 设备 A 实测记录与判定

### 左臂 J1--J7 新一轮记录（2026-10-12，约 17:02--17:41）

本节仅依据 [phase7_2_1_A](../../validation_records/phase7_2_1_A/) 中的新一轮记录判定；
旧轮次存放于 `phase7_2_1_A_old`，不计入下表。七轮均由 `lyy` 操作，
使用左臂 `FTBEO9L1`、`m6left_isomorphism.urdf`、`position_only`、
2 s 渐增和 10 s 最长测试时间。表中上限与峰值是**软件命令电流**，不是
舵机 `Present Current(126)` 回读，也不是实测关节力矩。

七份汇总和原始轨迹均显示 `IDLE → GRAVITY_TEST → IDLE`，仅目标 ID 有
非零命令电流，命令峰值不超过该轮上限，结束后观察到全零命令；每轮均有
目标 ID 的实际电流回读。操作者七轮都记录同姿态支撑力“减轻”、无异常运动。
操作者补充确认：J1--J6 在 10 s 超时、J7 在发送 `stop` 后返回 `IDLE`；
七轮测试节点进程随后均正常退出，测试至失能回读期间持续供电、USB 未拔插，
也没有其它程序控制左臂。这些现场确认补足了汇总文件未记录的退出与供电信息。

| 目标关节／新记录 | 型号 | 配置 gravity gain × gain_scale | 命令上限／峰值 (mA) | 结束方式／测试时长 | 独立失能回读 | 本轮判定 |
| --- | --- | --- | ---: | --- | --- | --- |
| [J1](../../validation_records/phase7_2_1_A/left_id1_run1.json) | XM430-W350 | 0.4 × 0.1 | 20／15.69 | 超时／10.012 s | [8/8 失能](../../validation_records/phase7_2_1_A/left_id1_run1_torque_disable.json) | **PASS** |
| [J2](../../validation_records/phase7_2_1_A/left_id2_run1.json) | XM430-W350 | 0.2 × 0.1 | 20／4.51 | 超时／10.014 s | [8/8 失能](../../validation_records/phase7_2_1_A/left_id2_run1_torque_disable.json) | **PASS** |
| [J3](../../validation_records/phase7_2_1_A/left_id3_run1.json) | XC330-T288 | 0.7 × 0.2 | 30／30.00 | 超时／10.012 s | [8/8 失能](../../validation_records/phase7_2_1_A/left_id3_run1_torque_disable.json) | **PASS** |
| [J4](../../validation_records/phase7_2_1_A/left_id4_run1.json) | XC330-T288 | 0.6 × 0.2 | 30／29.80 | 超时／10.011 s | [8/8 失能](../../validation_records/phase7_2_1_A/left_id4_run1_torque_disable.json) | **PASS** |
| [J5](../../validation_records/phase7_2_1_A/left_id5_run1.json) | XC330-T288 | 1.0 × 0.9 | 30／30.00 | 超时／10.011 s | [8/8 失能](../../validation_records/phase7_2_1_A/left_id5_run1_torque_disable.json) | **PASS** |
| [J6](../../validation_records/phase7_2_1_A/left_id6_run1.json) | XC330-T288 | 0.8 × 1.0 | 30／25.25 | 超时／10.013 s | [8/8 失能](../../validation_records/phase7_2_1_A/left_id6_run1_torque_disable.json) | **PASS** |
| [J7](../../validation_records/phase7_2_1_A/left_id7_run1.json) | XC330-T288 | 1.0 × 1.0 | 30／30.00 | 操作者确认 `switch_stop`／9.750 s | [8/8 失能](../../validation_records/phase7_2_1_A/left_id7_run1_torque_disable.json) | **PASS** |

J1--J7 的七份独立只读回读均显示左臂 ID 1--8 的
`torque_enabled=false`，自动结果为 `PASS`。七份测试汇总中的
`process_exit` 均为 `PENDING`，`torque_disable_readback_path` 均为
`null`；上述进程、供电和 USB 状态依据操作者的补充确认，而非汇总 JSON
字段。J7 的回读开始于 17:48:50，距本轮返回 `IDLE` 约 7 分 14 秒。
尽管操作者确认这段时间供电未中断、USB 未拔插，也没有其它控制程序，
这次回读仍晚于第 4 节要求的“立即回读”。此时间差作为操作偏差记录；
结合上述现场确认和 8/8 失能回读，本轮判为 `PASS`。

J7 的状态在约 9.750 s 返回 `IDLE`，早于配置的 10 s 最长时间。操作者
确认使用以下命令提前停止；轨迹与该操作一致，但未记录模式请求本身：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

### 左臂验收结论

**本轮左臂 J1--J7 均判为单轮 `PASS`。**
七轮均有辅助方向正确、命令限幅、状态返回和全零命令的证据，且失能回读
均为 8/8；J7 还验证了提前停止路径。左臂本轮逐关节重力补偿检查通过。

### 右臂 J1--J7 新一轮记录（2026-10-12，约 18:18--19:00）

本节依据 [phase7_2_1_A](../../validation_records/phase7_2_1_A/) 内右臂七份新汇总、
原始轨迹及独立失能回读。七轮均由 `lyy` 操作，使用右臂 `FTBEO8WL`、
`m6right_isomorphism.urdf`、`position_only`、2 s 渐增和 10 s 最长测试时间。
表中上限与峰值是软件命令电流，不是实测关节力矩。

| 目标关节／新记录 | 型号 | 配置 gravity gain × gain_scale | 命令上限／峰值 (mA) | 结束方式／测试时长 | 独立失能回读 | 本轮判定 |
| --- | --- | --- | ---: | --- | --- | --- |
| [J1](../../validation_records/phase7_2_1_A/right_id1_run1.json) | XM430-W350 | 0.4 × 0.1 | 20／15.22 | 超时／10.011 s | [8/8 失能](../../validation_records/phase7_2_1_A/right_id1_run1_torque_disable.json) | **PASS** |
| [J2](../../validation_records/phase7_2_1_A/right_id2_run1.json) | XM430-W350 | 0.2 × 0.5 | 20／19.72 | 超时／10.010 s | [8/8 失能](../../validation_records/phase7_2_1_A/right_id2_run1_torque_disable.json) | **PASS** |
| [J3](../../validation_records/phase7_2_1_A/right_id3_run1.json) | XC330-T288 | 0.7 × 0.2 | 30／30.00 | 超时／10.011 s | [8/8 失能](../../validation_records/phase7_2_1_A/right_id3_run1_torque_disable.json) | **PASS** |
| [J4](../../validation_records/phase7_2_1_A/right_id4_run1.json) | XC330-T288 | 0.6 × 0.2 | 30／28.57 | 超时／10.013 s | [8/8 失能](../../validation_records/phase7_2_1_A/right_id4_run1_torque_disable.json) | **PASS** |
| [J5](../../validation_records/phase7_2_1_A/right_id5_run1.json) | XC330-T288 | 1.0 × 0.9 | 30／30.00 | 超时／10.012 s | [8/8 失能](../../validation_records/phase7_2_1_A/right_id5_run1_torque_disable.json) | **PASS** |
| [J6](../../validation_records/phase7_2_1_A/right_id6_run1.json) | XC330-T288 | 0.8 × 1.0 | 30／24.68 | 超时／10.012 s | [8/8 失能](../../validation_records/phase7_2_1_A/right_id6_run1_torque_disable.json) | **PASS** |
| [J7](../../validation_records/phase7_2_1_A/right_id7_run1.json) | XC330-T288 | 1.0 × 1.0 | 30／30.00 | 超时／10.013 s | [8/8 失能](../../validation_records/phase7_2_1_A/right_id7_run1_torque_disable.json) | **PASS** |

七份汇总及原始轨迹均显示 `IDLE → GRAVITY_TEST → IDLE`、仅目标 ID 有非零
命令电流且未超本轮上限、结束后全零命令，并有目标 ID 的实际电流回读。
操作者七轮均记录同姿态支撑力“减轻”、无异常运动，结束方式均记为超时。
七份独立只读回读的自动结果均为 `PASS`，每份均显示右臂 ID 1--8 的
`torque_enabled=false`。原始汇总仍将 `process_exit` 记为 `PENDING`，
`torque_disable_readback_path` 为 `null`；操作者补充确认：七轮测试节点均
正常退出，从每轮测试结束到独立失能回读期间持续供电、USB 未拔插，且没有
其它程序控制右臂。这些现场确认补足了汇总文件未记录的退出与供电信息。

#### J3 过载告警的理解与复测

J3 在本批测试前曾出现 `Hardware Error Status(70)=0x20` 过载告警，
18:38 的旧失能检查失败。[ROBOTIS XC330-T288 手册](https://emanual.robotis.com/docs/en/dxl/x/xc330-t288/)
将 `0x20` 定义为持续负载超过舵机输出能力；触发停机保护时，舵机会清除
`Torque Enable(64)` 并停止输出力矩。这不表示软件命令电流超过本轮上限，
也不能单凭一次告警判定电机已经损坏。

操作者补充：告警前，J1、J2 关节去使能后，机械臂从零姿态快速回到
自然下垂姿态，其余节段也快速下垂。这一过程可能使 J3 承受过大负载，
但现有记录不能确认唯一原因；快速下垂也可能对传动件造成机械冲击。
手册提醒，长时间施加高电流可能损伤电机，因此保护动作本身不保证
舵机和传动件没有损伤。

当前同名 J3 文件已由约 18:45 的新测试覆盖。新一轮完成了 10 s 测试，
操作者记录无异常运动；18:46 的独立回读为 `PASS`，ID 1--8 均失能。
这支持 J3 目前没有明显功能故障，但不能排除隐性损伤；后续留意新异响、
卡滞、齿隙明显增大或过载告警再次出现。此处仅按当前新记录判定，
不把此前失败的回读当作本轮失能证明。

### 双臂阶段结论

**左、右臂 J1--J7 本轮均判为单轮 `PASS`。** 两侧全部关节的方向、命令限幅、
状态返回、全零命令和独立失能回读均有通过证据。左臂 J7 已验证提前
`switch_stop`。右臂表中七轮均为超时结束；操作者另行确认右臂提前
`switch_stop` 已现场实测有效，但没有保存该次测试的原始轨迹或独立记录。
依据此现场确认，设备 A 双臂在 `position_only` 下的本清单验收结论为 `PASS`。
