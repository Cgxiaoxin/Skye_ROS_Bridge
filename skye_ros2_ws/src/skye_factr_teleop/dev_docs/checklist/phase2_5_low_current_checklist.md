# 阶段 2.5.2：单舵机低电流验证清单

本文用于阶段 2.5.2。连接硬件前先完成第 1--3 节；第 4 节之后才包含真实串口写入。
本工具只调用生产 `DynamixelDriver`，不启动控制核心或 ROS 遥操节点。
本轮使用遥操设备 A（Thor）：左侧 USB by-id 为 `FTBEO9L1`，右侧为 `FTBEO8WL`；
两侧命令使用 `config/grav_comp_m6_*.yaml`，不得混用遥操设备 B（Orin）的串口或预演记录。

## 当前进度（2026-10-10）

| 小节 | 状态 | 依据或待办 |
| --- | --- | --- |
| 1. 覆盖范围与硬限制 | PASS（静态核对） | 左 ID 1 为 XM430-W350，`20 ≤ 50 ≤ 100 mA`；右 ID 3 为 XC330-T288，`20 ≤ 50 ≤ 91 mA`。现场批准仍按第 4 节确认。 |
| 2. 无硬件自动测试 | PASS | 2026-10-10 复测 `test_dynamixel_driver.py` 与 `test_low_current_check.py`，合计 60 项通过，覆盖中断与通信失败清理逻辑。 |
| 3. 连接前参数预演 | PASS | 设备 A 左右配置的 `validation_records/phase2_5_2_A/left_id1_prepared.json` 与 `right_id3_prepared.json` 均为 `PREPARED`。 |
| 4. 上电和连接前现场确认 | PASS（操作者确认） | 操作者确认阶段 2.5.2 已完成；JSON 可证明测试前目标舵机 Torque Disable，其余现场项目未逐项留痕。 |
| 5. 真实单舵机验证 | PASS（正常路径） | 设备 A 左右 ID 1--8 共 16 份真机结果均为 `PASS`；操作者曾用 Ctrl+C 中断工具，但未保存该次结果。 |
| 6. 通过与停止条件 | PASS（含人工确认） | 左侧最终 Torque Disable 有回读；右侧最后一次测试后的状态由操作者现场确认，既有右侧回读早于该次测试。异常路径软件故障注入测试通过。 |
| 7. 验收记录 | PASS | 16 份真机 JSON 和两份 Torque Disable 回读已保存；右侧最终状态的人工确认及异常路径覆盖范围见[2.5.3 冒烟记录](../skye_factr_teleop_development_plan.md#253-冒烟验收记录)。 |

`PREPARED` 只证明参数预演通过，不代表舵机已连接、运动方向已验证或 Torque Disable
已在真机上确认。第 4、6、7 节的现场结果已汇总在
[开发计划阶段 2.5.3](../skye_factr_teleop_development_plan.md#253-冒烟验收记录)。

## 1. 覆盖范围与硬限制

最小验收矩阵如下，同时覆盖左右总线和两种实际型号：

| 侧别 | ID | 配置型号 | 配置文件 |
| --- | ---: | --- | --- |
| left | 1 | XM430-W350 | `config/grav_comp_m6_left.yaml` |
| right | 3 | XC330-T288 | `config/grav_comp_m6_right.yaml` |

如实体接线标识要求换用同型号的其它 ID，可以替换 ID，但必须重新运行
`--prepare-only` 并在记录中保存实际 ID。

### 三层电流约束（执行前确认）

| 层次 | 参数或检查 | 作用 |
| --- | --- | --- |
| 1. 实际脉冲 | `--test-current-ma` | 发送 `+测试电流` 和 `-测试电流`；例如 `20 mA`。 |
| 2. 本轮软件上限 | `--current-limit-ma` | 操作者事先批准的最大幅值；测试电流不得超过它，例如 `50 mA`。 |
| 3. 工具硬上限 | `min(100 mA, 型号控制表 Current Limit(38) 最大允许值的 10%)` | 本轮软件上限也不得超过此值：XM430-W350 为 `100 mA`，XC330-T288 为 `91 mA`。 |

因此，执行前必须满足：`测试脉冲幅值 ≤ 本轮软件上限 ≤ 工具硬上限`。先由现场负责人
结合负载、姿态和可断电条件确定 `--current-limit-ma`，再选择
`--test-current-ma`。第 3 节的 `50 mA` / `20 mA` 是预演示例，并非必须使用的数值。
`--pulse-seconds` 指定每次正、负电流脉冲的持续时间，最长 2 秒。

生产驱动还会按型号把 mA 换算成寄存器整数、检查取整后的值，并拒绝超过该型号控制表
允许范围的命令。工具也检查取整后的脉冲幅值不超过本轮软件上限。

> **帮助理解：10% 从哪里来？** XM430-W350 的控制表最大值是
> `1193 × 2.69 = 3209.17 mA`，其 10% 超过 `100 mA`，所以工具取 `100 mA`；
> XC330-T288 是 `910 × 1 = 910 mA`，其 10% 为 `91 mA`。
> 这个 10% 是本项目的保守冒烟策略，不是厂家规定的连续额定电流或安全工作电流。
>
> **帮助理解：EEPROM 与软件上限。** EEPROM 是断电后仍保存配置的存储区。
> `--current-limit-ma` 仅在本次程序中生效，不会写入舵机的 EEPROM
> Current Limit(38) 寄存器；真实测试会将同属 EEPROM 的 Operating Mode(11)
> 写为 Current Control Mode。

## 2. 无硬件自动测试

在 `skye_ros2_ws/src/skye_factr_teleop` 下运行：

```bash
python3 -m pytest -q \
  test/test_dynamixel_driver.py \
  test/test_low_current_check.py
```

测试必须通过以下契约：

- Torque Disable → Current Control Mode → 零 Goal Current → Torque Enable；
- XM430/XC330 电流换算、signed 16-bit 编码和限幅；
- `write_currents()` 无论成功或失败都调用 `clearParam()`；
- 正常关闭和零电流写失败时仍尝试 Torque Disable 并关闭端口；
- 冒烟工具只构造单 ID 驱动并按正、零、负、零顺序写入；
- 通信失败和中断路径进入生产驱动的安全关闭流程。

## 3. 连接前参数预演

首次使用本工具或更新源码后，先从 `skye_ros2_ws` 目录重新构建包，并在当前终端加载工作区：

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
colcon build --packages-select skye_factr_teleop --symlink-install
source install/setup.bash
ros2 pkg executables skye_factr_teleop
# 根据设备更换为A/B后缀
RECORD_ROOT=/home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/validation_records/phase2_5_2_A
mkdir -p "$RECORD_ROOT"
```

确认最后一条命令列出 `skye_factr_dynamixel_low_current_check`。新开终端后需重新执行
`source install/setup.bash`，才能通过 `ros2 run` 找到该包和命令。

以下 `50 mA` 限值和 `20 mA` 测试电流是保守的预演参数。真实测试前由现场负责人确认；
若需要修改，仍必须低于第 1 节硬上限。

从 `skye_ros2_ws` 目录运行左侧 XM430 预演：

```bash
# 根据设备更换为A/B后缀
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --prepare-only --side left --id 1 \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2_A/left_id1_prepared.json
```

运行右侧 XC330 预演：

```bash
# 根据设备更换为A/B后缀
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --prepare-only --side right --id 3 \
  --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2_A/right_id3_prepared.json
```

两条命令必须显示 `result: PREPARED`。检查 JSON 中的端口、型号、ID、电流单位、raw 值、
编码、操作者限值和硬上限。`--prepare-only` 不构造驱动、不解析 `/dev/serial/by-id`，也不
打开串口。

## 4. 上电和连接前现场确认

- [ ] 阶段 2.5.1 对应总线 `bus_result` 为 `PASS`；
- [ ] 大臂运动链路已断开或保持停机；
- [ ] 已停止所有可能占用 leader 串口的 FACTR/Dynamixel 进程；
- [ ] 当前只连接目标舵机，或已确认工具只会寻址并使能目标 ID；
- [ ] 舵机脱离会碰撞人员、线缆或设备的负载位置；
- [ ] 操作者可立即切断舵机电源；
- [ ] USB by-id、侧别、型号、ID、本次限值和测试电流已与预演 JSON 一致；
- [ ] 现场负责人已批准本次低电流数值和 0.5 秒脉冲；
- [ ] 未启动控制核心或 ROS 遥操节点。

可保留现有 Dynamixel 菊花链接线，但同侧所有舵机会一起上电。若保留整条总线，先独立
确认非目标 ID 均为 Torque Disable、没有其它程序发送控制命令，并支撑好可能下垂的机械臂；
本工具只检查目标 ID，不会核对其它舵机的 Torque 状态。若要物理隔离，仅在断电后断开
其它舵机的连接，保持目标舵机的通信和外部供电；不得带电插拔。两次验证之间同样先断电。

只读确认串口占用：

```bash
pgrep -af 'factr_teleop|skye_factr_teleop|dynamixel' || true
```

## 5. 真实单舵机验证

只有第 4 节全部确认后，才把预演命令中的 `--prepare-only` 替换为
`--confirm-single-servo`，并把输出文件名改为 `*_result.json`。以下命令均从
`skye_ros2_ws` 目录、已加载工作区的同一终端执行。

先确认左侧 USB by-id 名称及其对应的实体设备。

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
```

核对 `USB_BY_ID` 与左侧 YAML 的 `dynamixel_port` 和实体标签一致：

```bash
# 设备A
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0

TTY_PATH=$(readlink -f "$USB_BY_ID")
test -c "$TTY_PATH"
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
```

确认 `test -c` 成功，且 `TTY_PATH` 指向预期设备后，再设置并回读 latency timer：

```bash
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

确认 latency timer 回读为 `1`，再运行左侧 ID 1（XM430-W350）验证：

```bash
# 根据设备更换为A/B后缀
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --confirm-single-servo --side left --id 1 \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2_A/left_id1_result.json
```

两次验证之间先断电，并完成第 4 节对应侧别的全部检查。

重新确认右侧 USB by-id 名称及其对应的实体设备。

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
```

核对 `USB_BY_ID` 与右侧 YAML 的 `dynamixel_port` 和实体标签一致：

```bash
# 设备A
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0

TTY_PATH=$(readlink -f "$USB_BY_ID")
test -c "$TTY_PATH"
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
```

确认 `test -c` 成功，且 `TTY_PATH` 指向预期设备后，再设置并回读 latency timer：

```bash
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

确认 latency timer 回读为 `1`，再运行右侧 ID 3（XC330-T288）验证：

```bash
# 根据设备更换为A/B后缀
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --confirm-single-servo --side right --id 3 \
  --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2_A/right_id3_result.json
```

两侧工具内部的执行顺序相同：

1. 单 ID 只读预检并核对实际 model number；
2. 关闭只读连接；
3. 生产驱动执行 Torque Disable → Current Control Mode → 零 Goal Current → Torque Enable；
4. 读取初始位置；
5. 短时正电流 → 零电流 → 读取位置；
6. 短时负电流 → 零电流 → 读取位置；
7. 比较两次位置增量是否方向相反；
8. 生产关闭路径再次写零电流、Torque Disable、关闭串口。

## 6. 通过与停止条件

通过条件：

- JSON 顶层 `result` 为 `PASS`；
- `initialization.result`、`direction.result`、`shutdown.result` 均为 `PASS`；
- 实际 model number 与配置型号匹配；
- `command_log` 严格为 positive、zero_after_positive、negative、zero_after_negative；
- 操作者断电前再次确认舵机可自由移动，即 Torque Disable。

出现以下任一情况立即断电，不自动提高电流后重试：

- 非目标舵机动作、方向异常、突然加速、机械干涉或线缆受力；
- 型号、ID、USB 侧别或电流记录不符；
- 工具返回 `FAIL`、`INCONCLUSIVE`、通信错误或安全关闭失败；
- Ctrl-C/SIGTERM 后无法确认 Torque Disable。

`INCONCLUSIVE` 表示低电流未产生足够的位置变化。应先断电检查负载、姿态和批准的电流
范围，再决定是否重新预演和测试，不得在原进程中自动增大电流。

## 7. 验收记录

每侧保存对应 JSON，并记录：

- USB by-id；
- 舵机型号、model number 与 ID；
- 波特率和 latency timer；
- 操作者限值、硬上限、测试电流及 raw 编码；
- 正负电流的位置增量和方向结果；
- 初始化、命令序列、异常信息和安全关闭结果。

阶段 2.5.2 的硬件验收只有在 XM430 和 XC330 各至少一台真实舵机通过、左右侧记录均已
保存且所有测试结束后舵机均为 Torque Disable 时完成。

## 8. 设备 A 左臂实测记录（2026-10-10）

使用左侧 `FTBEO9L1`、`--current-limit-ma 50`、`--pulse-seconds 0.5` 和
`--settle-seconds 0.2`，逐 ID 运行单舵机低电流工具。下表的电流是测试命令值；
`PASS` 表示本次正、负脉冲后的位移均达到阈值且方向相反，不代表最小启动电流或重力补偿所需电流。

| ID | 型号 | 已观察到 `PASS` 的测试电流 | 较低档位的结果 | 正/负脉冲位移（rad，当前 JSON） | 当前保存的结果文件 |
| ---: | --- | --- | --- | --- | --- |
| 1 | XM430-W350 | 20 mA | 未测试 | +0.0874 / -0.3743 | [`left_id1_result.json`](../../validation_records/phase2_5_2_A/left_id1_result.json)（20 mA） |
| 2 | XM430-W350 | 20 mA | 未测试 | +0.1166 / -0.3528 | [`left_id2_result.json`](../../validation_records/phase2_5_2_A/left_id2_result.json)（20 mA） |
| 3 | XC330-T288 | 20、30 mA | 10 mA：`INCONCLUSIVE` | +1.0692 / -1.2134 | [`left_id3_result.json`](../../validation_records/phase2_5_2_A/left_id3_result.json)（30 mA） |
| 4 | XC330-T288 | 30 mA | 20 mA：`INCONCLUSIVE`（两次） | +0.0414 / -0.0920 | [`left_id4_result.json`](../../validation_records/phase2_5_2_A/left_id4_result.json)（30 mA） |
| 5 | XC330-T288 | 20 mA | 未测试 | +0.0629 / -0.2040 | [`left_id5_result.json`](../../validation_records/phase2_5_2_A/left_id5_result.json)（20 mA） |
| 6 | XC330-T288 | 30 mA | 20 mA：`INCONCLUSIVE` | +0.4387 / -0.9219 | [`left_id6_result.json`](../../validation_records/phase2_5_2_A/left_id6_result.json)（30 mA） |
| 7 | XC330-T288 | 20 mA | 未测试 | +0.0230 / -0.0368 | [`left_id7_result.json`](../../validation_records/phase2_5_2_A/left_id7_result.json)（20 mA） |
| 8 | XC330-T288 | 30 mA | 20 mA：`INCONCLUSIVE` | +0.8912 / -0.7915 | [`left_id8_result.json`](../../validation_records/phase2_5_2_A/left_id8_result.json)（30 mA） |

位移列依次为 `positive_delta_rad / negative_delta_rad`：正脉冲后位置减初始位置、
负脉冲后位置减正脉冲后位置。数值来自当前保存的 JSON，不包含已被覆盖的测试。

当前八份 JSON 的 `result`、`initialization.result`、`direction.result` 和
`shutdown.result` 均为 `PASS`。同名文件被后续测试覆盖：ID 3 的 20 mA `PASS` 和各次
`INCONCLUSIVE` 仅保留在现场终端输出中，当前 JSON 不含这些测试的详细位移。
各 ID 通过档位不同，可能与负载、姿态、静摩擦或采样时刻有关；这些记录不足以确定具体原因。
ID 3、6、8 在当前 30 mA 记录中的单次位置增量最大分别约为 1.21、0.92、0.89 rad，
后续测试仍需留意机械行程。右侧结果见下节；整体硬件验收仍须现场确认测试结束后
舵机均为 Torque Disable。

## 9. 设备 A 右臂实测记录（2026-10-10）

使用右侧 `FTBEO8WL`、`--current-limit-ma 50`、`--pulse-seconds 0.5` 和
`--settle-seconds 0.2`，逐 ID 运行单舵机低电流工具。下表列出已观察到 `PASS` 的
测试命令电流，不代表最小启动电流或重力补偿所需电流。

| ID | 型号 | 已观察到 `PASS` 的测试电流 | 其它观察 | 正/负脉冲位移（rad，当前 JSON） | 当前保存的结果文件 |
| ---: | --- | --- | --- | --- | --- |
| 1 | XM430-W350 | 20 mA | 无较低档位记录 | +0.0476 / -0.2746 | [`right_id1_result.json`](../../validation_records/phase2_5_2_A/right_id1_result.json)（20 mA） |
| 2 | XM430-W350 | 20 mA | 无较低档位记录 | +0.0982 / -0.3973 | [`right_id2_result.json`](../../validation_records/phase2_5_2_A/right_id2_result.json)（20 mA） |
| 3 | XC330-T288 | 20 mA（两次） | 20 mA 另有一次 `INCONCLUSIVE` | +0.0905 / -0.1166 | [`right_id3_result.json`](../../validation_records/phase2_5_2_A/right_id3_result.json)（20 mA） |
| 4 | XC330-T288 | 30 mA（两次） | 20 mA：`INCONCLUSIVE` | +0.0568 / -0.1058 | [`right_id4_result.json`](../../validation_records/phase2_5_2_A/right_id4_result.json)（30 mA） |
| 5 | XC330-T288 | 30 mA | 20 mA：`INCONCLUSIVE` | +0.8023 / -0.8176 | [`right_id5_result.json`](../../validation_records/phase2_5_2_A/right_id5_result.json)（30 mA） |
| 6 | XC330-T288 | 20 mA | 无较低档位记录 | +0.1120 / -0.1718 | [`right_id6_result.json`](../../validation_records/phase2_5_2_A/right_id6_result.json)（20 mA） |
| 7 | XC330-T288 | 30 mA | 20 mA：`INCONCLUSIVE` | +0.3543 / -0.8912 | [`right_id7_result.json`](../../validation_records/phase2_5_2_A/right_id7_result.json)（30 mA） |
| 8 | XC330-T288 | 30 mA | 20 mA：`INCONCLUSIVE` | +0.7455 / -0.5952 | [`right_id8_result.json`](../../validation_records/phase2_5_2_A/right_id8_result.json)（30 mA） |

当前八份右臂 JSON 的 `result`、`initialization.result`、`direction.result` 和
`shutdown.result` 均为 `PASS`。较早的 `INCONCLUSIVE` 结果已被同名文件覆盖，
仅保留在现场终端输出中。ID 3 在相同 20 mA 下既有 `PASS` 也有 `INCONCLUSIVE`，
因此不能据此断定 20 mA 每次都能产生足够位移。ID 5、7、8 在当前 30 mA 记录中的
单次位置增量最大分别约为 0.82、0.89、0.75 rad，后续测试仍需留意机械行程。
左右侧 JSON 均已保存；整体硬件验收仍须现场确认测试结束后舵机均为 Torque Disable，
并满足第 6 节的其他通过条件。

## 10. 测试结束后复核 Torque Disable

低电流工具退出后，**保持舵机供电**，不要断电重启、拔插 USB 或启动其它控制程序；
在当前供电周期内立即读取 Torque Enable(64)。该寄存器为 `0` 才表示 Torque Disable。
手推关节感觉可自由移动不能代替寄存器回读，因为零电流命令也可能让已使能的舵机
看起来阻力很小。若测试后已经断电重启，当前读数不能证明上一次测试的关闭结果，
需重新执行低电流测试并立即复核。保持机械臂支撑和急停可用。

### 10.1 同侧 ID 1--8 均在线

在 `skye_ros2_ws` 目录执行；左侧测试完成后先使用 `SIDE=left`，右侧测试完成后改为
`SIDE=right`，分别保存记录。若 USB 曾重新插拔，须先按第 5 节重新设置并回读
FTDI latency timer 为 `1`，然后重新测试并复核。

```bash
# 根据设备更换为A/B后缀
RECORD_ROOT=/home/lyy/projects/skye_ros_bridge/skye_ros2_ws/src/skye_factr_teleop/validation_records/phase2_5_3_A
mkdir -p "$RECORD_ROOT"

cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
SIDE=left  # 复核右侧时改为 right
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side "$SIDE" \
  --config "src/skye_factr_teleop/config/grav_comp_m6_${SIDE}.yaml" \
  --frames 2 --interval 0 \
  --output "src/skye_factr_teleop/validation_records/phase2_5_3_A/${SIDE}_torque_after_test.json"
```

工具使用生产 `DynamixelDriver.open_read_only()`，读取每个 ID 的 Torque Enable；
若某个 ID 仍为 Torque On，命令会报该 ID 并返回 `FAIL`，不会为了复核而修改舵机寄存器。
终端必须显示 `automated result: PASS`。再检查保存的 JSON：`devices` 应包含当前侧
ID 1--8，且每项 `torque_enabled` 均为 `false`。可用以下命令判定：

```bash
python3 - "$SIDE" <<'PY'
import json
import sys
from pathlib import Path

side = sys.argv[1]
path = Path(f"src/skye_factr_teleop/validation_records/phase2_5_3_A/{side}_torque_after_test.json")
record = json.loads(path.read_text())
devices = record["devices"]
assert record["automated_result"] == "PASS", record["error"]
assert [item["id"] for item in devices] == list(range(1, 9))
assert all(item["torque_enabled"] is False for item in devices)
print(f"{side}: ID 1--8 Torque Disable PASS; record: {path}")
PY
```

若只读命令失败或有任一 ID 无法确认 Torque Disable，停止后续硬件测试，支撑好机械臂并
按现场安全流程断电；保存失败记录，不以 `shutdown.result=PASS` 代替寄存器回读。

### 10.2 物理隔离，仅目标舵机在线

第 10.1 节的工具固定要求 ID 1--8 全部在线，因此只有单台舵机连接时不能使用。
仍保持该舵机供电，可用生产驱动的单 ID 只读入口复核。以下以设备 A 左侧 ID 1 为例；
测试其它 ID 时，把命令末尾的侧别、ID、型号和 USB by-id 一起改为实际值。

```bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source install/setup.bash
python3 - left 1 XM430-W350 usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0 <<'PY'
import json
import sys
from datetime import datetime
from pathlib import Path
from skye_factr_teleop.dynamixel_driver import DynamixelDriver

side, id_text, model, port = sys.argv[1:]
device_id = int(id_text)
record = {"checked_at": datetime.now().astimezone().isoformat(timespec="seconds"),
          "side": side, "id": device_id, "model": model, "port": port, "result": "FAIL"}
driver = DynamixelDriver(ids=(device_id,), servo_types=(model,), port=port)
try:
    driver.open_read_only()
    info = driver.device_info[0]
    expected_model = {"XM430-W350": 1020, "XC330-T288": 1220}[model]
    if info.model_number != expected_model:
        raise RuntimeError(f"ID {device_id} model {info.model_number}, expected {expected_model}")
    record.update(model_number=info.model_number,
                  torque_enabled=info.torque_enabled, result="PASS")
except Exception as exc:
    record["error"] = str(exc)
finally:
    driver.close()
path = Path(f"src/skye_factr_teleop/validation_records/phase2_5_3_A/{side}_id{device_id}_torque_after_test.json")
path.parent.mkdir(parents=True, exist_ok=True)
path.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n")
print(record)
print(f"record: {path}")
if record["result"] != "PASS":
    raise SystemExit(1)
PY
```

单 ID 只读入口也会在 Torque Enable 不为 `0` 时失败并报告设备 ID。将对应 JSON 与
本次低电流结果一同保存；各侧被测 ID 均须在其测试退出后、断电前分别复核。
