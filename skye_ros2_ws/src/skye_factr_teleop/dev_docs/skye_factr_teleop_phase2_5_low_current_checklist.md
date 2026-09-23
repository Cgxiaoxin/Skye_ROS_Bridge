# 阶段 2.5.2：单舵机低电流验证清单

本文用于阶段 2.5.2。连接硬件前先完成第 1--3 节；第 4 节之后才包含真实串口写入。
本工具只调用生产 `DynamixelDriver`，不启动控制核心或 ROS 遥操节点。

## 当前进度（2026-09-23）

| 小节 | 状态 | 依据或待办 |
| --- | --- | --- |
| 1. 覆盖范围与硬限制 | PASS（静态核对） | 左 ID 1 为 XM430-W350，`20 ≤ 50 ≤ 100 mA`；右 ID 3 为 XC330-T288，`20 ≤ 50 ≤ 91 mA`。现场批准仍按第 4 节确认。 |
| 2. 无硬件自动测试 | PASS | `test_dynamixel_driver.py` 与 `test_low_current_check.py` 合计 59 项通过。 |
| 3. 连接前参数预演 | PASS | ROS 2 已发现低电流工具；`validation_records/phase2_5_2/left_id1_prepared.json` 与 `right_id3_prepared.json` 均为 `PREPARED`。 |
| 4. 上电和连接前现场确认 | PENDING | 需现场逐项确认。 |
| 5. 真实单舵机验证 | PENDING | 左右串口映射、latency timer 和真实电流脉冲尚未验收。 |
| 6. 通过与停止条件 | PENDING | 需根据真实测试结果判定。 |
| 7. 验收记录 | PENDING | 尚无真实测试结果记录。 |

`PREPARED` 只证明参数预演通过，不代表舵机已连接、运动方向已验证或 Torque Disable
已在真机上确认。第 4--7 节延期到[开发计划阶段 7 任务 7.1](skye_factr_teleop_development_plan.md)
完成；通过前不得执行基础硬件运动验收。

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
colcon build --packages-select skye_factr_teleop --symlink-install
source install/setup.bash
ros2 pkg executables skye_factr_teleop
```

确认最后一条命令列出 `skye_factr_dynamixel_low_current_check`。新开终端后需重新执行
`source install/setup.bash`，才能通过 `ros2 run` 找到该包和命令。

以下 `50 mA` 限值和 `20 mA` 测试电流是保守的预演参数。真实测试前由现场负责人确认；
若需要修改，仍必须低于第 1 节硬上限。

从 `skye_ros2_ws` 目录运行左侧 XM430 预演：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --prepare-only --side left --id 1 \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2/left_id1_prepared.json
```

运行右侧 XC330 预演：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --prepare-only --side right --id 3 \
  --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2/right_id3_prepared.json
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

只读确认串口占用：

```bash
pgrep -af 'factr_teleop|skye_factr_teleop|dynamixel' || true
```

## 5. 真实单舵机验证

只有第 4 节全部确认后，才把预演命令中的 `--prepare-only` 替换为
`--confirm-single-servo`，并把输出文件名改为 `*_result.json`。以下命令均从
`skye_ros2_ws` 目录、已加载工作区的同一终端执行。

先确认左侧 USB by-id 名称及其对应的实体设备。当前左侧配置使用遥操设备 B；核对
`USB_BY_ID` 与左侧 YAML 的 `dynamixel_port` 和实体标签一致，再设置并回读 latency timer：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTB8HNOT-if00-port0
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
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --confirm-single-servo --side left --id 1 \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2/left_id1_result.json
```

两次验证之间先断电，并完成第 4 节对应侧别的全部检查。

重新确认右侧 USB by-id 名称及其对应的实体设备。当前右侧配置使用遥操设备 B；核对
`USB_BY_ID` 与右侧 YAML 的 `dynamixel_port` 和实体标签一致，再设置并回读 latency timer：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTAO51EA-if00-port0
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
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --confirm-single-servo --side right --id 3 \
  --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2/right_id3_result.json
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
