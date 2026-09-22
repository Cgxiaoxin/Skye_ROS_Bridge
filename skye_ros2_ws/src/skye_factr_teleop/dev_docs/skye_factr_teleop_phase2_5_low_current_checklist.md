# 阶段 2.5.2：单舵机低电流验证清单

本文用于阶段 2.5.2。连接硬件前先完成第 1--3 节；第 4 节之后才包含真实串口写入。
本工具只调用生产 `DynamixelDriver`，不启动控制核心或 ROS 遥操节点。

## 1. 覆盖范围与硬限制

最小验收矩阵如下，同时覆盖左右总线和两种实际型号：

| 侧别 | ID | 配置型号 | 配置文件 |
| --- | ---: | --- | --- |
| left | 1 | XM430-W350 | `config/grav_comp_m6_left.yaml` |
| right | 3 | XC330-T288 | `config/grav_comp_m6_right.yaml` |

如实体接线标识要求换用同型号的其它 ID，可以替换 ID，但必须重新运行
`--prepare-only` 并在记录中保存实际 ID。

工具实施两层电流限制：

1. `--current-limit-ma` 是操作者在测试前明确记录的本次限制；
2. 工具拒绝超过 `min(100 mA, 型号额定电流限制的 10%)` 的限制。

当前硬上限为 XM430-W350 `100 mA`、XC330-T288 `91 mA`。测试脉冲还必须不超过
`--current-limit-ma`，持续时间不得超过 2 秒。工具不修改 EEPROM 的 Current Limit
寄存器，全部命令仍经过生产驱动的型号换算和额定限幅。

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
`--confirm-single-servo`，并把输出文件名改为 `*_result.json`。以左侧 ID 1 为例：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_low_current_check -- \
  --confirm-single-servo --side left --id 1 \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --current-limit-ma 50 --test-current-ma 20 \
  --pulse-seconds 0.5 --settle-seconds 0.2 \
  --output src/skye_factr_teleop/validation_records/phase2_5_2/left_id1_result.json
```

工具执行顺序：

1. 单 ID 只读预检并核对实际 model number；
2. 关闭只读连接；
3. 生产驱动执行 Torque Disable → Current Control Mode → 零 Goal Current → Torque Enable；
4. 读取初始位置；
5. 短时正电流 → 零电流 → 读取位置；
6. 短时负电流 → 零电流 → 读取位置；
7. 比较两次位置增量是否方向相反；
8. 生产关闭路径再次写零电流、Torque Disable、关闭串口。

执行右侧 ID 3 时使用右侧配置和输出路径。两次验证之间先断电并完成第 4 节对应侧别的
全部检查。

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
