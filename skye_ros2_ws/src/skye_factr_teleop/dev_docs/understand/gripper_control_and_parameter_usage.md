# 夹爪分阶段控制与 YAML 参数消费

整理日期：2026-10-29。

本文依据当前源码，说明新 `skye_factr_teleop` 流程中的夹爪控制：启动 → SYNC → 稳定保持 → 从臂对齐 → TELEOP → STOP。排查时宿主机未发现运行中的 ROS 节点，因此这里记录的是源码行为，不代表已确认现场启动参数或完成真机验证。

下文参数数值以 Thor 左右臂配置为准；旧 Marvin Docker／Operator UI 默认入口与新包入口不同，不能直接套用本文对新主臂控制核心的结论。

## 1. 两个控制对象

| 对象 | 控制职责 | 实现 |
|---|---|---|
| 主臂 J8 扳机 | 同步走位、保持、回弹手感、提供操作者输入 | Dynamixel 电流模式：软件计算力矩，换算为受限电流 |
| 从臂末端夹爪 | 执行扳机给出的开度 | `skye_robot_driver`：Thor 使用 DM4310，Orin 使用 Robotiq Hand-E |

主臂配置：[左臂 YAML](../../config/grav_comp_m6_left.yaml)、[右臂 YAML](../../config/grav_comp_m6_right.yaml)。主臂控制实现：[control_core.py](../../skye_factr_teleop/control_core.py)、[ros_driver.py](../../skye_factr_teleop/ros_driver.py)。

## 2. 各阶段夹爪控制

| 阶段 | 主臂 J8 扳机 | 从臂末端夹爪 |
|---|---|---|
| 启动／`IDLE` | 打开并配置总线，保持去使能，无回弹控制 | 从臂驱动连接并启动夹爪桥接后，将末端目标设置为张开 |
| `TELEOP_SYNCING` | Ruckig 规划 J8 到 `gripper_setpoint`，轨迹 PD 跟踪；启用时叠加限位屏障 | 新主臂节点不发布夹爪命令，末端继续执行此前目标 |
| `HOLD_SETTLING` | 保持同步目标，从轨迹 PD 平滑切换为带积分的保持控制；检查实测位置和速度 | 同上 |
| `SYNCED` | 继续保持同步目标，积分控制继续运行 | 同上 |
| 从臂对齐 | 主臂仍处于 `SYNCED`，J8 控制不变 | 对齐节点只控制机械臂 7 轴，不控制夹爪 |
| `TELEOP` | 清除同步轨迹与保持积分，改为朝 `gripper_setpoint` 回弹的 PD；操作者克服回弹力捏合扳机 | 按 J8 实测逻辑角度计算开度并持续发布 |
| `STOP`／同步超时／从臂反馈失效 | 清除轨迹与积分，写零电流并去使能 | 主臂停止发布；末端驱动继续执行最后目标，普通 `switch_stop` 不会关闭末端夹爪驱动 |

新流程的 SYNC 使用从臂 7 轴位置和本地主臂夹爪设定点组成 8 轴目标，**没有读取末端夹爪开度来同步 J8**。`joint_move` 也使用配置中的夹爪设定点。

## 3. 主臂 J8 消费的 YAML 参数

以下数组参数均取第 8 项，即 Python 下标 `[7]`。这是主臂 J8，不是从臂夹爪电机。

### 3.1 总线、坐标和输出约束

| 参数路径 | Thor 当前值 | 用途 |
|---|---|---|
| `dynamixel.dynamixel_port` | 左右侧各自配置的串口 | 选择总线 |
| `dynamixel.baudrate` | `4000000` | 总线通信速率 |
| `dynamixel.servo_types[7]` | `XC330-T288` | 电流单位、力矩到电流的换算 |
| `dynamixel.joint_offsets[7]` | `0.0 rad` | 角度零点校正 |
| `dynamixel.joint_signs[7]` | `+1` | 逻辑状态与物理输出方向转换 |
| `dynamixel.current_limit_ma[7]` | `50.0 mA` | 最终输出电流限幅 |
| `controller.frequency` | `500 Hz` | 控制循环与 Ruckig 更新频率 |

逻辑位置和速度的转换为：

```text
q  = (q_raw - joint_offsets[7]) × joint_signs[7]
dq = dq_raw × joint_signs[7]
```

各控制项力矩相加后，再乘方向系数、按舵机型号换算为物理电流并限幅。下面公式描述的是限幅前的逻辑力矩，不代表实际输出可无限增大。

### 3.2 SYNC：Ruckig 轨迹与 PD 跟踪

轨迹参数前缀为 `controller.sync_hold_control`。最终目标单独读取
`controller.limit_barrier_control.gripper.gripper_setpoint`。

| 相对参数路径 | Thor 当前值 | 用途 |
|---|---|---|
| `ruckig.kp[7]` | `0.35` | 轨迹位置误差增益 |
| `ruckig.kd[7]` | `0.05` | 轨迹速度误差增益 |
| `ruckig.max_velocity[7]` | `0.8 rad/s` | 参考轨迹速度上限 |
| `ruckig.max_acceleration[7]` | `1.0 rad/s²` | 参考轨迹加速度上限 |
| `ruckig.max_jerk[7]` | `5.0 rad/s³` | 参考轨迹加加速度上限 |
| `ruckig.trajectory_timeout` | `30.0 s` | 同步轨迹阶段超时后停止 |

```text
τ_tracking = ruckig.kp[7] × (q_ref - q)
           + ruckig.kd[7] × (dq_ref - dq)
```

其中 `q_ref`、`dq_ref` 来自 Ruckig 当前周期的轨迹采样；最终目标为 `gripper_setpoint`。

### 3.3 HOLD_SETTLING／SYNCED：固定目标保持

参数前缀为 `controller.sync_hold_control.hold`。

| 相对参数路径 | Thor 当前值 | 用途 |
|---|---|---|
| `kp[7]`、`kd[7]`、`ki[7]` | `0.35`、`0.05`、`0.15` | 固定同步目标的保持控制 |
| `i_max[7]` | `0.10 rad·s` | 位置误差积分绝对值上限 |
| `i_thresh[7]` | `0.15 rad` | 误差超过阈值时，该轴积分清零 |
| `sync_position_tolerance[7]` | `0.05 rad` | J8 实测位置稳定容差 |
| `sync_velocity_tolerance[7]` | `0.05 rad/s` | J8 实测速度稳定容差 |
| `settle_duration` | `0.25 s` | 全部 8 轴连续达标的时间 |
| `settle_timeout` | `25.0 s` | 稳定等待超时后停止 |

```text
error = q_set - q
τ_hold = hold.kp[7] × error
       - hold.kd[7] × dq
       + hold.ki[7] × integral
```

`q_set` 是开始同步时保存的夹爪目标。积分只在误差阈值内累积，限幅后再乘 `ki`。轨迹 PD 到保持控制的过渡时间为代码常量 `HOLD_BLEND_DURATION_S = 0.5 s`，不来自 YAML。

轨迹结束仅表示参考轨迹完成；J8 与其他 7 轴的实测位置、速度全部连续达标后，才会进入 `SYNCED`。

### 3.4 TELEOP 回弹与限位屏障

| 参数路径 | Thor 当前值 | 用途 |
|---|---|---|
| `controller.limit_barrier_control.gripper.gripper_setpoint` | `0.5 rad` | TELEOP 回弹目标 |
| `controller.limit_barrier_control.gripper.kp` | `0.301658022` | 回弹刚度，同时用于越界恢复 |
| `controller.limit_barrier_control.gripper.kd` | `0.0315069084` | 回弹阻尼，同时用于越界阻尼 |
| `controller.limit_barrier_control.gripper.limit_min` | `-0.0175 rad` | J8 下限，同时作为开度映射下端点 |
| `controller.limit_barrier_control.gripper.limit_max` | `0.52517 rad` | J8 上限，同时作为开度映射上端点 |
| `controller.limit_barrier_control.enable` | `true` | 是否叠加限位屏障力矩 |

TELEOP 回弹公式为：

```text
τ_spring = gripper.kp × (gripper_setpoint - q) - gripper.kd × dq
```

这里使用的是 `limit_barrier_control.gripper.kp/kd`，不是同步阶段的 `ruckig.kp/kd` 或 `hold.kp/kd`，也没有保持积分项。

限位屏障只在 J8 超出 `limit_min/max` 后产生恢复力矩，阻尼只抵抗继续向外的运动。启用时，它与回弹／同步跟踪／保持力矩叠加。

两个容易混淆的行为：

- `limit_barrier_control.enable=false` 只关闭限位屏障。TELEOP 回弹仍直接读取该配置块中的夹爪 `kp/kd`，开度映射仍读取 `limit_min/max`。
- `limit_barrier_control.joint.safety_margin` 只作用于机械臂 7 轴，不作用于 J8 夹爪区间。

## 4. gripper_setpoint 不只作用于 TELEOP

同一个 `gripper_setpoint` 用于三个阶段：

1. `TELEOP_SYNCING`：作为 J8 的最终轨迹目标。
2. `HOLD_SETTLING`／`SYNCED`：保持开始同步时保存的该目标。
3. `TELEOP`：作为扳机回弹目标。

因此，修改配置并重新加载后，会同时改变同步位置、同步后的保持位置、遥操时的回弹位置。它不直接指定末端夹爪开度。

该字段现统一位于 `controller.limit_barrier_control.gripper`，放在 `limit_min` 与 `kp` 之间：

```yaml
controller:
  limit_barrier_control:
    # enable 只控制限位屏障，不控制 TELEOP 回弹和开度映射。
    enable: true
    # joint 等其他配置省略。
    gripper:
      limit_max: 0.52517
      limit_min: -0.0175
      gripper_setpoint: 0.5
      kp: 0.301658022
      kd: 0.0315069084
```

迁移旧配置时，从 `sync_hold_control` 删除 `gripper_setpoint`，保留其 `ruckig` 和 `hold`。
这是配置位置调整，SYNC、保持与 TELEOP 回弹的使用方式不变。控制核心的显式
`gripper_setpoint=` 同步调用参数也保持不变，可覆盖本次同步目标。

## 5. 如何理解“回弹目标”和“发送扳机位置”同时存在

主臂扳机可以理解为一个**带回弹弹簧的位置输入器**：回弹控制负责手感，实测位置负责给从臂下指令。

每个 TELEOP 周期同时进行两项计算：

| 功能 | 输入 | 输出 |
|---|---|---|
| 主臂回弹 | `gripper_setpoint - q` 与实测速度 | 主臂 J8 电流，产生回弹力 |
| 从臂控制 | J8 实测逻辑角度 `q` | 归一化开度命令 |

设定点是扳机“想回去的位置”，发送值是操作者实际把它捏到的位置。电流控制提供柔性回弹，允许手改变扳机位置。

以 `gripper_setpoint=0.5 rad` 为例：

1. 松手：回弹力将扳机推向张开目标，从臂夹爪随之张开。
2. 捏扳机：手克服回弹力，将扳机压到更小角度，从臂夹爪随之闭合。
3. 捏住不动：手与回弹力平衡，从臂持续收到当前开度。
4. 再次松手：扳机回弹，从臂重新张开。

实际松手位置还受电流限幅、摩擦等影响；回弹设定点不是实际位置的保证。这套回弹来自本地 PD，不代表末端实际夹持力。

## 6. 从 J8 角度到末端开度

仅在 `TELEOP` 发布夹爪命令，使用 J8 的实测逻辑角度：

```text
open_norm = clip((q - limit_min) / (limit_max - limit_min), 0, 1)
```

发布话题为 `/left_teleop_gripper/ctrl` 或 `/right_teleop_gripper/ctrl`，消息类型为 `sensor_msgs/msg/JointState`，开度放在 `position[0]`：

- `open_norm=0`：闭合端。
- `open_norm=1`：张开端。

新主臂节点每周期读取实测位置进行映射，没有把 `gripper_setpoint` 当作末端命令直接发送。当前 Thor 参数下，实测位置恰好为 `0.5 rad` 时，`open_norm≈0.954`，即约 95.4% 的归一化张开量；这不表示夹爪实际毫米开度已经到位。

从臂驱动默认使用 `gripper_left_invert=true`、`gripper_right_invert=true` 转换：

```text
motor_norm = 1 - open_norm
```

因此电机空间为 `0=张开、1=闭合`，再按 `gripper_close_limit` 限制最大闭合程度。

## 7. 从臂末端 YAML 参数消费

从臂配置先加载 [skye_robot.yaml](../../../skye_robot_driver/config/skye_robot.yaml)，再加载 [Thor profile](../../../skye_robot_driver/config/profiles/thor.yaml) 或 [Orin profile](../../../skye_robot_driver/config/profiles/orin.yaml) 覆盖同名参数；launch 最后还可覆盖启动开关。下列数值描述仓库默认配置，不代表已确认的运行时参数。

### 7.1 通用参数

| 参数 | 用途与当前配置 |
|---|---|
| `enable_gripper` | 是否启动末端夹爪桥接，启动脚本默认传入 `true` |
| `gripper_left_type`、`gripper_right_type` | Thor 为 `dm4310`，Orin 为 `robotiq` |
| `gripper_left_invert`、`gripper_right_invert` | 命令和反馈方向转换，当前均为 `true` |
| `gripper_invert` | 左右侧独立反转参数的默认值；当前已有显式左右配置 |
| `gripper_rate_hz` | 末端控制定时器频率，当前 `100 Hz` |
| `gripper_close_limit` | 电机空间最大闭合比例，Thor 为 `0.93`，Orin 为 `1.0` |

### 7.2 Thor：DM4310

通过末端 CANFD 发送 MIT 位置 PD 命令，目标速度和前馈力矩均为零。

| 参数 | 当前值／用途 |
|---|---|
| `gripper_kp`、`gripper_kd` | `3.0`、`0.12`，末端 DM4310 的位置 PD 参数，与主臂 J8 增益不同 |
| `gripper_pos_min`、`gripper_pos_max` | `0.0`、`1.6 rad`，归一化目标到电机角度的映射 |
| `gripper_left_motor_id`、`gripper_right_motor_id` | `1`、`2`，启动时反馈探测可能选到其他 ID |
| `gripper_right_terminal` | `1`，右侧 ARM1；左侧固定 ARM0 |
| `gripper_feedback_timeout_ms` | `1 ms`，常规发送／反馈读取超时参数 |

```text
q_motor_target = gripper_pos_min
               + motor_norm × (gripper_pos_max - gripper_pos_min)
```

### 7.3 Orin：Robotiq Hand-E

通过末端 RS485 Modbus 发送目标开度、速度和夹持力设置。它不使用 DM4310 的 `gripper_kp/kd`。

| 参数 | 当前值／用途 |
|---|---|
| `gripper_robotiq_speed`、`gripper_robotiq_force` | `255`、`40` |
| `gripper_left_robotiq_pos_min_mm`、`gripper_right_robotiq_pos_min_mm` | 左 `2 mm`、右 `13 mm`，闭合端开度 |
| `gripper_left_robotiq_pos_max_mm`、`gripper_right_robotiq_pos_max_mm` | 均 `50 mm`，张开端开度 |
| `gripper_robotiq_pos_min_mm`、`gripper_robotiq_pos_max_mm` | 左右独立开度参数的默认值；当前已有显式左右配置 |
| `gripper_left_robotiq_terminal`、`gripper_right_robotiq_terminal` | `0`、`1` |
| `gripper_left_robotiq_485_channel`、`gripper_right_robotiq_485_channel` | 均为 `485A` |
| `gripper_left_robotiq_slave_id`、`gripper_right_robotiq_slave_id` | 均为 `9` |
| `gripper_robotiq_modbus_timeout_ms` | `200 ms`，具体操作还受代码中的最低超时约束 |

```text
opening_mm = pos_max_mm - motor_norm × (pos_max_mm - pos_min_mm)
```

实现参考：[末端命令与反馈转换](../../../skye_robot_driver/src/driver_node.cpp)、[DM4310 后端](../../../skye_robot_driver/src/dm4310_gripper_arm.cpp)、[Robotiq 后端](../../../skye_robot_driver/src/robotiq_gripper_arm.cpp)。

## 8. 未参与 J8 控制的配置与反馈语义

- `controller.force_feedback_control.gripper` 下的 `enable`、`gain`、`ema_beta`、`effort_threshold`、`damping`、`ema_fast_alpha` 当前仅解析配置，没有接入控制计算。扳机回弹不依赖末端夹持力。
- `joint_gravity_comp`、`static_friction_comp`、`null_space_regulation` 及同步／保持阶段的 `gravity_comp_gain` 只作用于主臂 J1～J7，不作用于 J8。
- 末端 `/left_gripper/state`、`/right_gripper/state` 的位置转换回 `0=闭合、1=张开` 的语义；若无有效反馈，驱动会以目标值回显，`frame_id=no_feedback`，不能把它当作实测位置。
- `/gento/{left,right}_gripper_action_applied` 发布的是电机空间目标，即 `0=张开、1=闭合`，与上述 state／ctrl 开度方向不同。

## 9. 配置与入口边界

新包通过 [launch_support.py](../../skye_factr_teleop/launch_support.py) 按 `robot_profile` 选择主臂 YAML：Thor 使用 `config/`，Orin 使用 `config/orin/`，也可通过 `left_config`／`right_config` 显式指定。

本文主臂参数表只列 Thor 当前值。Orin 配置的设定点、方向、开关和超时存在差异；当前 Orin YAML 还缺少新流程运行所需的 Ruckig 运动上限，不能将其视为已可直接运行的新流程配置。

仓库 Operator UI 默认 playbook 仍指向旧 Marvin Docker launch；整理时工作区没有 `marvin_ws` 内容。判断现场行为时，应以实际启动入口和加载的 YAML 为准，不以旧文档示例替代当前源码或运行参数。
