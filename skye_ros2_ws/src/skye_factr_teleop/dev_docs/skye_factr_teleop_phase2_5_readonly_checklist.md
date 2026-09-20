# 阶段 2.5.1：Dynamixel 通信只读验证清单

本清单只用于阶段 2.5.1。不得启动 `skye_factr_teleop_driver`、控制核心、ROS 遥操节点，
不得调用普通 `DynamixelDriver.open()`，也不得进入阶段 2.5.2 的电流写入。

本文中的“只读”特指不向 Dynamixel 执行器寄存器写入。设置 FTDI latency timer 会写入
主机的 sysfs，但不会写入 Dynamixel 寄存器。

## 0. 推荐主流程

本节是默认执行入口。第 1--6 节保留为安全要求、首次建立方向标记、连续角度检查、断连
检查和失败复测的详细参考；引导检查通过时，不需要再逐条执行 4.3--4.5 的手工正反方向
流程。

### 0.1 首次工程验收

首次验收按以下顺序执行：

1. 完成第 1 节安全确认，并按第 2 节确认 USB by-id、设置当前侧 latency timer；
2. 若实体上还没有经过独立确认的“URDF +”方向标记，只执行一次 4.2，在左右 leader 上
   保留固定方向标记；以后 URDF 和机械装配不变时不重复；
3. 构建并加载工作区：

   ```bash
   cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --packages-select skye_factr_description skye_factr_teleop
   source install/setup.bash
   RECORD_ROOT=/home/lyy/projects/leader_for_skye/skye_ros2_ws/validation_records/phase2_5_1
   mkdir -p "$RECORD_ROOT"
   ```

4. 运行左侧引导检查：

   ```bash
   ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
     --guided --side left \
     --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
     --baseline-frames 50 --movement-frames 200 \
     --settle-frames 50 --endpoint-frames 100 \
     --interval 0.02 --max-abs-velocity 20.0 --max-position-step 0.2 \
     --max-hold-velocity 0.05 --max-hold-position-span 0.01 \
     --output "$RECORD_ROOT/left_guided.json"
   ```

5. 工具会从 ID 1 到 ID 8 依次提示。每个 ID 只做一次动作：
   - 看到 `HOLD STILL (baseline)` 时保持全部关节静止约 1 秒；
   - 看到 `MOVE NOW` 时，只沿实体“URDF +”方向缓慢移动当前关节；ID 8 执行打开动作；
   - 看到 `STOP NOW` 时停止移动；随后约 1 秒是反应缓冲，缓冲期仍持续采样；
   - 看到 `HOLD STILL (endpoint)` 时保持终点约 2 秒，工具会自动检查终点是否稳定；
   - 当前 ID 显示 `PASS` 后再准备下一个 ID。

   每个 ID 的自动采样时间约为 `1 + 4 + 1 + 2 = 8` 秒。ID 之间等待 Enter 的准备阶段
   不采样，操作者可以从容调整起点并确认其它关节稳定。
6. 将第 2 节的 latency timer 设置切换到右侧，然后用右侧配置运行同一命令：

   ```bash
   ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
     --guided --side right \
     --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
     --baseline-frames 50 --movement-frames 200 \
     --settle-frames 50 --endpoint-frames 100 \
     --interval 0.02 --max-abs-velocity 20.0 --max-position-step 0.2 \
     --max-hold-velocity 0.05 --max-hold-position-span 0.01 \
     --output "$RECORD_ROOT/right_guided.json"
   ```

7. 左右两侧都必须显示 `bus result: PASS`、`guided result: PASS`，并且 ID 1--8 每行均为
   `PASS`；任一 ID 失败时，只按 4.3--4.5 的详细方法复测该 ID，不默认补做全部反方向；
8. 对能够安全跨越编码器圈界的关节，按 4.6 完成一次连续角度检查；不能安全跨越的关节
   记录机械或线缆限制；
9. 每条独立总线按第 5 节完成一次断连诊断。连续角度和断连检查属于首次工程验收，日常
   上电不重复；
10. 按第 6 节保存验收结果和记录路径。

引导检查在一次串口连接中完成该侧全部 ID。每个 ID 自动采集“起点静止—单向移动—停止
缓冲—终点静止”四个窗口，并检查主要移动 ID、映射方向、移动幅度、其它 ID 误动、速度、
位置跳变和终点稳定性。终点窗口内任一设备最大绝对速度超过 `0.05 rad/s`，或位置跨度
超过 `0.01 rad`，本轮均判为 FAIL。

### 0.2 日常或重新上电检查

URDF、配置、机械装配和接线未改变时，不重复关节方向、连续角度和断连测试：

1. 完成第 1 节安全确认；
2. 若 USB 发生过拔插，按第 2 节重新设置当前侧 latency timer；
3. 左右侧各执行一次第 3 节自动只读检查；
4. 两侧均为 `automated result: PASS` 即完成日常检查。

### 0.3 维修、重新接线或配置变更后

- 只维修或重新接线一侧：对该侧重新执行 0.1 的引导检查；
- 修改 `joint_signs`、URDF、关节装配或 ID：重新确认方向标记，并对受影响侧执行完整
  引导检查；
- 引导检查失败但原因不明确：按 4.3--4.5 对失败 ID 单独执行正向和反向复测；
- 更换 USB 转串口、总线拓扑或通信相关代码：同时重做第 5 节断连诊断。

---

以下为详细参考和失败复测流程。

## 1. 上电和连接前

- [x] 大臂运动链路已断开或保持停机，现场无其它程序可控制大臂；
- [x] 左右 leader 均处于可自由移动、不会因重力碰撞人员或设备的位置；
- [x] 已停止所有可能占用 leader 串口的 FACTR/Dynamixel 进程；
- [x] 操作者知道本轮验证的侧别，左右总线每次只验证一侧；
- [x] 准备好紧急断电方式；
- [x] 不运行阶段 2.5.2 或任何带 torque/current 写入的命令。

只读确认占用情况：

```bash
pgrep -af 'factr_teleop|skye_factr_teleop|dynamixel' || true
```

## 2. 连接后、运行工具前

确认稳定设备名及左右映射，不使用临时 `/dev/ttyUSB*` 名称作为配置：

```bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%p -> %l\n' | sort
```

已知配置映射：

| 侧别 | USB by-id |
|---|---|
| 左 | `usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0` |
| 右 | `usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0` |

拔插会使 FTDI latency timer 复位。本轮只设置待测一侧：先用对应的 USB by-id 解析实际
tty，再将 latency timer 设置为 1 并回读确认。以下以左侧为例：

```bash
USB_BY_ID=/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO9L1-if00-port0
TTY_PATH=$(readlink -f "$USB_BY_ID")
TTY_NAME=$(basename "$TTY_PATH")
echo "USB by-id: $USB_BY_ID -> $TTY_PATH"
echo 1 | sudo tee "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
cat "/sys/bus/usb-serial/devices/${TTY_NAME}/latency_timer"
```

测试右侧时，只将 `USB_BY_ID` 改为
`/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTBEO8WL-if00-port0`。确认解析出的
`TTY_PATH` 是字符设备，且回读结果为 `1`。

## 3. 左右总线自动只读检查

先构建并加载工作区：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_factr_description skye_factr_teleop
source install/setup.bash
RECORD_ROOT=/home/lyy/projects/leader_for_skye/skye_ros2_ws/validation_records/phase2_5_1
mkdir -p "$RECORD_ROOT"
```

左侧：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --frames 500 --interval 0.02 \
  --max-abs-velocity 20.0 --max-position-step 0.2 \
  --output "$RECORD_ROOT/left_readonly.json"
```

右侧：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side right \
  --config src/skye_factr_teleop/config/grav_comp_m6_right.yaml \
  --frames 500 --interval 0.02 \
  --max-abs-velocity 20.0 --max-position-step 0.2 \
  --output "$RECORD_ROOT/right_readonly.json"
```

本清单固定使用最大绝对速度 `20.0 rad/s`、最大相邻帧位置变化 `0.2 rad` 作为自动检查
阈值，不依赖工具默认值。

每侧的自动检查必须显示 `automated result: PASS`，并确认记录中：

- [ ] 使用了对应侧的 USB by-id；
- [ ] 波特率为 `4000000`，latency timer 为 `1`；
- [ ] ID 1--2 的型号编号为 `1020`（XM430-W350）；
- [ ] ID 3--8 的型号编号为 `1220`（XC330-T288）；
- [ ] 每个设备的 Return Delay Time 为 `0`；
- [ ] 每个设备的 `torque_enabled` 为 `false`；
- [ ] 多帧位置和速度均为有限值，无超限速度或异常位置跳变。

任一项失败即停止，不进入后续步骤。

## 4. 关节顺序、方向和连续角度

本节每次只验证一侧、一个关节和一个运动方向。不要在一次采样中依次移动多个关节，
也不要在采样结束前将关节移回起点；工具用“最后一帧减第一帧”计算方向，往返运动会
让结果接近零。

### 4.1 ID、关节名称和四种方向定义

先区分以下概念，不要把“舵机数值增加”直接当成“URDF 正方向”：

| 名称 | 适用对象 | 定义 | 判定方法 |
|---|---|---|---|
| Dynamixel 原始正方向 | ID 1--8 | 舵机 Present Position 原始计数增加的方向 | JSON 中 `position_delta_rad > 0`；与 URDF 无关 |
| 配置映射正方向 | ID 1--8 | 原始增量乘以 YAML `joint_signs` 后为正 | JSON 中 `signed_position_delta_rad > 0`；终端表格对应 `signed_delta_rad > 0` |
| URDF 正方向 | ID 1--7 | URDF 中该关节坐标增加的方向 | 在模型中令该关节从 `q=0` 增加到 `q=+0.2 rad`，观察 child link 的运动；或在正确的关节局部坐标系中按 `<axis>` 使用右手定则 |
| 夹爪约定正方向 | ID 8 | 本项目人为规定的夹爪语义方向 | 本清单按现有 FACTR 语义固定为“打开为正、闭合为负”，并写入验收记录 |

ID 1--7 的验收目标是证明“配置映射正方向”与“URDF 正方向”一致。计算关系为：

```text
signed_position_delta_rad = position_delta_rad × joint_sign
```

因此，操作者必须先用 URDF 模型或已验证的机械方向标记，独立确定要往哪个物理方向
移动，之后才能用采样结果验证 `joint_signs`。不能先看 `joint_signs` 决定怎么移动，再
用同一个 `joint_signs` 宣称方向正确，那会形成循环论证。

URDF 方向的推荐判定步骤：

1. 在下表找到目标 ID 对应的 URDF joint；
2. 在 URDF 可视化模型中只把该 joint 从当前位置增加一个小正值，例如 `+0.2 rad`；
3. 记录 child link 的实际运动趋势，并在实体关节旁贴上“URDF +”箭头；
4. 若只能阅读 URDF：右手拇指指向关节局部坐标中的 `<axis>` 正向，四指弯曲方向为
   URDF 正转；`<axis>` 属于关节局部坐标，不能直接把 `0 0 1` 理解成现场空间的“向上”；
5. 若没有可视化、机械标记或既有已验证记录，当前只能验证 ID 顺序和原始编码器方向，
   方向项必须保持未完成。

`joint_signs` 用于把 Dynamixel 原始方向换算为配置映射方向。当前配置如下：

| ID | 左侧对象 | 左 `joint_sign` | 右侧对象 | 右 `joint_sign` |
|---:|---|---:|---|---:|
| 1 | `joint1_left` | +1 | `joint1_right` | +1 |
| 2 | `joint2_left` | +1 | `joint2_right` | -1 |
| 3 | `joint3_left` | +1 | `joint3_right` | +1 |
| 4 | `joint4_left` | +1 | `joint4_right` | -1 |
| 5 | `joint5_left` | +1 | `joint5_right` | +1 |
| 6 | `joint6_left` | -1 | `joint6_right` | -1 |
| 7 | `joint7_left` | -1 | `joint7_right` | -1 |
| 8 | leader 夹爪（打开为正） | +1 | leader 夹爪（打开为正） | +1 |

左右 URDF 文件分别是：

- `skye_factr_description/urdf/m6left_isomorphism.urdf`；
- `skye_factr_description/urdf/m6right_isomorphism.urdf`。

具体期望关系：

| 测试动作 | `joint_sign=+1` 时原始增量 | `joint_sign=-1` 时原始增量 | 映射后增量 |
|---|---:|---:|---:|
| 沿 URDF 正方向移动 | `position_delta_rad > 0` | `position_delta_rad < 0` | `signed_position_delta_rad > 0` |
| 沿 URDF 负方向移动 | `position_delta_rad < 0` | `position_delta_rad > 0` | `signed_position_delta_rad < 0` |

例如左侧 ID 6 的 `joint_sign=-1`。若实体沿 `joint6_left` 的 URDF 正方向移动，原始
`position_delta_rad` 应为负，乘以 `-1` 后的 `signed_position_delta_rad` 应为正。
这才说明 ID 6 的方向映射正确。

### 4.2 在 RViz 中查看 URDF 并标记正方向

这一步是纯模型可视化，不连接串口，也不启动 FACTR、Dynamixel 驱动或遥操节点。以下
三个终端必须使用相同的 `ROS_DOMAIN_ID`。这里使用隔离的 `92`，避免模型话题混入当前
机器人 ROS 域。

本节截图统一保存到：

```text
/home/lyy/projects/leader_for_skye/skye_ros2_ws/src/skye_factr_teleop/dev_docs/skye_factr_teleop_phase2_5_readonly
```

#### 终端 1：启动 URDF 发布器

先构建描述包并选择侧别：

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select skye_factr_description
source install/setup.bash
export ROS_DOMAIN_ID=92
export ROS_LOCALHOST_ONLY=1

SIDE=left
URDF_PATH=$(ros2 pkg prefix --share skye_factr_description)/urdf/m6left_isomorphism.urdf
```

查看右侧时，将最后两行改为：

```bash
SIDE=right
URDF_PATH=$(ros2 pkg prefix --share skye_factr_description)/urdf/m6right_isomorphism.urdf
```

确认路径存在，然后启动 `robot_state_publisher`：

```bash
test -f "$URDF_PATH" && echo "URDF: $URDF_PATH"
ros2 run robot_state_publisher robot_state_publisher "$URDF_PATH"
```

终端应出现 `got segment ...`。ROS 2 Humble 可能提示命令行 URDF 文件参数是兼容用法，
该警告不影响本次只读可视化。保持此终端运行。

#### 终端 2：启动并配置 RViz

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92
export ROS_LOCALHOST_ONLY=1
rviz2
```

RViz 打开后按以下顺序设置：

1. 在左侧 `Displays → Global Options` 中把 `Fixed Frame` 改为 `torso_link`；
2. 点击左下角 `Add`，选择 `By display type → rviz_default_plugins → RobotModel`；
3. 展开 `RobotModel`，将 `Description Source` 设为 `Topic`；
4. 将 `Description Topic` 设为 `/robot_description`；
5. 再添加 `TF` 显示项，打开 `Show Names` 和 `Show Axes`，用于区分 link 坐标轴；
6. 点击工具栏 `Reset` 或按 `F` 聚焦模型，拖动鼠标调整观察角度。

如果当前 RViz 版本的 `RobotModel` 没有 `Description Source` 选项，直接把其
`Robot Description`/`Description Topic` 字段设为 `/robot_description` 即可。

#### 终端 3：启动 Joint State Publisher GUI

```bash
cd /home/lyy/projects/leader_for_skye/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=92
export ROS_LOCALHOST_ONLY=1

URDF_PATH=$(ros2 pkg prefix --share skye_factr_description)/urdf/m6left_isomorphism.urdf
ros2 run joint_state_publisher_gui joint_state_publisher_gui "$URDF_PATH"
```

查看右侧时，把最后两行替换为：

```bash
URDF_PATH=$(ros2 pkg prefix --share skye_factr_description)/urdf/m6right_isomorphism.urdf
ros2 run joint_state_publisher_gui joint_state_publisher_gui "$URDF_PATH"
```

GUI 应显示 `joint1_left`--`joint7_left` 或
`joint1_right`--`joint7_right` 七个滑块，以及 `Center`、`Randomize` 按钮。滑块上方的
数值单位是 rad，显示到三位小数。本测试不要点击 `Randomize`。

#### 使用 GUI 判定每个关节的 URDF 正方向

对 ID 1--7 逐个执行：

1. 点击 `Center`，确认七个关节的显示值均为 `0.000`；
2. 在 RViz 中截图保存零位姿态；
3. 只拖动目标 joint 的滑块，直到显示约 `+0.200`，其它六个关节保持 `0.000`；
4. 观察 RViz 中从零位到 `+0.200 rad` 时 child link 的移动趋势；该趋势就是目标关节的
   URDF 正方向；
5. 截图，并记录“侧别、ID、joint 名称、从哪个视角观察、child link 向哪里移动”；
6. 点击 `Center` 回到零位，再把同一滑块拖到约 `-0.200`；
7. 确认 child link 向相反方向移动；若没有反向，先排查是否误动了其它滑块；
8. 再次点击 `Center`，然后测试下一个 joint；
9. 对照实体 leader 的相同 child link，在实体关节旁贴上“URDF +”方向标记。

GUI 的滑块值是 URDF 关节坐标，不是 Dynamixel 编码器读数，也没有应用 YAML
`joint_signs`。因此 GUI 中从 `0.000` 到 `+0.200` 的运动可作为独立方向参照，随后才能
用真实舵机只读结果验证 `joint_signs`。

左右侧必须分别启动对应 URDF 并保存记录，不能用左侧模型的观察结果代替右侧。GUI
发布的 `/joint_states` 只发送给隔离 ROS 域中的模型发布器，不会写入舵机。

完成后：

1. 在终端 1 按 `Ctrl-C` 停止 `robot_state_publisher`；
2. 关闭 RViz；
3. 关闭 Joint State Publisher GUI；
4. 关闭这三个测试终端，不要直接在仍设置为 `ROS_DOMAIN_ID=92` 的终端中启动日常
   机器人节点。

#### RViz 无模型或模型不动时

依次检查：

```bash
ros2 topic echo --once /robot_description
ros2 topic list | rg '^/(robot_description|joint_states|tf|tf_static)$'
```

- 三个终端必须都已 `source` 同一工作区，并使用相同的 `ROS_DOMAIN_ID=92`；
- `Fixed Frame` 必须是 `torso_link`；
- 必须先启动 `robot_state_publisher`，再启动 RViz；
- Joint State Publisher GUI 中必须显示七个对应侧的 joint 滑块；若没有滑块，检查传入
  GUI 的 `URDF_PATH`；
- GUI 运行时 `/joint_states` 必须持续存在，可用
  `ros2 topic echo --once /joint_states` 检查；
- 若移动滑块但模型不动，确认 GUI、RViz 和 `robot_state_publisher` 使用相同的
  `ROS_DOMAIN_ID=92`；
- 若 RobotModel 显示错误，展开其 `Status` 查看缺失的 mesh、TF 或描述话题；
- 若模型过小或不在视野内，选中 `RobotModel` 后按 `F` 聚焦。

#### 左侧各关节的定义如下

`URDF axis` 是 `<joint><axis>` 在关节局部坐标系中的定义；“父 link 坐标系下的轴”还
考虑了该 joint 的 `<origin rpy>` 旋转。下表中的正方向始终按对应旋转轴的右手定则
判定。

| ID | URDF axis | 父 link 坐标系下的轴 | `+0.2 rad` 的正确方向 |
|---:|---|---|---|
| 1 | `+Z` | `lbase_link` 的 `+Z` | 绕局部 `+Z` 按右手定则旋转 |
| 2 | `+Z` | `Link1_left` 的 `-Y` | 绕局部 `+Z` 按右手定则旋转 |
| 3 | `+Z` | `Link2_left` 的 `+Y` | 绕局部 `+Z` 按右手定则旋转 |
| 4 | `+Z` | `Link3_left` 的 `+Y` | 绕局部 `+Z` 按右手定则旋转 |
| 5 | `+Z` | `Link4_left` 的 `-Y` | 绕局部 `+Z` 按右手定则旋转 |
| 6 | `-Z` | `Link5_left` 的 `+Y` | 绕局部 `-Z` 按右手定则旋转 |
| 7 | `-Z` | `Link6_left` 的 `+Y` | 绕局部 `-Z` 按右手定则旋转 |

左侧零位参考截图：

- [后视图](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view1_back.png)
- [顶视图](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view2_top.png)
- [左后视图](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback.png)

左侧各关节从 `0.000` 到约 `+0.200 rad` 的左后视图：

- [joint1_left](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback_j1_dot2rad.png)
- [joint2_left](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback_j2_dot2rad.png)
- [joint3_left](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback_j3_dot2rad.png)
- [joint4_left](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback_j4_dot2rad.png)
- [joint5_left](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback_j5_dot2rad.png)
- [joint6_left](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback_j6_dot2rad.png)
- [joint7_left](skye_factr_teleop_phase2_5_readonly/leftarm_zerostate_view3_leftback_j7_dot2rad.png)

#### 右侧各关节的定义如下

| ID | URDF axis | 父 link 坐标系下的轴 | `+0.2 rad` 的正确方向 |
|---:|---|---|---|
| 1 | `+Z` | `rbase_link` 的 `+Z` | 绕局部 `+Z` 按右手定则旋转 |
| 2 | `-Z` | `Link1_right` 的 `+Y` | 绕局部 `-Z` 按右手定则旋转 |
| 3 | `+Z` | `Link2_right` 的 `+Y` | 绕局部 `+Z` 按右手定则旋转 |
| 4 | `-Z` | `Link3_right` 的 `+Y` | 绕局部 `-Z` 按右手定则旋转 |
| 5 | `+Z` | `Link4_right` 的 `-Y` | 绕局部 `+Z` 按右手定则旋转 |
| 6 | `+Z` | `Link5_right` 的 `-Y` | 绕局部 `+Z` 按右手定则旋转 |
| 7 | `+Z` | `Link6_right` 的 `-Y` | 绕局部 `+Z` 按右手定则旋转 |

右侧零位参考截图：

- [后视图](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view1_back.png)
- [顶视图](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view2_top.png)
- [右后视图](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback.png)

右侧各关节从 `0.000` 到约 `+0.200 rad` 的右后视图：

- [joint1_right](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback_j1_dot2rad.png)
- [joint2_right](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback_j2_dot2rad.png)
- [joint3_right](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback_j3_dot2rad.png)
- [joint4_right](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback_j4_dot2rad.png)
- [joint5_right](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback_j5_dot2rad.png)
- [joint6_right](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback_j6_dot2rad.png)
- [joint7_right](skye_factr_teleop_phase2_5_readonly/rightarm_zerostate_view3_rightback_j7_dot2rad.png)

RViz 坐标轴颜色为红色 `+X`、绿色 `+Y`、蓝色 `+Z`。对于 `URDF axis=+Z` 的关节，
拇指指向蓝色 `+Z` 箭头；对于 `URDF axis=-Z` 的关节，拇指指向蓝色箭头的反方向。
从旋转轴正端朝关节中心观察时，正向旋转表现为逆时针。

后续实体测试必须以 RViz 中目标关节从 `0.000` 移到约 `+0.200 rad` 时 child link 及其
下游结构的实际运动趋势作为“URDF +”方向。不要根据屏幕上的固定顺/逆时针或 YAML
`joint_signs` 决定实体移动方向；`joint_signs` 是后续需要用只读采样独立验证的对象。

### 4.3 执行顺序和每轮采样前的准备

本节按以下顺序执行，避免把“每侧一次”和“每条记录一次”的操作混在一起：

1. 每侧开始测试时，只执行一次第 3 节自动检查，并按 4.4 设置该侧的 `SIDE`、`CONFIG`
   和 `RECORD_ROOT`；
2. 每生成一条正向、反向或连续角度记录前，都重新选择 `TEST_ID` 和 `DIRECTION`，完成
   下列准备，并生成新的 `RUN_TAG`、`RECORD` 和 `LOG`；
3. 正反方向测试按 4.4 的时间线执行一次采样，命令结束后立即检查退出码和记录文件；
4. 按 4.5 判读该轮正反方向结果。本轮有效且通过后，才能准备下一轮；
5. 某侧 ID 1--8 的正反方向记录全部完成后，再按 4.6 执行该侧连续角度检查。

以下准备必须在每条记录开始前重复：

1. 确认第 3 节对应侧自动检查已经 PASS，尤其是所有 ID 的
   `torque_enabled=false`。
2. 一名操作者看终端并计时，另一名操作者托住 leader；单人操作时先放稳 leader，
   确保可以腾出一只手执行目标关节动作。
3. 将目标关节放在机械行程中部，避开硬限位至少约 10°；其余关节保持不动。
4. 在关节本体贴临时标签，写明本轮预期 ID、起始位置以及独立确认过的“URDF +”
   方向；ID 8 标记“打开 +”。
5. 臂关节单次移动建议为 5°--10°（约 0.09--0.17 rad）；ID 8 夹爪移动全行程的
   10%--20%。动作必须缓慢、单向，遇到阻力立即停止。

### 4.4 正反方向单轮采样

#### 4.4.1 每侧设置一次公共参数

在 `skye_ros2_ws` 目录执行。左侧使用：

```bash
SIDE=left
CONFIG=src/skye_factr_teleop/config/grav_comp_m6_left.yaml
RECORD_ROOT=/home/lyy/projects/leader_for_skye/skye_ros2_ws/validation_records/phase2_5_1
mkdir -p "$RECORD_ROOT"
```

右侧改为：

```bash
SIDE=right
CONFIG=src/skye_factr_teleop/config/grav_comp_m6_right.yaml
```

#### 4.4.2 每条记录重新设置参数

为当前待测关节设置 ID 和动作标签。例如测试 ID 1 的 URDF 正方向：

```bash
TEST_ID=1
DIRECTION=urdf_positive
RUN_TAG=$(date +%Y%m%d_%H%M%S_%N)
RECORD=${RECORD_ROOT}/${SIDE}_id${TEST_ID}_${DIRECTION}_${RUN_TAG}.json
LOG=${RECORD_ROOT}/${SIDE}_id${TEST_ID}_${DIRECTION}_${RUN_TAG}.log
```

`RUN_TAG` 精确到纳秒，因此每次执行都会生成新的文件名。`RECORD` 保存完整 JSON 数据，
`LOG` 保存终端表格和错误信息。

注意：`TEST_ID` 和 `DIRECTION` 只用于生成记录文件名，不会控制工具或硬件。工具仍会
读取全部 ID；操作者必须亲手只移动 `TEST_ID` 对应的实体关节，并按 `DIRECTION` 标签
所写的 URDF 方向操作。

#### 4.4.3 启动命令并按时间线动作

以下命令采集 500 帧，帧间隔为 0.02 秒，因此实际采样窗口约为 10 秒。串口打开、设备
预检和关闭发生在采样窗口之外，所以整条命令的运行时间可能超过 10 秒。

```bash
set -o pipefail
echo "开始采样：${SIDE} ID=${TEST_ID} ${DIRECTION}"
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side "$SIDE" --config "$CONFIG" \
  --frames 500 --interval 0.02 \
  --max-position-step 0.2 \
  --output "$RECORD" 2>&1 | tee "$LOG"
CHECK_STATUS=${PIPESTATUS[0]}
echo "ros2 退出码：$CHECK_STATUS"
ls -lh "$RECORD" "$LOG"
```

具体动作时间线如下。命令启动时间只是现场计时参考，最终应以 JSON 逐帧数据是否包含
“起点静止—单向移动—终点静止”三个阶段为准：

1. 按下回车前，所有关节已经保持静止；
2. 命令启动后的前 2 秒继续保持所有关节静止；
3. 第 2--6 秒只缓慢移动目标关节，到达建议幅度后停止；
4. 第 6--10 秒保持新的终点位置，不要回到起点，并继续保持到命令自行结束；
5. 若设备预检耗时导致 JSON 开头没有稳定的起点样本，本轮无效；使用新的 `RUN_TAG`
   重新测试，必要时增加 `--frames`，但仍保持上述三个动作阶段；
6. 等命令自行结束并显示 `automated result: PASS`；
7. 确认 `CHECK_STATUS=0`，并确认 `.json`、`.log` 两个文件均已生成。

`tee "$LOG"` 会让表格继续显示在终端，同时原样写入 `.log` 文件；`2>&1` 会把错误信息
也保存进去。由于命令使用了管道，必须启用 `set -o pipefail`，并立即用
`CHECK_STATUS=${PIPESTATUS[0]}` 保存 `ros2 run` 的真实退出码：`0` 表示命令成功，非
`0` 表示本轮失败。不要用 `tee` 自己的退出码判断硬件检查结果。

`ls -lh` 必须同时列出 `.json` 和 `.log`。如果其中任一文件不存在或大小为 0，本轮
记录不完整，需要使用新的 `RUN_TAG` 重新测试。不要手工把下一轮的 `RECORD` 或 `LOG`
改回已有文件名，因为工具和 `tee` 都可能覆盖同名文件。

### 4.5 正反方向测试顺序和结果判读

对 ID 1--8 逐个执行以下两轮，合计每侧 16 个记录：

1. 从中间位置沿实体上的“URDF +”标记移动，`DIRECTION=urdf_positive`；
2. 重新放到中间位置，沿“URDF +”的反方向移动，`DIRECTION=urdf_negative`。

ID 8 不使用 URDF 标签：打开夹爪时用 `DIRECTION=gripper_open`，关闭夹爪时用
`DIRECTION=gripper_close`。打开记录应为正，关闭记录应为负。

不要直接用反向动作回到正向测试的起点，因为起始位置、移动幅度和终点保持时间不易
一致。每轮都重新执行 4.3 的准备，重新启动命令并生成独立记录。每轮结束后先完成以下
判读；若本轮无效或失败，使用新的 `RUN_TAG` 重测同一方向，不进入下一方向或下一 ID。

终端表格每行含义（括号内为对应的 JSON 字段）：

- `position_delta_rad`：仅在 JSON 中提供，表示 Dynamixel 原始位置增量；
- `signed_delta_rad`（`signed_position_delta_rad`）：`position_delta_rad × joint_sign`，
  表示配置映射后的增量；
- `max_velocity_rad_s`（`max_abs_velocity_rad_s`）：采样期间的最大绝对速度；
- `max_step_rad`（`max_position_step_rad`）：相邻两帧间的最大位置变化，用于发现跳变。

也可查看 JSON 中的完整分析和逐帧数据：

```bash
/usr/bin/python3 -m json.tool "$RECORD" | less
```

`automated result: PASS` 是本轮继续人工判读的必要条件，但不能代替下列方向和位移检查。
本轮通过条件如下，字段均使用 JSON 名称：

- JSON 逐帧数据包含“起点静止—单向移动—终点静止”三个阶段；
- 目标 ID 的 `abs(signed_position_delta_rad)` 应至少约为 `0.05` rad；
- 目标 ID 应是本轮绝对位移最大的设备；
- 其余 ID 应基本不动，建议 `abs(signed_position_delta_rad) < 0.03` rad；
- URDF 正方向测试应得到 `signed_position_delta_rad > 0.05` rad；
- URDF 负方向测试应得到 `signed_position_delta_rad < -0.05` rad；
- `max_position_step_rad` 必须小于或等于本命令设置的 `0.2` rad；
- 若其它 ID 也明显移动，重新固定其它关节后重测，不据此修改配置。

### 4.6 连续角度检查

普通引导检查完成后，对 ID 1--7 运行一条自动跨圈引导命令。能够安全跨圈的关节执行
实际测试；不能安全跨圈的关节在提示处跳过并记录原因：

```bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --guided-continuous --ids 1,2,3,4,5,6,7 --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --baseline-frames 50 --continuous-max-frames 750 \
  --settle-frames 50 --endpoint-frames 100 --interval 0.02 \
  --max-abs-velocity 20.0 --max-position-step 0.2 \
  --max-other-delta 0.03 --motion-reversal-tolerance 0.02 \
  --max-hold-velocity 0.05 --max-hold-position-span 0.01 \
  --output "$RECORD_ROOT/left_continuous.json"
```

右侧改用 `--side right`、右侧 YAML 和 `right_continuous.json`。`--ids` 不得包含夹爪
ID 8；完整首次验收应列出 ID 1--7，使每个关节最终都有 `PASS` 或
`SKIPPED_UNSAFE` 记录。

工具对每个 ID 自动执行：

1. 提示操作者把目标关节放在安全起点并保持静止，按 Enter 后采集约 1 秒基线；
2. 显示当前位置、最近的 `2π` 圈界、所需移动距离以及实体 `URDF +`/`URDF -` 方向；
3. 操作者确认该距离和方向安全后按 Enter；若不能安全跨越，输入
   `s:机械或线缆限制原因`；
4. 看到 `MOVE NOW` 后只缓慢移动目标关节。工具最多监测 750 帧（约 15 秒），检测到
   真正跨界、位置不连续或超时后自动提示停止；
5. 看到 `STOP NOW` 后停止，经过约 1 秒反应缓冲，再按提示保持终点约 2 秒；
6. 工具自动检查跨界、方向、单向性、其它 ID 误动、速度、相邻帧跳变和终点稳定性。

单个 ID 的结果含义：

- `PASS`：确实跨过指定圈界，位置连续，且其它检查全部通过；
- `FAIL`：出现约 `2π` 跳变、方向错误、明显反向、其它 ID 误动或稳定性失败；
- `INCONCLUSIVE`：在最大采样时间内没有真正跨界，不能作为通过证据；
- `SKIPPED_UNSAFE`：操作者确认不能安全跨界，并在 JSON 中保存了原因。

整侧 `continuous result: PASS` 或 `PASS_WITH_SKIPS` 才可接受；后者要求每个跳过项都有明确
的机械或线缆限制原因。`FAIL` 和 `INCONCLUSIVE` 均需要排查或重测。

不得为了跨圈而越过机械限位、扭转线缆或强行旋转关节。若某个关节在安全行程内无法
跨越圈界，必须使用 `s:原因` 记录为 `SKIPPED_UNSAFE`，不得把普通小幅运动填写成
“已完成跨圈验证”。

### 4.7 本节完成条件

只有正反方向记录及其人工判读全部完成，并完成连续角度检查或正确记录无法安全跨圈的
原因后，才逐项确认：

- [ ] 左右两侧均有完整的 guided JSON 记录；
- [ ] 左侧 ID 1--8 与物理关节/夹爪顺序一致；
- [ ] 右侧 ID 1--8 与物理关节/夹爪顺序一致；
- [ ] ID 1--7 的 URDF 正向记录中 `signed_position_delta_rad` 为正；
- [ ] ID 8 打开记录为正；
- [ ] 每次只有被操作关节出现主要位移；
- [ ] 失败或存疑的 ID 已按 4.3--4.5 单独完成必要的正反方向复测；
- [ ] 可安全跨圈的关节结果为 `PASS`，无约 `2π` 瞬时跳变；
- [ ] 无法安全跨圈的关节为 `SKIPPED_UNSAFE`，已记录原因且未强行越过限位。

## 5. 单设备断连诊断

只在确认 torque 全部关闭后执行。使用预先准备的单设备安全隔离开关或可断开的测试分支；
禁止在总线带电时直接拔插 DYNAMIXEL 接头。启动较长采样后，隔离一个预先记录的舵机：

```bash
RECORD_ROOT=/home/lyy/projects/leader_for_skye/skye_ros2_ws/validation_records/phase2_5_1
mkdir -p "$RECORD_ROOT"
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side left \
  --config src/skye_factr_teleop/config/grav_comp_m6_left.yaml \
  --frames 1500 --interval 0.02 \
  --max-abs-velocity 20.0 --max-position-step 0.2 \
  --output "$RECORD_ROOT/left_disconnect.json"
```

- [ ] 命令非零退出；
- [ ] 错误包含 `read_state` 或 `ping` 操作名；
- [ ] 错误包含被断开设备的 ID；
- [ ] 进程退出且串口已关闭；
- [ ] 记录文件保留失败前的采样和完整错误。

右侧重复同样验证。恢复隔离设备前先断电。

## 6. 验收记录

```text
日期/操作者：
侧别：left / right
记录根目录：
USB by-id：
配置文件：
舵机型号与 ID：
波特率：4000000
FTDI latency timer：1
Return Delay Time：0 / FAIL
Torque Enable：全部 0 / FAIL
连续采样记录：
关节顺序结果：
关节方向结果：
连续角度结果：
单设备断连 ID：
断连错误是否含 ID 和操作名：
记录索引路径：
结论：PASS / FAIL
备注：
```

每个实际执行的记录占一行；左右侧、ID 1--8 的正反方向记录以及连续角度、断连记录均应
列入索引：

| 侧别 | ID | 测试方向/类型 | `signed_position_delta_rad` | JSON 路径 | 日志路径 | 结果 |
|---|---:|---|---:|---|---|---|
|  |  |  |  |  |  | PASS / FAIL |

验收记录必须保存在持久化目录中，不得将 `/tmp` 中的临时文件直接作为最终验收证据。

只有左右总线全部完成且上述人工项均填写为 PASS，才认为阶段 2.5.1 完成。此结论不
授权进入阶段 2.5.2；低电流验证需单独确认。
