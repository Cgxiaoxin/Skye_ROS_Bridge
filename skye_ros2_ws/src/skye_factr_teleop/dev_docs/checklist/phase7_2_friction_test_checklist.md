# 阶段 7.2：单臂静摩擦补偿低电流验收清单

状态：软件入口与无硬件测试已准备；设备 A 左右两侧 ID 1--7 均按文末的操作者现场结论验收 `PASS`。本项只测试臂关节 ID 1--7，夹爪 ID8 没有静摩擦项。

## 前提与判定

- 确认该侧阶段 7.1 低电流验收仍有效。只连接目标侧，小臂无负载；一人持续支撑小臂，另一人操作终端，清场并确保可以立即断电。配置、装配或接线变化时，先重验受影响侧的只读方向。
- 每个 ID 使用**一次启动、一次最长通电窗口、一次记录和一次结束后 Torque Disable 回读**。先在 `off` 阶段感受零输出基准，再原地切换到 `on` 阶段比较。切换时先写零电流；每次进入 `on` 都重新渐增，但**不会延长整轮最长通电时间**。需要时可在时间窗口内再次切回 `off` 或 `on`。
- 启动参数须满足 `joint_id=1..7`、`0<gain_scale<=1`、`0<current_limit_ma<=50` 且不超过 YAML 任一 ID 上限、`ramp_seconds>=1`、`ramp_seconds<max_duration_seconds<=300`。准备脚本默认 0.1 倍增益、20 mA、2 s 渐增、25 s 总时长；需要最长 300 s 时在准备脚本的“最长通电时间 s”提示中输入 `300`。从低电流开始，长时间测试中仍可随时发送停止请求。
- `off` 阶段总线 8 个 ID 虽已使能，但命令电流均为零，不能提供机械支撑。`on` 阶段仅目标 ID 可收到静摩擦项；重力模型只提供已乘重力增益的幅值参考，不下发重力力矩。速度绝对值达到该关节 `enable_speed` 时静摩擦输出为零。
- 目标 ID 命令电流不得超过本轮上限，其他 ID 必须为零。出现突动、其他关节动作、线缆干涉或通信异常，立即发送 `switch_stop`。低电流下效果无法分辨时记为 `INCONCLUSIVE`，不得判为 `PASS`。
- 记录器保存阶段切换、关节状态、命令与回读电流及现场观察。ROS 话题间不是严格同步采样；速度阈值的精确边界由无硬件测试验证，真机结合 trace 与现场观察复核。配对记录只检查首次 `off→on` **切换前后**七关节的最大角度变化不超过 0.1 rad；`off` 阶段可以推动关节，无须回到该阶段的起始姿态。记录中的 `switch_off_position_rad` 和 `switch_pose_max_difference_rad` 用于核对这一条件。

## 一轮操作：一个侧别、一个关节 ID

先进入 ROS 工作空间。源码变化后构建并加载环境：

~~~bash
cd /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to skye_factr_teleop --symlink-install
source install/setup.bash
~~~

### 1. 准备会话与端口

准备脚本只读 YAML，不打开串口；默认生成 `off → on` 配对会话。端口脚本核对设备 A USB by-id，并写入、回读 FTDI `latency_timer=1`。确认串口没有被其他进程占用：

~~~bash
cd /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
python3 src/skye_factr_teleop/scripts/prepare_friction_trial.py && \
  bash src/skye_factr_teleop/scripts/check_friction_trial_port.sh && \
  source /tmp/skye_factr_friction_trial.env
~~~

准备脚本报错时先修正并重新运行，不要沿用 `/tmp` 中可能残留的旧会话文件。

### 2. 启动记录器与测试节点

终端 A 启动只读记录器，保存逐帧 trace 和本轮结果。状态与输出阶段变化会在此显示，结束时按提示填写现场观察。

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 run skye_factr_teleop skye_factr_friction_test_recorder -- \
  --side "$SIDE" --joint-id "$JOINT_ID" --config "$CONFIG" \
  --operator "$OPERATOR" --gain-scale "$GAIN_SCALE" \
  --current-limit-ma "$CURRENT_LIMIT_MA" --ramp-seconds "$RAMP_SECONDS" \
  --max-duration-seconds "$MAX_DURATION_SECONDS" --enabled false --paired \
  --output "$OUTPUT"
~~~

终端 B 启动目标侧节点。启动后应处于 `IDLE` 且 Torque Disable；确认终端 A 已看到 `IDLE`：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 launch skye_factr_teleop "$LAUNCH_FILE" \
  config_path:="$CONFIG" joint_id:="$JOINT_ID" gain_scale:="$GAIN_SCALE" \
  current_limit_ma:="$CURRENT_LIMIT_MA" ramp_seconds:="$RAMP_SECONDS" \
  max_duration_seconds:="$MAX_DURATION_SECONDS" enabled:=false
~~~

### 3. 独立的监看终端

参照[限位测试的现场观察方式](phase7_2_limit_test_checklist.md#步骤-5进入测试并现场观察新开终端-c)，在进入测试模式之前打开独立的监看终端。每个代码块都包含该终端需要的环境加载命令，可整块复制执行。`ros2 topic echo` 显示原始消息；最后新增的绝对值窗口只刷新同一行。这些命令均不发送控制请求。

**(optional) 终端 D1：操作状态**（`IDLE` → `FRICTION_TEST` → `IDLE`）：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 topic echo /teleop/state std_msgs/msg/String --qos-durability transient_local
~~~

**终端 D2：静摩擦输出阶段**（`idle`/`off`/`on`）：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 topic echo "/leader_arm/$SIDE/friction_test_phase" \
  std_msgs/msg/String --qos-durability transient_local
~~~

**终端 D3：关节状态**（`position`/`velocity` 按 J1--J7 顺序，单位 rad/rad/s）：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 topic echo /leader_arm/current_state sensor_msgs/msg/JointState
~~~

**(optional) 终端 D4：原始命令电流**（数组按设备 ID 1--8 顺序，正负号保留，单位 mA）：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 topic echo "/leader_arm/$SIDE/friction_test_current_ma" \
  std_msgs/msg/Float64MultiArray
~~~

**终端 D5：所选关节命令电流绝对值**（仅显示 `|I_command[JOINT_ID]|`，单位 mA）：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 run skye_factr_teleop skye_factr_friction_test_monitor -- \
  --side "$SIDE" --joint-id "$JOINT_ID"
~~~

D5 收到命令电流后在**同一行**持续刷新最新绝对值，不逐行打印，也不计算峰值或均值；超过 1 s 没有新消息时显示 `--`。它每 0.05 s 刷新屏幕，带正负号的原始电流仍看 D4。逐帧命令与实际回读电流由终端 A 的 trace 保存。结束监看时按 Ctrl+C，不会停止测试节点。

### 4. 进入测试并现场比较

终端 C 先加载本轮会话，确认 `SIDE` 与实体侧别相符。若 `SIDE` 为空，不发送模式请求；重新运行步骤 1 的准备脚本。确认小臂受支撑后显式进入测试；从模式请求开始计总时长。命令写成单行，避免续行反斜杠后的空格使 shell 解析出错：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
printf '本轮侧别: %s\n' "${SIDE:?SIDE 未设置，请重新准备会话}"
ros2 topic pub --once "/mode/${SIDE:?SIDE 未设置}/switch_friction_test" std_msgs/msg/String '{data: switch_friction_test}'
~~~

同一终端 C 接着启动交互切换工具，等待它显示 `模式: FRICTION_TEST`、`输出阶段: off`。以后只需按回车切换 `off/on`，不用再次输入 ROS 发布命令：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_friction_trial.env
ros2 run skye_factr_teleop skye_factr_friction_test_switch -- --side "${SIDE:?SIDE 未设置}"
~~~

1. **观察 `off` 基准。** 终端 D1 应显示 `FRICTION_TEST`，D2 显示 `off`；D4 的 8 个 ID 命令均为零，D5 显示 `0.00 mA`。持续支撑小臂，轻缓推动目标关节，在准备切换的位置感受起动阻力；可以先移动到该位置，无须回到 `off` 开始时的姿态。
2. **原地切换到 `on`。** 停住并扶稳关节，不退出节点、不重新准备会话，也不改变支撑姿态。在终端 C **按一次回车**；工具会请求 `on`，收到节点确认后显示 `输出阶段: on`。如果未收到确认，先查看 D2 的阶段，再决定是否重试。

3. **观察静摩擦输出。** 终端 D2 应显示 `on`。待 2 s 渐增完成后，仍在相近姿态下轻缓推动目标关节，与刚才的 `off` 手感比较。D4 可看到带正负号的目标 ID 命令、其余 ID 应为零；D5 显示目标 ID 最新命令电流绝对值。D5 的值不得超过本轮上限；全程峰值和其他 ID 的零命令还需结束后从终端 A 的 trace 核对。再在支撑下缓慢移动目标关节至配置的 `enable_speed`（当前 YAML 为 `0.05 rad/s`）以上，对照 D3 的速度与 D5 的命令归零现象。电流与关节状态话题并非严格同步。
4. **需要再次对比时切回 `off`。** 在终端 C 再按一次回车；此后每按一次回车就按已确认的阶段交替切换。每次进入 `on` 都重新渐增，总时长仍从本节的模式请求开始计时。

全程不发送同步、走位或 follower 命令。出现异常或希望提前结束时，在终端 C **输入 `s` 并按回车**，确认 D1 返回 `IDLE`。若切换工具不可用，在另一个终端运行下面的独立停止命令：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws/install/setup.bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
~~~

### 5. 结束、回读和归档

可提前停止或等待总时长到期。确认终端 D1 显示 `IDLE`、D2 显示 `idle`，D4 的命令电流回到零、D5 显示 `0.00 mA`；终端 A 收到结束状态并保存末尾零命令后，按提示填写相对 `off` 阶段的起动阻力、阈值现象、异常运动、结束方式与备注，分辨不出时选 `unclear`。各监看终端可在此用 Ctrl+C 退出。

在终端 B 用 Ctrl+C 正常退出 launch，确认节点进程已结束。保持供电和支撑，不重启总线、不拔插 USB，立即做独立只读 Torque Disable 回读。只有回读通过、且终端 A 已生成本轮 JSON，才用**同一份**配对记录归档：

~~~bash
cd /home/lyy/projects/Skye_ROS_Bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /tmp/skye_factr_friction_trial.env
if ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
    --side "$SIDE" --config "$CONFIG" --frames 2 --interval 0 \
    --output "$READBACK"; then
  if [[ -f "$OUTPUT" ]]; then
    python3 src/skye_factr_teleop/scripts/finalize_friction_trial.py \
      --record "$OUTPUT" --readback "$READBACK"
  else
    printf '记录器 JSON 不存在，请检查终端 A：%s\n' "$OUTPUT"
  fi
else
  printf 'Torque Disable 回读失败；不执行归档，请先核对终端 B 与失能状态。\n'
fi
~~~

若回读报 `torque is enabled for device ID ...`，该检查会在首个使能 ID 处停止，其余 ID 尚未得到失能确认。不要继续下一轮；先确认测试节点已经停止和退出，复查终端 B 的关闭日志，再进行独立回读。若本轮 `.trace.jsonl` 和结果 JSON 都不存在，说明记录器没有成功开始记录；回看终端 A 的启动输出，本轮不能归档为通过。

若终端 B 的节点**仍在运行**，在切换工具输入 `s` 后回车（或使用上面的独立 `switch_stop` 命令），确认 `IDLE` 与命令电流全零，再用 Ctrl+C 正常退出终端 B；保持支撑和供电，用**新文件名**重新做只读失能回读，保留原失败文件。若节点**已经退出**而回读仍报告 Torque Enable，ROS 停止话题已没有接收者；只读回读工具也不会写失能寄存器。按现场安全流程支撑机械臂并断电，保留失败记录，排查终端 B 的关闭错误后再开始新一轮。

归档脚本检查 ID 1--8 Torque Disable 回读、节点正常退出和 `off → on` 阶段证据。`PASS` 还要求两阶段起始姿态接近、`off` 全零、`on` 观察到正负命令和阈值行为、起动阻力改善且没有异常运动；数据缺失或效果无法分辨时为 `INCOMPLETE` 或 `INCONCLUSIVE`。左右两侧 J1--J7 均完成后，才更新开发计划 7.2 状态。

已有分轮记录仍可使用 `prepare_friction_trial.py --single` 与旧的 `--baseline` 归档方式；新验收建议使用上述配对会话。

## 现场人工验收结果（2026-10-15）

操作者确认设备 A **左、右臂 ID 1--7 静摩擦补偿测试全部 `PASS`**，决定本次两侧按现场测试结论完成验收。当前独立入口只输出静摩擦项，不输出重力补偿；操作者在低电流测试中难以分辨 `off/on` 的起动阻力差异。本次接受静摩擦命令输出及现场安全观察为验收依据，减阻效果留待重力补偿稳定后，在相同重力输出下只切换静摩擦项复核。这是本次左右两侧验收口径的例外，不改变上文记录器对 `PASS` 的自动判定条件，也不代表减阻效果已由测量证实。

现存[左侧 ID1 配对记录](../../validation_records/phase7_2_3_A/left_id1_paired_20261015T065033482312Z.json)和[左侧 ID2 配对记录](../../validation_records/phase7_2_3_A/left_id2_paired_20261015T065513814076Z.json)均记录到 `off` 全零、`on` 仅目标 ID 正负命令、速度阈值样本、末尾零命令、正常退出及独立 Torque Disable 回读；两份原始 `assessment` 均为 `INCONCLUSIVE`，因为操作者当时将手感差异记为 `unclear`。目录另有[左侧 ID7 Torque Disable 回读](../../validation_records/phase7_2_3_A/left_id7_paired_20261015T084634710160Z_torque_disable.json)，但没有对应的配对汇总和逐帧轨迹。左侧其他关节及右侧 ID 1--7 的逐轮数据未保存在该目录。本节的左右两侧 `PASS` 是操作者现场结论，不能从现存文件逐关节独立复核；原始自动结果保持不变。

左侧 ID1、ID2 的逐帧轨迹还记录到非目标关节运动；轨迹不能判断这些运动是否由操作者主动带动。本次人工结论接受现场判断，不把 `operator_motion: none` 解释为所有非目标关节位置始终不变。
