# 阶段 7.2：单臂限位屏障低电流验收清单

状态：左右两侧整项真机验收均为 `PASS`。左侧 16 轮中 14 轮自动判为 `PASS`，J6 两轮按实际到达的边界人工复核为 `PASS`（原自动结果仍为 `FAIL`）；2026-10-15 操作者接受第 4 节记录中的检查缺口并判定左侧整项 `PASS`。右侧按第 5 节的现场人工结论验收 `PASS`。先确认该侧[阶段 7.1](../skye_factr_teleop_development_plan.md#71-完成阶段-252-真机验证与-253-冒烟验收记录)仍有效。

## 1. 每轮启动前

- 只连接并启动目标侧；小臂无负载，操作者持续支撑，可立即断电。确认没有其它进程占用串口或发布模式请求。
- 核对实体侧别、USB by-id、ID、已标定的 `joint_offsets`、`joint_signs` 和每个 ID 的 `current_limit_ma`。配置、装配、接线改变时先重验受影响侧只读方向。
- 核对 YAML 的臂关节 `limit_min/max` 与 `safety_margin` 对应的有效边界。夹爪 ID8 用独立 `gripper.limit_min/max`，没有臂关节安全裕量。**先确认该边界可在机械止挡之前安全触及；无法确认时不做真机试验。**
- 启动参数必须满足 `joint_id=1..8`、`0<gain_scale<=1`、`0<current_limit_ma<=50` 且不超过 YAML 任一 ID 上限、`ramp_seconds>=1`、`ramp_seconds<max_duration_seconds<=30`。从较小增益和电流上限开始，每次只测一个 ID 的一侧边界。

左臂 J1--J7 按 `safety_margin = 0.1 rad` 计算的有效边界如下：有效下边界为 `limit_min + safety_margin`，有效上边界为 `limit_max - safety_margin`；表内角度保留三位小数。配置变更后须重新计算，不能直接沿用本表。

| 关节 | 有效下边界 (deg) | 有效上边界 (deg) |
| --- | ---: | ---: |
| J1 | -166.158 | 166.158 |
| J2 | -108.862 | 108.862 |
| J3 | -166.158 | 166.158 |
| J4 | -131.780 | 51.566 |
| J5 | -166.158 | 166.158 |
| J6 | -45.837 | 45.837 |
| J7 | -40.107 | 40.107 |

左臂手爪 ID8 使用独立的 `gripper.limit_min/max`，不扣除 `safety_margin`：有效下边界 `-0.0175 rad = -1.003 deg`，有效上边界 `0.52517 rad = 30.090 deg`（角度保留三位小数）。

本模式启动时 Torque Disable；收到目标侧 `/mode/{side}/switch_limit_test` 才使能总线 8 个 ID。仅目标 ID 可能收到非零电流；其余 ID 为零电流，但零电流不提供机械支撑。安全区间内屏障输出为零，操作者须手动、缓慢接近有效边界；入口不会自动走位。屏障是软限位，不能保证机械限位绝不被越过。

## 2. 启动与记录

以下按一轮试验的时间顺序执行。每轮只测一个 ID 的一个边界；下一轮从步骤 2 重新开始。准备脚本会询问侧别、ID、边界、操作者和限值，自动选择 YAML、launch、话题和唯一记录文件名。终端 A/B/C 和可选监看终端共用 /tmp/skye_factr_limit_trial.env；同一时刻只进行一轮试验。命令可原样复制，无需改路径或文件名。

### 步骤 1：构建并加载环境（终端 A）

首次测试或源码变更后执行：

~~~bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-up-to skye_factr_teleop --symlink-install
source install/setup.bash
ros2 pkg executables skye_factr_teleop
~~~

确认最后一条列出 skye_factr_teleop_driver、skye_factr_limit_test_recorder 和 skye_factr_dynamixel_readonly_check。

### 步骤 2：选择本轮并核对设备（终端 A）

运行准备脚本，按提示选择侧别（left/right）、关节 ID（1--8）、测试边界（lower/upper），并填写操作者姓名。增益、电流上限、渐增时间和最长通电时间可直接回车，使用显示的默认值（0.1、20 mA、2 s、10 s）。脚本只读取配置、生成本轮会话文件 `/tmp/skye_factr_limit_trial.env`，不会打开串口：

~~~bash
python3 src/skye_factr_teleop/scripts/prepare_limit_trial.py
~~~

准备脚本会检查 YAML 中的 USB by-id 是否与设备 A 在 2.5.1 阶段验收的值一致（左侧 `FTBEO9L1`，右侧 `FTBEO8WL`）；不一致时不会生成会话文件。查看脚本打印的有效边界和 USB by-id，对照实体标签确认所选侧与接线，并确认目标有效边界能在机械止挡之前安全触及。

列出当前连接的 USB 串口链接，找到脚本打印的 by-id 名称。列表只能说明系统识别到了该设备，实体侧别仍需现场核对（可选）：

~~~bash
find /dev/serial/by-id -maxdepth 1 -type l -printf '%f -> %l\n' | sort
~~~

运行端口检查脚本：它会确认 by-id 与所选侧匹配、链接指向字符设备，然后将 FTDI `latency_timer` 设为 1 并回读确认；`sudo` 可能要求输入密码。`&&` 后的 `source` 只在检查成功时执行，把本轮变量加载到当前终端 A，供步骤 3 使用：

~~~bash
bash src/skye_factr_teleop/scripts/check_limit_trial_port.sh && \
  source /tmp/skye_factr_limit_trial.env
~~~

最后查看该 by-id 实际指向的串口是否被占用（可选）：

~~~bash
sudo fuser -v -- "$(readlink -f -- "/dev/serial/by-id/$USB_BY_ID")"
~~~

若列出 PID，先确认并停止占用该串口的进程；没有列出 PID 时再进入步骤 3。`fuser` 在没有进程占用时通常会返回状态码 1。

### 步骤 3：启动只读记录器（终端 A）

复制下面命令并保持运行。记录器只订阅监视话题，不发布模式请求：

~~~bash
ros2 run skye_factr_teleop skye_factr_limit_test_recorder -- \
  --side "$SIDE" --joint-id "$JOINT_ID" --boundary "$BOUNDARY" \
  --operator "$OPERATOR" --config "$CONFIG" \
  --gain-scale "$GAIN_SCALE" --current-limit-ma "$CURRENT_LIMIT_MA" \
  --ramp-seconds "$RAMP_SECONDS" --max-duration-seconds "$MAX_DURATION_SECONDS" \
  --output "$OUTPUT"
~~~

### 步骤 4：启动目标侧节点（新开终端 B）

节点启动后应处于 IDLE、Torque Disable，关节不会自动移动。终端 A 的记录器应显示“状态: IDLE”；若没有显示，先排查，不进入测试模式。

~~~bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /tmp/skye_factr_limit_trial.env
ros2 launch skye_factr_teleop "$LAUNCH_FILE" \
  config_path:="$CONFIG" joint_id:="$JOINT_ID" gain_scale:="$GAIN_SCALE" \
  current_limit_ma:="$CURRENT_LIMIT_MA" ramp_seconds:="$RAMP_SECONDS" \
  max_duration_seconds:="$MAX_DURATION_SECONDS"
~~~

### 步骤 5：进入测试并现场观察（新开终端 C）

确认小臂仍由操作者支撑、终端 A 已看到 IDLE。建议先打开现场监看，再发送模式请求，以免错过短暂的测试窗口。每个监看终端先执行以下环境命令，再各运行一条对应的 `ros2 topic echo`：

~~~bash
source /opt/ros/humble/setup.bash
source /home/lyy/projects/skye_ros_bridge/skye_ros2_ws/install/setup.bash
source /tmp/skye_factr_limit_trial.env
~~~

运行状态（应从 `IDLE` 变为 `LIMIT_TEST`，结束后回到 `IDLE`）：

~~~bash
ros2 topic echo /teleop/state std_msgs/msg/String
~~~

J1--J7 关节角度：`position` 按 J1--J7 顺序排列，单位 rad；`velocity` 单位 rad/s。ID8 不在此话题中。

~~~bash
ros2 topic echo /leader_arm/current_state sensor_msgs/msg/JointState
~~~

测试 ID8 时改看夹爪状态：数组第 1 项为位置（rad），第 2 项为速度（rad/s）。

~~~bash
ros2 topic echo "/leader_arm/$SIDE/limit_test_gripper_state" std_msgs/msg/Float64MultiArray
~~~

需要现场另看命令电流时，再开一个已加载环境的终端：

~~~bash
ros2 topic echo "/leader_arm/$SIDE/limit_test_current_ma" std_msgs/msg/Float64MultiArray
~~~

在终端 C 发送一次模式请求：

~~~bash
cd /home/lyy/projects/skye_ros_bridge/skye_ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
source /tmp/skye_factr_limit_trial.env
ros2 topic pub --once "/mode/$SIDE/switch_limit_test" \
  std_msgs/msg/String '{data: switch_limit_test}'
~~~

随后按以下顺序操作和判断；详细验收标准见第 3 节：

1. 确认状态已变为 `LIMIT_TEST`，继续支撑小臂。此时总线 8 个 ID 已使能，只有所选 ID 可能收到非零电流；其余 ID 的零电流不提供机械支撑。
2. 在有效边界内确认命令电流为零，然后手动、缓慢接近所选边界；节点不会自动走位。
3. 在有效边界外、原始限位内，核对角度和回推方向：下边界应推向角度增大，上边界应推向角度减小；同时确认无突动、无其它关节动作，所选 ID 命令电流不超过本轮上限，其余 ID 命令为零。
4. 方向或回推效果无法辨别时记录 `INCONCLUSIVE`；方向错误、突动、机械或线缆干涉、通信异常时立即按第 3 节停止并记录 `FAIL`。其余情况按步骤 6 停止或等待超时，并确认回到 `IDLE`、末尾命令电流全零；最终 `PASS` 还须完成步骤 7 的正常退出和 Torque Disable 回读。

记录器保存状态、所选轴位置和速度、8 个 ID 的软件命令与实际电流回读。结束 topic echo 用 Ctrl+C；它不会停止测试节点。命令和回读不是严格同一瞬间的值。

### 步骤 6：停止并回答记录器（终端 C → A）

可等待最长通电时间到期自动返回 IDLE，也可在终端 C 提前执行：

~~~bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
~~~

确认终端 A 收到 IDLE 和末尾零命令，按记录器提示填写实际施力方向、结束方式与备注。方向错误、突动或机械干涉时立即按第 3 节停止，不要等待超时。记录器生成本轮 JSON 和同名 .trace.jsonl，初步结果最多为 PENDING_SHUTDOWN_CHECK。

### 步骤 7：退出节点并回读 Torque Disable（终端 B → C）

终端 B 中按 Ctrl+C，确认 launch 正常退出、串口已释放。保持供电和支撑，不拔插 USB、不重启总线，也不启动其它控制程序。然后在终端 C 立即执行：

~~~bash
ros2 run skye_factr_teleop skye_factr_dynamixel_readonly_check -- \
  --side "$SIDE" --config "$CONFIG" --frames 2 --interval 0 \
  --output "$READBACK"
python3 - "$READBACK" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
record = json.loads(path.read_text())
devices = record["devices"]
assert record["automated_result"] == "PASS", record.get("error")
assert [device["id"] for device in devices] == list(range(1, 9))
assert all(device["torque_enabled"] is False for device in devices)
print(f"ID 1--8 Torque Disable PASS: {path}")
PY
python3 src/skye_factr_teleop/scripts/finalize_limit_trial.py \
  --record "$OUTPUT" --readback "$READBACK"
~~~

回读或断言失败时，本轮不得判为 PASS。最后一条命令会询问节点是否正常退出且回读前未断电或重启总线；如实输入 yes/no，它会把退出结果、回读文件路径和最终判定写入本轮 JSON，无需手工改文件。下一轮从步骤 2 重新选择，脚本会生成新文件名。

## 3. 每个 ID 的下边界和上边界

1. 在有效边界内确认命令为零。操作者持续支撑，缓慢接近目标边界；不要借助屏障推动关节，更不要越过原始机械边界。
2. 在下边界外、仍处于原始限位内时，确认逻辑恢复力矩指向增大角度；在上边界外时指向减小角度。现场观察需与命令电流及已验收的 `joint_signs` 一致，且没有突动或其它关节动作。夹爪按其独立上下限检查。
3. 确认所选 ID 命令电流绝对值不超过本轮上限，其余 ID 命令为零；记录位置、速度、命令与回读电流峰值。渐增同时限制测试项力矩比例和当时可用的电流上限；姿态变化也会改变力矩，不要求电流单调增长。
4. 若低电流下方向或回推效果无法辨别，记录 `INCONCLUSIVE`。方向错误、突动、机械或线缆干涉、通信异常，立即停止并记录 `FAIL`。

可提前结束：

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
```

超时也必须回到 `IDLE`、命令电流全零并 Torque Disable。记录器只给出 `PENDING_SHUTDOWN_CHECK` 等初步判定。**每轮节点正常退出后，保持供电且不重启总线，按[2.5.2 第 10 节](phase2_5_low_current_checklist.md#10-测试结束后复核-torque-disable)对本侧 ID 1--8 做只读 Torque Disable 回读**；第 2 节步骤 7 会保存独立文件并将退出结果和回读路径写入该轮记录。按这一逐轮记录流程，失联、强杀进程或没有回读时不得判为 `PASS`；右侧本次人工验收的单次例外见第 5 节。

通常左右各 J1--J7 与夹爪 ID8 的上下边界均有现场记录、停止与失能回读后，才能把本项限位屏障真机状态改为 `PASS`；右侧 2026-10-14 的现场人工验收按第 5 节的单次例外处理。

## 4. 左侧真机记录与判定（2026-10-13）

设备 A 左侧、操作者 `lyy`，ID 1--8 的实际上下边界各有一轮记录。下表按**实际到达的边界**列出并链接到原始 JSON；J6 两个链接的文件名与表中边界相反，详见表后备注。每份 JSON 指向对应的逐帧 trace 和独立 Torque Disable 回读。电流列为目标 ID 的**命令峰值/本轮上限**，单位 mA；增益倍率为本轮 `gain_scale`。

| ID | 下边界 | 上边界 | 增益倍率（下/上） | 命令峰值/上限（下；上） |
| --- | --- | --- | ---: | ---: |
| 1 | [PASS](../../validation_records/phase7_2_2_A/left_id1_lower_20261013T092952858246Z.json) | [PASS](../../validation_records/phase7_2_2_A/left_id1_upper_20261013T093502078180Z.json) | 0.1 / 0.1 | 20/20；20/20 |
| 2 | [PASS](../../validation_records/phase7_2_2_A/left_id2_lower_20261013T094309464006Z.json) | [PASS](../../validation_records/phase7_2_2_A/left_id2_upper_20261013T094826250880Z.json) | 0.5 / 0.5 | 20/20；16.825/20 |
| 3 | [PASS](../../validation_records/phase7_2_2_A/left_id3_lower_20261013T095704119004Z.json) | [PASS](../../validation_records/phase7_2_2_A/left_id3_upper_20261013T095452869598Z.json) | 0.1 / 0.1 | 20/20；20/20 |
| 4 | [PASS](../../validation_records/phase7_2_2_A/left_id4_lower_20261013T103147760769Z.json) | [PASS](../../validation_records/phase7_2_2_A/left_id4_upper_20261013T105132483227Z.json) | 0.5 / 1.0 | 20/20；35.666/40 |
| 5 | [PASS](../../validation_records/phase7_2_2_A/left_id5_lower_20261013T114226713513Z.json) | [PASS](../../validation_records/phase7_2_2_A/left_id5_upper_20261013T113541402025Z.json) | 1.0 / 1.0 | 20/20；20/20 |
| 6 | [人工复核 PASS](../../validation_records/phase7_2_2_A/left_id6_upper_20261013T114702316831Z.json) | [人工复核 PASS](../../validation_records/phase7_2_2_A/left_id6_lower_20261013T114537993663Z.json) | 1.0 / 1.0 | 20/20；20/20 |
| 7 | [PASS](../../validation_records/phase7_2_2_A/left_id7_lower_20261013T114858335364Z.json) | [PASS](../../validation_records/phase7_2_2_A/left_id7_upper_20261013T115133010579Z.json) | 1.0 / 1.0 | 20/20；20/20 |
| 8 | [PASS](../../validation_records/phase7_2_2_A/left_id8_lower_20261013T115329071053Z.json) | [PASS](../../validation_records/phase7_2_2_A/left_id8_upper_20261013T115450885698Z.json) | 1.0 / 1.0 | 6.609/20；19.839/20 |

16 轮均记录到 `IDLE → LIMIT_TEST → IDLE`、末尾零命令、其他 7 个 ID 命令峰值为零，测试节点正常退出；各轮独立回读均为 `PASS`，本侧 ID 1--8 均为 Torque Disable。所有目标 ID 命令峰值均未超过本轮上限。操作者在 16 轮中均填写 `inward`、`timeout`，备注为空。

**J6 人工复核备注：**操作者确认两轮均转向了预选边界的相反一侧，但手感均与实际转动方向相反。标为下边界的一轮位置为 `-0.037～0.980 rad`，实际越过上边界 `0.8 rad`；在该侧非零命令电流为正，结合 J6 的 `joint_signs=-1`，逻辑力矩指向角度减小。标为上边界的一轮位置为 `-1.163～0.153 rad`，实际越过下边界 `-0.8 rad`；在该侧非零命令电流为负，逻辑力矩指向角度增大。因此按**实际到达的边界**，J6 上、下两侧的回推方向均人工复核为 `PASS`。原始 JSON 的 `boundary` 标签填反，记录器分别将 3181、2926 个非零命令样本按预选边界判为反向，故两个 JSON 的自动 `assessment=FAIL` 保持原样；本表的人工复核结论不改写原始自动判定。

另外，16 轮中有 13 轮的记录位置超出该 ID 在 YAML 中配置的原始 `limit_min/max`，包括 11 轮自动判为 `PASS` 的记录和上述 J6 两轮。例如 ID1 的两侧分别达到 `-3.648` 和 `3.648 rad`（配置为 `-3～3 rad`），ID3 上侧达到 `3.863 rad`（配置上限 `3 rad`）。ID8 下边界一轮的位置始终在 `-0.040～-0.034 rad`，低于其有效下边界 `-0.0175 rad`，未记录到安全区间内的零命令检查。这些角度不等同于已碰到实体机械止挡，但与本清单“不越过原始边界”的操作要求不符。记录器的自动 `PASS` 没有检查这些条件；后续使用该配置时仍需核对逻辑角度、配置边界及实体机械范围。

**本批左侧整体判定：`PASS`（2026-10-15，操作者验收决定）。**限位回推方向按实际边界 16/16 轮通过；操作者接受现有记录中的越界样本和 ID8 安全区间内零命令检查缺口，决定将左侧限位屏障整项真机验收记为 `PASS`。上述记录事实及原始 JSON 的自动判定均保留。

## 5. 右侧现场人工结果（2026-10-14）

操作者报告：右臂限位屏障测试已完成，ID 1--8 的下边界和上边界均为**现场人工 `PASS`**。操作者确认以现场回推手感和方向判断接受本次结果，认为继续保存逐轮数据会增加操作负担；**本次右侧限位屏障真机验收结论为 `PASS`**。本次未保存逐轮测试数据，也未提供各轮命令电流、状态和测试后 Torque Disable 回读记录；这些指标不能从本次记录独立复核。此决定仅适用于右侧本次限位屏障测试，不改变左侧及其他验收项的记录要求。
