# Orin 双臂启动遥操

本机为 **遥操设备 B（Orin）**。用主机包 `skye_factr_teleop`，**不进 Docker、不用** `marvin_ws` **小臂**。

> 除「本机取值」表里的 FTDI 序列号、domain、小臂参数外，本文命令一律用**仓库内相对路径**，换机器可直接用。

| 项               | 本机取值                                                           |
| --------------- | -------------------------------------------------------------- |
| `robot_profile` | `orin`                                                         |
| `ROS_DOMAIN_ID` | `21`（A / B / D 必须相同）                                           |
| 左臂 FTDI         | `FTB8HNOT`（以本机 `/dev/serial/by-id` 与 yaml 为准）                  |
| 右臂 FTDI         | `FTAO51EA`                                                     |
| 小臂配置            | `skye_factr_teleop/config/orin/grav_comp_m6_{left,right}.yaml` |

Thor 对照：[`双臂启动遥操.md`](./双臂启动遥操.md)（Thor domain=`20`）。旧 Docker：[`Thor_Orin_遥操启动.md`](../Thor_Orin_遥操启动.md)。

串口已写在 Orin yaml。**换线 / by-id 对不上**时再绑一次（见下「换 FTDI」）。

---

## 前置依赖（每台新机器一次）

`package.xml` 声明的运行时依赖，缺任何一个都会在启动瞬间崩：

```bash
sudo apt install -y ros-humble-dynamixel-sdk python3-serial ros-humble-pinocchio
```

- `dynamixel_sdk` 缺 → `setup_factr_teleop_env.sh` 报 `No module named 'dynamixel_sdk'`
- `python3-serial` 缺 → `dynamixel_sdk` 内部 `import serial` 失败，报 `No module named 'serial'`
- `pinocchio` 缺 → launch 起来后 `ros_driver.py` 报 `No module named 'pinocchio'`

自检与编译（仓库根目录）：

```bash
/usr/bin/python3 -c 'import pinocchio, dynamixel_sdk, serial; print("ok")'
bash scripts/teleop.sh build
```

---

## 一分钟流程（照抄）

开 **3 个终端**，每个都先（在仓库根目录）：

```bash
cd "$(git rev-parse --show-toplevel)"                          # 任意目录 → 仓库根目录
source skye_ros2_ws/scripts/setup_factr_teleop_env.sh --robot orin
unset FASTRTPS_DEFAULT_PROFILES_FILE   # 避免指向已删除的 marvin_ws/fastrtps xml
echo "DOMAIN=$ROS_DOMAIN_ID PROFILE=$ROBOT_PROFILE"   # 期望 21 / orin
```

> `source` 之后 shell 会自动 **cd 到** `skye_ros2_ws`，并导出 `DRIVER_CONFIG` /
> `LEFT_CONFIG` / `RIGHT_CONFIG`。后面要跑仓库根目录的脚本时，先 `cd "$(git rev-parse --show-toplevel)"` 回去。

| 终端 | 做什么 |
|---|---|
| **A 大臂** | 下面「A」 |
| **B 双小臂** | 先做「串口」，再下面「B」 |
| **D 控制** | 托住小臂后再发 sync / teleop；**不要在 A/B 里发** |

### A — 大臂

```bash
ros2 launch skye_robot_driver skye_robot_driver.launch.py \
  robot_profile:=orin "params_file:=$DRIVER_CONFIG"
```

看到连上控制器、切入 `STATE_IMP_JOINT` 即可。`$DRIVER_CONFIG` 由上面的 `source` 提供；跳过那步会变成空参数。

### B — 双小臂（先串口）

```bash
ls -l /dev/serial/by-id/
# 期望含 FTAO51EA（右）、FTB8HNOT（左）；须与 orin yaml 的 dynamixel_port 一致

# 一次性装 udev 规则后（sudo ./scripts/setup_ftdi_latency.sh）插拔/重启都自动为 1，
# 下面这步可跳过；未装规则时 latency_timer 只有 root 能写，ttyUSB 编号会变，按 by-id 推导：
for l in /dev/serial/by-id/*FTDI*; do
  d=$(basename "$(readlink -f "$l")")
  echo 1 | sudo tee "/sys/bus/usb-serial/devices/$d/latency_timer"
done

# 需要重新绑定小臂 FTDI 时（先回仓库根目录）：
cd "$(git rev-parse --show-toplevel)"
pkill -f skye_factr_teleop_driver || true
export ROBOT_PROFILE=orin
python3 scripts/bind_leader_arms.py
# 按提示拔【左臂】USB → 回车 → 插回 → 回车
# 脚本末尾的 "latency 失败 ... Permission denied" 属正常（它没有 root）；
# 绑定本身已成功，latency 用上面的 for 循环补一次。

ros2 launch skye_factr_teleop start_teleop_m6_dual.launch.py \
  robot_profile:=orin sync_diagnostics:=true
```

**必须**写 `robot_profile:=orin`：这个 launch 的默认值**硬编码为** `thor`，并且**不读环境变量**，
`export ROBOT_PROFILE=orin` 对它无效（只有大臂 launch 有 env 回退）。漏写就按 Thor 配置跑（`joint_signs` 等全是 Thor 的）。

启动后是 **IDLE、力矩关**，小臂不会自己动。

### D — 同步 → 遥操（托住双小臂）

```bash
# 先确认大臂反馈在本 domain 可读：
ros2 topic hz /gento/left_joint_states
ros2 topic hz /gento/right_joint_states

# 查看是否同步到位
ros2 run skye_factr_teleop skye_factr_dual_trial   --phase sync --left-config "$LEFT_CONFIG" --right-config "$RIGHT_CONFIG"   --operator lyy --duration 180

ros2 topic pub --once /mode/switch_sync std_msgs/msg/String '{data: switch_sync}'
# 看 B 日志走到 SYNCED（两侧），再：
ros2 topic pub --once /mode/switch_teleop std_msgs/msg/String '{data: switch_teleop}'
# 静止 5～10 s，再慢动
```

状态：

```text
IDLE ──switch_sync──► … ──► SYNCED   # 小臂使能，跟大臂；还不发大臂命令
SYNCED ──switch_teleop──► TELEOP     # 开始发 /gento/{left,right}_joint_control
任意 ──switch_stop──► IDLE           # 关小臂力矩；不自动 hold 大臂
```

确认：

```bash
ros2 topic echo /teleop/state --once \
  --qos-durability transient_local --qos-reliability reliable
```

TELEOP 后扳机 J8 → 大臂夹爪：`/left|right_teleop_gripper/ctrl`。

### 停止（托住双小臂）

```bash
ros2 topic pub --once /mode/switch_stop std_msgs/msg/String '{data: switch_stop}'
ros2 service call /gento/hold_current std_srvs/srv/Trigger '{}'

# 查看小臂温度
ros2 topic echo /leader_arm/left/temperature_c
ros2 topic echo /leader_arm/right/temperature_c
```

---

## 串口说明

`ttyUSB*` 编号会变，以 **by-id** 为准：

```text
..._FTAO51EA-if00-port0 -> ../../ttyUSB?   # 右
..._FTB8HNOT-if00-port0 -> ../../ttyUSB?   # 左
```

没有 by-id：检查供电/线缆，或：

```bash
sudo udevadm trigger && sudo udevadm settle
ls -l /dev/serial/by-id/
```

USB 重插或重启后 `latency_timer` 会复位，启小臂前再写一次 `1`。

---

## 换 FTDI / 串口对不上（推荐）

`serial port does not exist: ...FTDI...` 时，**不要手改一堆路径**，在仓库根目录：

```bash
cd "$(git rev-parse --show-toplevel)"
# 两臂上电、USB 插好；先停掉占用串口的 teleop
pkill -f skye_factr_teleop_driver || true

export ROBOT_PROFILE=orin
python3 scripts/bind_leader_arms.py
# 正好 2 路：按提示拔【左臂】USB → 回车 → 插回 → 回车
# 脚本写入 skye_factr_teleop config/orin + install（若非 symlink）
# latency=1 一步会因权限失败，属正常，见上面「B」的 for 循环
```

预览：`python3 scripts/bind_leader_arms.py --dry-run`。然后重新开终端 B。

---

## 改配置 / 重编注意

- Launch 读的是 `install/.../share/.../config/orin/`（symlink-install 时链到 `src`）。
- 只改 `src` 后若 share **不是** symlink：跑上面的 bind（会双写），或
`colcon build --packages-select skye_factr_teleop --symlink-install`
然后 **重启 B**。
- 脚本打印的 `LEFT_CONFIG`/`RIGHT_CONFIG` 已是本机 profile 路径；真正用哪份由 `robot_profile:=orin` 决定。

---

## 常见问题

| 现象 | 原因 | 处理 |
|---|---|---|
| `No module named 'pinocchio'` | 缺 `ros-humble-pinocchio` | `sudo apt install -y ros-humble-pinocchio` |
| `No module named 'dynamixel_sdk'` / `'serial'` | 缺 SDK / pyserial | `sudo apt install -y ros-humble-dynamixel-sdk python3-serial` |
| `serial port does not exist: ...FTDI...` | yaml by-id ≠ 本机插着的 FTDI | `ROBOT_PROFILE=orin python3 scripts/bind_leader_arms.py` 后重启 B |
| bind 打印 `latency 失败 ... Permission denied` | `/sys` 需 root | 正常现象；用「B」的 for 循环补写 |
| by-id 目录空 / 只有一根 | USB 未起来 | 查供电线缆；拔插 / `udevadm` |
| `latency_timer ... expected 1` | 时延未设 | 写 `latency_timer=1` 后重启 B |
| `Waiting for subscription` / sync 无反应 | A/B/D domain 不一致 | 都 `source ... --robot orin`，确认 `21`，**重启** A/B |
| B 狂刷 `Rejected follower state: ... stale` | 大臂反馈时间戳不可用；**sync 不会启动** | 三端同为 21；`unset` 坏的 FASTRTPS 路径；必要时 `ROS_LOCALHOST_ONLY=1` 排除局域网串台后再试 |
| `Synchronization requires fresh follower state` | 同上，没有新鲜 `/gento/*_joint_states` | 先 `ros2 topic hz /gento/left_joint_states` 有约 250 Hz 再 sync |
| `gripper_setpoint` unknown / yaml 字段报错 | install 代码与 yaml 版本不一致 | `colcon build --packages-select skye_factr_teleop --symlink-install` 后重启 B |
| `[XMLPARSER Error] realpath failed` | `FASTRTPS_DEFAULT_PROFILES_FILE` 失效 | `unset FASTRTPS_DEFAULT_PROFILES_FILE` |
| 启动时打几行 `sequence size exceeds remaining buffer` | DDS 反序列化某条样本失败（缓冲不足或对端消息定义不一致），该样本被丢弃 | 非致命；launch 已自动加载 `config/fastrtps_teleop.xml`（4 MB UDP 缓冲）+ `scripts/setup_host_tuning.sh`（rmem_default 8 MB） |
| 小臂只轻微动一下就停 | 常见是 stale 拒绝跟随，或旧 install 仍是 50 mA 验证电流 | 先看 B 是否刷 stale；再确认 orin yaml 的 `current_limit_ma` 已是 Thor 基线并已进 install |

杀残留大臂（只能有一个 SDK 客户端）：

```bash
pkill -f skye_robot_driver || true
pkill -f gento_robot_driver || true
```

---

## 通用 vs 本机特有

| 内容 | 换机器是否通用 |
|---|---|
| 前置依赖、`bash scripts/teleop.sh build` | 通用（每台新机器装一次） |
| `robot_profile:=orin`、`sync_diagnostics:=true`、`--robot orin` | 通用 |
| sync / teleop / stop 话题、`/gento/hold_current` | 通用（代码里的固定接口） |
| 相对路径命令 | 通用（任意 clone 目录都能跑） |
| `ROS_DOMAIN_ID=21`（Thor 为 20） | **本组约定**，A/B/D 三端必须一致 |
| 左 `FTB8HNOT` / 右 `FTAO51EA` | **本机特有**，换机器必须重跑 `bind_leader_arms.py` |
| `config/orin/grav_comp_m6_*.yaml` 里的 `dynamixel_port` / `current_limit_ma` | **随机器走**，由绑定与实测决定 |
| `ttyUSB*` 编号、`latency_timer` | **每次拔插/重启都会变**，按 by-id 推导 |
| 控制器 IP（Thor `6.6.7.191` / Orin `6.6.7.190`） | 按机型固定，见大臂 profile |

---

## 相关

- Thor：[`双臂启动遥操.md`](./双臂启动遥操.md)
- 排障总览：[`小臂reboot操作说明.md`](./小臂reboot操作说明.md)
- 环境：`skye_ros2_ws/scripts/setup_factr_teleop_env.sh`
- Orin 小臂参数：`skye_ros2_ws/src/skye_factr_teleop/config/orin/`
- 大臂：`skye_robot_driver/config/skye_robot.yaml` + `profiles/orin.yaml`
