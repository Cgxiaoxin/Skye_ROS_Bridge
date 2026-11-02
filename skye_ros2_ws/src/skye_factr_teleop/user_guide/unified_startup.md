# 原生大小臂统一启动

在仓库根目录执行。该入口面向当前 `skye_factr_teleop`，两套小臂可以交叉接到主机。

## 首次设置与编译

```bash
cd /home/lyy/projects/skye_ros_bridge
bash scripts/teleop.sh configure thor
bash scripts/teleop.sh build
```

`configure thor` 选择控制器 `6.6.7.191` 及 Thor 大臂参数；`configure orin` 选择
`6.6.7.190` 及 Orin 大臂参数。选择保存到已有的 `marvin_ws/.skye/robot_profile`
（或已有脚本的回退位置），每台主机只需设置一次。它表示本机要控制的大臂，与接入哪套小臂无关。
临时换大臂可在 `check`、`start`、环境脚本中传入 `--robot thor` 或 `--robot orin`。
优先级为显式 `--robot`、`ROBOT_PROFILE` 环境变量、已保存配置；未设置时会提示配置。

`build` 自动加载 Humble，依次编译小臂及依赖、大臂驱动，保持大臂普通安装方式。
首次和源码、launch、安装资源变更后编译；只改当前源 YAML 时重启即可。

## 日常检查与启动

```bash
bash scripts/teleop.sh check --fix-latency
bash scripts/teleop.sh start
```

`check` 核对源配置、串口权限、占用、FTDI 延迟、控制器 ping 和已安装入口，不读取电机寄存器。
不加 `--fix-latency` 时只检查；加上时按需将延迟设为 1 ms，权限不足会调用 `sudo tee`。
`start` 包含同样检查及延迟设置，自动加载 ROS 环境并在一个 ROS launch 下启动大臂和选中的小臂。
大臂会连接真机并进入原配置控制模式，包含配置中的大臂夹爪；小臂初始为 IDLE，**不自动同步或进入遥操**。
启动前电机只读检查仍按[阶段 7.5 checklist](../dev_docs/checklist/phase7_5_dual_follower_sync_teleop_checklist.md)执行。

USB 自动识别使用现有 YAML 中的唯一序列号：

| 小臂实体 | 左 USB 序列号 | 右 USB 序列号 | 使用的源配置目录 |
|---|---|---|---|
| A | FTBEO9L1 | FTBEO8WL | `config/` |
| B | FTB8HNOT | FTAO51EA | `config/orin/` |

`config/orin/` 是历史目录名，在此入口中表示设备 B 标定，**不要求大臂选择 Orin**。
接入 B 并选择 Thor 时，大臂使用 Thor 参数、小臂使用 B 各侧的方向、零点和控制参数。
左右分别识别；缺少所选侧或同侧同时检测到 A/B 时停止并给出提示，不按 `ttyUSB0/1` 顺序猜测。
更换新的 USB 转换器时需先在对应小臂 YAML 中登记其唯一标识，并确认实体侧别和标定。

```bash
bash scripts/teleop.sh start --side left         # 只启动左小臂及一份双侧大臂驱动
bash scripts/teleop.sh start --side right
bash scripts/teleop.sh start --device b          # 两套同时接入时明确选 B
bash scripts/teleop.sh start --robot thor --dry-run  # 仅打印匹配配置及命令，不设置延迟或启动节点
```

domain 默认 Thor=73、Orin=22，可通过 `ROS_DOMAIN_ID` 覆盖，再由 `--domain` 覆盖；
入口始终显示有效 domain。跨终端务必使用相同的大臂、侧别、设备选择及 domain。
不会根据主机 CPU 架构或 USB 小臂身份推断要控制的大臂。

## 记录与操作终端

另开终端，在仓库根目录执行：

```bash
source skye_ros2_ws/scripts/setup_factr_teleop_env.sh
```

环境脚本自动加载 Humble、当前工作区及同一套识别规则，导出 `LEFT_CONFIG`、`RIGHT_CONFIG`、
`DRIVER_CONFIG`、`ROBOT_PROFILE` 和 ROS 环境。单侧模式仅导出对应侧配置。
临时选项与启动终端保持一致，例如：

```bash
source skye_ros2_ws/scripts/setup_factr_teleop_env.sh --robot thor --device b --domain 73
```

环境脚本会切换到 `skye_ros2_ws`。阶段 7.5 的双臂记录器、只读检查及 sync / teleop / stop / hold 命令可继续使用，
其中配置路径变量已根据实际 USB 选择。同步和遥操仍须分别确认两侧状态。

## 停止与退出

先按 checklist 请求共同 stop，分别确认两侧 IDLE；进入过遥操时再调用大臂 hold 并确认成功。
然后在统一启动终端按 Ctrl+C，ROS launch 统一退出大小臂；任一驱动进程退出时也会请求退出其余进程。
该退出机制不替代 stop / hold，不能当作两侧故障联锁。释放串口后再执行两侧独立失能回读。
