# 阶段 6：基础功能无硬件集成验证

验收日期：2026-09-29。目标运行环境由用户确认为当前主机 `lyy-ViTai`，
Linux 6.8.0-138-generic、x86_64。整个阶段未连接或打开 Dynamixel 串口。

## 6.1 全包测试

在 `skye_ros2_ws` 执行：

```bash
colcon build --packages-select skye_factr_description skye_factr_teleop --symlink-install
colcon test --packages-select skye_factr_teleop
colcon test-result --verbose
```

两个包构建通过；测试结果为 **288 tests, 0 errors, 0 failures, 0 skipped**。
ROS 图测试使用 DDS，本环境需允许本地网络套接字。

## 6.2 静态资源

加载 `install/setup.bash` 后，`ros2 pkg prefix` 可分别定位两个包。
安装产物包含 4 份控制配置、3 个 launch、3 个 URDF 和 38 个 mesh 文件。
对运行时代码、config、launch、URDF 和相应安装产物检索，未发现
`/marvin_ws`、对旧 `factr_teleop` 模块的导入、`python_utils` 或 `zmq` 导入。
历史开发文档中保留了说明旧路径不得用于运行时的文字。

## 6.3 ROS 图

`test_dual_ros_graph_types_remaps_and_isolation` 用注入的假硬件启动左右真实
`rclpy` 节点，并经 ROS 图检查：节点名唯一、每侧仅订阅自己的 7 轴 follower
状态、leader current/target 左右独立、`/joint_control` 正确 remap 到
`/gento/<side>_joint_control`。测试通过 `ros2 topic pub --once` 发布 sync、
teleop 和 stop 请求，验证两侧均收到全局模式请求，且 stop 后无遥操命令继续发布。
未启动键盘节点。状态订阅和控制发布的 QoS 分别为 RELIABLE/KEEP_LAST 1 和
BEST_EFFORT/KEEP_LAST 1，与 `skye_robot_driver` 的状态发布、控制订阅匹配。

## 6.4 软件性能基线

运行 `scripts/measure_stage6_software.py`，单线程 executor 同时运行左右 500 Hz
节点；Pinocchio 和 Ruckig 为真实实现，Dynamixel 为内存假设备，follower 状态以
100 Hz 发布。同步轨迹阶段连续测量 5 秒，每侧取得 2500 个定时器间隔。
原始统计见 [software_performance_2026-09-29.json](software_performance_2026-09-29.json)。

| 指标 | 左侧 | 右侧 |
| --- | ---: | ---: |
| 定时器间隔均值 / p99 / 最大值 (ms) | 2.000 / 2.205 / 2.560 | 2.000 / 2.194 / 2.457 |
| 完整控制回调 p99 / 最大值 (µs) | 410 / 586 | 430 / 571 |
| Pinocchio 单次 p99 (µs) | 7.1 | 9.9 |
| Ruckig 单次 p99 (µs) | 17.5 | 19.4 |
| follower ROS 回调 p99 (µs) | 32.9 | 22.7 |
| 超过 4 ms 的控制间隔 | 0 | 0 |
| 估算漏掉的 2 ms 定时器槽位 | 0 | 0 |

“漏掉槽位”按相邻控制回调开始时刻的间隔计算：
`max(0, round(interval / 2 ms) - 1)`，是无硬件控制定时器丢帧的代理指标。
本机软件基线冻结为上述实测分布；后续在相同双臂、单线程、假硬件条件下对照。
串口同步读写及连接硬件后的控制周期仍需在阶段 7.1 通过后测量。

复测命令：

```bash
cd skye_ros2_ws
source install/setup.bash
python3 src/skye_factr_teleop/scripts/measure_stage6_software.py --seconds 5
```
