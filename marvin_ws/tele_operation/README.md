# Tele Operation

`tele_operation` 是一个基于 ROS 2 的遥操作数据采集项目，主要负责把 RealSense 相机、ViTai/GF225 触觉传感器和机器人状态数据统一发布到 ROS topic，并通过同步订阅的方式按 episode 保存为 `.pkl` 数据。

项目的核心数据流如下：

```text
硬件设备
  ├─ RealSense 相机
  ├─ ViTai / GF225 触觉传感器
  └─ 机器人 ROS 状态 topic
        ↓
DeviceMappingServer 生成设备到 topic 的映射
        ↓
camera_node_launcher.py 启动多个发布器进程
        ↓
ROS 2 topics: 图像、触觉 marker、机器人状态
        ↓
record_data.py / DataRecorder 同步订阅并缓存
        ↓
FastAPI 控制 start/save episode
        ↓
episode_0000.pkl, episode_0001.pkl, ...
```

## 目录结构

```text
.
├── camera_node_launcher.py              # 启动设备映射服务，并按配置启动相机/触觉发布器
├── record_data.py                       # 启动数据记录节点
├── config/
│   └── real_world_env.yaml              # 机器人、发布器、设备映射服务配置
├── publisher/
│   ├── realsense_camera_publisher.py    # RealSense RGB 图像发布器
│   └── tactile_sensor_publisher.py      # ViTai/GF225 触觉图像和 marker 发布器
├── teleoperation/
│   └── data_recorder.py                 # 同步订阅 topic，按 episode 保存数据
└── common/
    ├── data_models.py                   # Pydantic 数据模型和控制枚举
    ├── ros_data_converter.py            # ROS 消息到 numpy/Pydantic 数据结构的转换
    ├── space_utils.py                   # 位姿、旋转、滤波等空间数学工具
    ├── time_utils.py                    # ROS 时间戳转换和同步检查工具
    └── device_mapping/
        ├── device_mapping_server.py     # FastAPI 设备映射服务
        └── device_mapping_utils.py      # 根据映射生成订阅 topic 列表
```

## 运行环境

代码依赖 ROS 2 Python 生态和若干硬件 SDK。实际版本需要和本机 ROS 2、RealSense、ViTai SDK 安装保持一致。

主要依赖包括：

- ROS 2 Python：`rclpy`、`sensor_msgs`、`geometry_msgs`、`message_filters`
- RealSense：`pyrealsense2`
- ViTai 触觉传感器：`pyvitaisdk`
- Web API：`fastapi`、`uvicorn`、`requests`
- 配置和日志：`hydra-core`、`omegaconf`、`loguru`
- 数值与图像：`numpy`、`opencv-python`、`scipy`、`transforms3d`、`numba`
- 系统工具：`psutil`、`v4l2-ctl`

示例安装命令只覆盖 Python 依赖，ROS 2、RealSense SDK、ViTai SDK 通常需要单独安装：

```bash
pip install fastapi uvicorn requests hydra-core omegaconf loguru numpy opencv-python scipy transforms3d numba psutil pydantic
```

## 配置文件

主配置在 `config/real_world_env.yaml`。

### 机器人配置

```yaml
robot_name: 'marvin'
robot_server:
  host_ip: "192.168.1.165"
  port: 8092
  robot_ip: "6.6.7.190"
  use_arm: ['A','B']
```

`use_arm` 用于控制记录器订阅哪些机械臂状态：

- `A`：左臂 topic
- `B`：右臂 topic
- `['A', 'B']`：双臂

### RealSense 发布器配置

```yaml
publisher:
  realsense_camera_publisher:
    - camera_serial_number: '327322061084'
      camera_type: 'D400'
      camera_name: 'external_camera'
      rgb_resolution: [320, 240]
      depth_resolution: [320, 240]
      random_sample_point_num: 3000
      fps: 24
      decimate: 2
      cpu_core_id: [0, 1, 2]
```

关键字段：

- `camera_serial_number`：RealSense 序列号，用于匹配物理设备。
- `camera_type`：RealSense product line，例如 `D400`。
- `camera_name`：生成 ROS topic 的逻辑名称。
- `rgb_resolution`：RGB 图像分辨率。
- `fps`：发布频率。
- `cpu_core_id`：该发布器进程绑定的 CPU 核。

### 触觉传感器配置

```yaml
publisher:
  tactile_sensor_publisher:
    - sensor_name: 'left_gripper_sensor_1'
      sensor_type: 'GF225'
      sensor_sn: "GF2251386F6E6"
      fps: 24
      marker_dimension: 3
      cpu_core_id: [9, 10, 11]
```

关键字段：

- `sensor_name`：触觉传感器逻辑名称。
- `sensor_sn`：ViTai/GF225 设备序列号。
- `marker_dimension`：marker 偏移维度，常用 `2` 或 `3`。
- `cpu_core_id`：该发布器进程绑定的 CPU 核。

### 设备映射服务

```yaml
device_mapping_server:
  host_ip: '127.0.0.1'
  port: 8062
```

映射服务提供接口：

```text
GET /get_mapping
```

返回结构示例：

```json
{
  "realsense": {
    "external_camera": {
      "topic_image": "external_camera/color/image_raw",
      "device_id": "327322061084",
      "type": "realsense"
    }
  },
  "tactile_sensor": {
    "left_gripper_sensor_1": {
      "topic_image": "/left_gripper_sensor_1/color/image_raw",
      "device_id": "GF2251386F6E6",
      "type": "tactile_sensor"
    }
  }
}
```

## 启动发布器

启动设备映射服务，并根据映射启动 RealSense 和触觉传感器发布器：

```bash
python camera_node_launcher.py
```

`camera_node_launcher.py` 的主要逻辑：

1. 读取 `config/real_world_env.yaml`。
2. 启动 `DeviceMappingServer`，通过 FastAPI 暴露 `/get_mapping`。
3. 请求最新设备映射。
4. 根据映射为每个 RealSense / 触觉传感器启动独立 `multiprocessing.Process`。
5. 为每个发布器进程设置 CPU affinity，降低多设备采集时的抖动。
6. 捕获 `KeyboardInterrupt` 后向子进程发送 `SIGUSR1`，调用各发布器的 `stop()` 清理硬件资源。

注意：代码尝试把主进程调度策略设置为 `SCHED_RR` 且优先级为 `99`，通常需要 root 权限或实时调度权限。若权限不足，程序会抛出 `Failed to set scheduler`。

## 启动数据记录器

```bash
python record_data.py --save_file_dir /path/to/save/pkl/
```

默认保存目录在代码中为：

```text
/home/vitai/wyz/shanghai_code/vr_data/dewu/pkl/
```

建议显式传入 `--save_file_dir`，避免数据写到旧路径。

`record_data.py` 会启动 `DataRecorder` 节点，并在内部启动一个 FastAPI 控制服务。控制接口来自 `teleoperation/data_recorder.py`：

```text
POST /start_episodes
POST /save_episodes
```

示例：

```bash
curl -X POST http://192.168.1.165:8092/start_episodes
curl -X POST http://192.168.1.165:8092/save_episodes
```

其中 IP 和端口来自配置：

```yaml
teleop_server:
  data_recorder_ip: ...
  data_recorder_port: ...
```

如果配置文件中没有 `teleop_server` 段，需要补齐，否则 `record_data.py` 会读取配置失败。

## ROS Topics

设备 topic 由设备映射和 `common/device_mapping/device_mapping_utils.py` 共同决定。

### 相机 topic

RealSense RGB：

```text
/{camera_name}/color/image_raw
```

触觉传感器 RGB：

```text
/{sensor_name}/color/image_raw
```

触觉 marker 偏移：

```text
/{sensor_name}/marker_offset/information
```

当前 `get_topic_and_type()` 中触觉 marker topic 被注释，记录器默认只订阅触觉 RGB，不订阅 marker 偏移。如果需要保存 marker，需要取消对应 `PointCloud2` 订阅并导入 `PointCloud2` 类型。

### 机器人状态 topic

左臂 `A`：

```text
/left_joints
/left_tcp_pose
/left_gripper_state
```

右臂 `B`：

```text
/right_joints
/right_tcp_pose
/right_gripper_state
```

动作命令 topic 在代码中已有转换逻辑，但订阅列表里目前被注释：

```text
/left_action_cmd
/left_gripper_cmd
/right_action_cmd
/right_gripper_cmd
```

## 数据保存格式

数据最终保存为 pickle：

```text
{save_file_dir}/episode_0000.pkl
{save_file_dir}/episode_0001.pkl
...
```

每个 pickle 文件保存的是 `list[Vitai_SensorMessage_Double]`。

`Vitai_SensorMessage_Double` 的主要字段：

| 字段 | 含义 | 默认形状 / 类型 |
| --- | --- | --- |
| `timestamp` | 当前同步帧的最新时间戳 | `float` |
| `leftRobotTCP` | 左臂 TCP 位姿，`x,y,z,r,p,y` | `(6,) float32` |
| `leftRobotJoints` | 左臂关节角 | `(7,) float32` |
| `leftRobotGripperState` | 左夹爪状态，宽度和力 | `(2,) float32` |
| `rightRobotTCP` | 右臂 TCP 位姿，`x,y,z,r,p,y` | `(6,) float32` |
| `rightRobotJoints` | 右臂关节角 | `(7,) float32` |
| `rightRobotGripperState` | 右夹爪状态，宽度和力 | `(2,) float32` |
| `externalCameraRGB` | 外部相机 RGB 数据 | `uint8 buffer` |
| `leftWristCameraRGB` | 左腕相机 RGB 数据 | `uint8 buffer` |
| `rightWristCameraRGB` | 右腕相机 RGB 数据 | `uint8 buffer` |
| `leftGripperCameraRGB1` | 左夹爪触觉相机 1 图像 | `uint8 buffer` |
| `leftGripperCameraMarkerOffset1` | 左夹爪触觉 marker 偏移 | `(63, 3) float32` |
| `rightGripperCameraRGB1` | 右夹爪触觉相机 1 图像 | `uint8 buffer` |
| `rightGripperCameraMarkerOffset1` | 右夹爪触觉 marker 偏移 | `(63, 3) float32` |

注意：图像发布时虽然使用 `sensor_msgs/Image`，但 `data` 字段里放的是 `cv2.imencode('.jpg', image)` 后的 JPEG 字节，不是未压缩的原始 BGR 数组。读取 pickle 后如需恢复图像，需要用 `cv2.imdecode()`。

示例读取：

```python
import pickle
import cv2
import numpy as np

with open('episode_0000.pkl', 'rb') as f:
    frames = pickle.load(f)

first = frames[0]
img_bytes = np.frombuffer(first.externalCameraRGB, dtype=np.uint8)
img = cv2.imdecode(img_bytes, cv2.IMREAD_COLOR)
print(first.timestamp, img.shape)
```

## 核心模块说明

### `camera_node_launcher.py`

负责统一启动采集端。

- 设置 OpenBLAS、MKL、OpenCV 等线程数，减少多进程采集时线程过量问题。
- 启动 `DeviceMappingServer`。
- 根据 `/get_mapping` 返回值创建多个发布器进程。
- 每个进程执行 `start_camera_publisher()`。
- `start_camera_publisher()` 会绑定 CPU 核并初始化 ROS 2 node。

### `publisher/realsense_camera_publisher.py`

`RealsenseCameraPublisher` 是 ROS 2 `Node`。

主要职责：

- 按序列号查找 RealSense 设备。
- 校验设备 product line 是否等于配置中的 `camera_type`。
- 开启 RGB stream。
- 启用 global time，并计算相机时间戳到 ROS 系统时间戳的 offset。
- 周期性获取 color frame，JPEG 编码后发布到：

```text
/{camera_name}/color/image_raw
```

### `publisher/tactile_sensor_publisher.py`

`TactileSensorPublisher` 是 ROS 2 `Node`。

主要职责：

- 通过 `VTSDeviceFinder` 根据序列号查找 ViTai/GF225 设备。
- 调用 `calibrate()` 完成初始化校准。
- 采集 warped image、marker 原始位置、marker offset。
- 对 marker 坐标按图像宽高做归一化。
- 发布 RGB 图像到：

```text
/{camera_name}/color/image_raw
```

- 发布 marker 信息到：

```text
/{camera_name}/marker_offset/information
```

marker 使用 `sensor_msgs/PointCloud2` 表示。`dimension=2` 时每个点 4 个 float：

```text
marker_location_x, marker_location_y, marker_offset_x, marker_offset_y
```

`dimension=3` 时每个点 6 个 float：

```text
marker_location_x, marker_location_y, marker_location_z,
marker_offset_x, marker_offset_y, marker_offset_z
```

### `teleoperation/data_recorder.py`

`DataRecorder` 是同步记录节点。

主要职责：

- 从 `DeviceMappingServer` 获取当前设备到 topic 的映射。
- 根据 `use_arm` 生成需要订阅的 ROS topic。
- 使用 `message_filters.ApproximateTimeSynchronizer` 进行近似时间同步。
- 收到同步帧后调用 `ROS2DataConverter.convert_all_data()` 转成 `Vitai_SensorMessage_Double`。
- 通过 FastAPI 控制是否记录 episode。
- `/start_episodes`：开始缓存同步帧。
- `/save_episodes`：停止记录并保存为新的 `episode_xxxx.pkl`。

### `common/ros_data_converter.py`

负责把 ROS 消息转换为训练/记录友好的 numpy 字段。

- `PoseStamped` 转成 6D pose：`x,y,z,r,p,y`。
- `JointState` 转成关节角或夹爪状态数组。
- `Image.data` 转成 `np.uint8` buffer。
- `PointCloud2.data` 转成 `float32` marker offset 数组。

### `common/data_models.py`

定义数据结构：

- `BimanualRobotStates`：双臂状态。
- `MoveGripperRequest`、`TargetTCPRequest`、`TargetJointsRequest`：控制请求模型。
- `Vitai_SensorMessage_Double`：单帧完整传感器数据。
- `SensorMessageList`：线程安全缓存，超过 `WARNING_THRESHOLD=2000` 会报警。
- `ControlMode`、`ImpedanceType`：机器人控制模式枚举。

### `common/space_utils.py`

提供位姿、旋转和平滑滤波工具：

- ROS `Pose`、6D pose、7D pose、4x4 齐次矩阵互转。
- 点云坐标变换。
- 9D rotation representation 互转。
- One Euro Filter 和低通旋转滤波器。

## 当前代码注意事项

以下是阅读代码时发现的风险点，运行前建议先修正：

1. `config/real_world_env.yaml` 中没有 `teleop_server` 配置段，但 `record_data.py` 会读取 `cfg.teleop_server.data_recorder_ip` 和 `cfg.teleop_server.data_recorder_port`。
2. `camera_node_launcher.py` 中 `CameraWorker` 判断 `camera_config.camera_type == 'realsense'`，但配置里的 RealSense `camera_type` 是 `D400`，会导致 RealSense 分支进不去。建议按独立字段区分发布器类型，或在启动 RealSense 时直接构造 `RealsenseCameraPublisher`。
3. 触觉配置字段是 `sensor_name`、`sensor_sn`、`marker_dimension`，但 `TactileSensorPublisher.__init__()` 参数是 `camera_name`、`camera_sn`、`dimension`。直接 `**camera_config` 会出现参数名不匹配。
4. `DeviceMappingServer.TactileSensorInfo.device_id` 类型标注为 `int`，但实际写入的是 `sensor_sn` 字符串。
5. `publisher/tactile_sensor_publisher.py` 中 `self.prev_time = time.time()()` 多了一次调用，会触发运行时错误，应为 `time.time()`。
6. `publisher/tactile_sensor_publisher.py` 的 `main()` 里实例化了未定义的 `UsbCameraPublisher`，该入口不能直接运行。
7. `teleoperation/data_recorder.py` 中 `check_sync()` 和 `check_timestamp()` 调用时没有传入 `self.timestamps`，打开 `self.time_check=True` 后会报错。
8. `DataRecorder.save()` 使用 `osp.dirname(self.save_dir)` 建目录，但 `self.save_dir` 本身被当作目录使用。如果传入的是目录路径，建议直接 `os.makedirs(self.save_dir, exist_ok=True)`。
9. `common/ros_data_converter.py` 右夹爪命令判断使用了 `if 'right_gripper_cmd' in topic_dict`，少了开头 `/`，与后续索引 `'/right_gripper_cmd'` 不一致。
10. 当前触觉 marker topic 在订阅工具中被注释，因此默认不会进入最终 pickle。

## 建议启动顺序

1. 确认 ROS 2 环境已经 source。
2. 确认 RealSense、ViTai/GF225 设备已连接，并能被对应 SDK 识别。
3. 修改 `config/real_world_env.yaml` 中的设备序列号、IP、CPU 核绑定和 `use_arm`。
4. 启动发布器：

```bash
python camera_node_launcher.py
```

5. 启动记录器：

```bash
python record_data.py --save_file_dir /path/to/save/pkl/
```

6. 开始 episode：

```bash
curl -X POST http://<data_recorder_ip>:<data_recorder_port>/start_episodes
```

7. 保存 episode：

```bash
curl -X POST http://<data_recorder_ip>:<data_recorder_port>/save_episodes
```

## 调试命令

查看设备映射：

```bash
curl http://127.0.0.1:8062/get_mapping
```

查看 ROS topic：

```bash
ros2 topic list
```

查看图像 topic 频率：

```bash
ros2 topic hz /external_camera/color/image_raw
```

查看 topic 类型：

```bash
ros2 topic info /external_camera/color/image_raw
```
