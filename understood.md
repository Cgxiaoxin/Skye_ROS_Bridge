# 项目架构理解

## 项目目录结构

```text
leader_for_skye/
├─ skye_ros2_ws/                 # 当前 Skye ROS 2 主工作区
│  ├─ src/
│  │  ├─ skye_robot_driver/      # C++ 机器人驱动：机械臂、夹爪、Modbus/CAN、配置、服务
│  │  ├─ skye_hitl_dagger/       # Python HITL / DAgger：控制仲裁、策略动作块、键盘遥操、录制
│  │  ├─ skye_follower_align/    # 从臂对齐与主机键盘辅助
│  │  └─ skye_data_recorder/     # 数据采集与 episode 路径管理
│  └─ scripts/                   # 构建、SDK ABI、接口、烟测与安全校验
│
├─ marvin_ws/                    # Marvin/M6 遥操兼容与历史工作区
│  ├─ gento_ros2_ws/             # Gento 机器人 ROS 2 驱动（C++）
│  ├─ tele_operation/            # 遥操、相机/触觉发布、设备映射、数据录制
│  ├─ gripper_robotiq/           # Robotiq 夹爪 SDK 与控制
│  ├─ gripper_changing/          # ChangingTek 夹爪适配与测试
│  ├─ configs/{thor,orin}/       # 不同机器人配置
│  ├─ launch_overlay/            # 双臂 Gento 遥操启动覆盖层
│  └─ urdf_classification/       # 大臂/小臂 URDF 分类目录
│
├─ third_party/gento_sdk/        # 预编译 Gento SDK
│  ├─ include/                   # 控制、运动学、运动规划、文件客户端等头文件
│  └─ lib/{x86_64,aarch64}/      # 不同架构库文件
│
├─ scripts/                      # 项目级运行、同步、调试、夹爪与部署脚本
├─ docs/                         # 操作手册、接口资料、开发计划与设计/实施文档
├─ .superpowers/sdd/             # 既有任务报告
├─ README.md / LICENSE / .gitlab-ci.yml
└─ 遥操高频优化路线.md            # 顶层路线文档
```

项目可概括为：以 `skye_ros2_ws` 为现行主线的 ROS 2 机器人控制系统，覆盖底层机器人/夹爪驱动、HITL 人机协作数据采集、主从臂对齐与数据录制；`marvin_ws` 承载 M6/Gento 双臂遥操、外设及兼容实现；`third_party/gento_sdk` 提供底层厂商 SDK 依赖。

## 用 ROS 2 概念理解架构

- **工作区（workspace）**：一组能一起编译、运行的 ROS 2 软件包。这里主要是 `skye_ros2_ws`。
- **软件包（package）**：一个独立功能模块，类似一个小应用。一个工作区可有多个软件包。
- **节点（node）**：实际运行的程序，例如机器人驱动节点、键盘控制节点。
- **话题（topic）**：节点之间持续发布/订阅的数据通道，例如关节状态、控制指令。
- **服务（service）**：一次请求、一次响应的接口，适合切换模式、设置运动速度。
- **启动文件（launch）**：一次启动多个节点及其配置的编排脚本。
- **消息（msg）**：话题上传输的数据结构。

当前主工作区的功能关系可理解为：

```text
操作者 / 策略程序
       │
       │ 控制指令、键盘输入、策略动作
       ▼
skye_hitl_dagger                 # 人机协作控制层
  ├─ hitl_keyboard_node          # 键盘遥操节点
  ├─ control_arbiter_node        # 控制仲裁：决定当前采用谁的指令
  ├─ chunk_player                # 分段播放策略动作
  └─ episode_recorder_node       # 记录一次操作过程（episode）
       │
       │ 最终控制指令 / 模式切换
       ▼
skye_robot_driver                # 硬件驱动层
  ├─ driver_node                 # ROS 2 与真实机器人之间的桥梁
  ├─ 机械臂控制、运动模式、状态读取
  ├─ DM / Robotiq 等夹爪适配
  └─ SetMode、SetMotionRates 服务
       │
       │ SDK 调用
       ▼
third_party/gento_sdk            # 厂商提供的底层控制库
       │
       ▼
真实机械臂、夹爪
```

配套软件包：

- `skye_follower_align`：主从臂对齐辅助，帮助 follower（从臂）与 leader（主臂）进入可遥操的对齐状态。
- `skye_data_recorder`：记录 ROS 2 中的状态、动作或实验过程。

## ROS 2 目录含义

| 目录或文件 | ROS 2 中的作用 |
|---|---|
| `skye_ros2_ws/src` | 工作区中所有 ROS 2 软件包的源码位置 |
| `package.xml` | 软件包描述：名称、依赖、版本等 |
| `CMakeLists.txt` | C++ 软件包的构建规则 |
| `setup.py` | Python 软件包的安装与入口规则 |
| `launch/` | 启动文件：一条命令拉起一组节点 |
| `msg/` | 自定义话题消息，如控制模式、策略动作块 |
| `srv/` | 自定义服务，如模式和运动速度设置 |
| `config/*.yaml` | 节点启动时读取的参数，例如机器人型号、夹爪配置 |
| `test/` | 对节点逻辑、控制仲裁和驱动核心的测试 |
| `scripts/` | 构建、部署、接口检查、硬件验证等辅助脚本 |

`marvin_ws` 可先理解为另一套偏 M6/Gento 的遥操工作区/兼容层。它包含类似的 ROS 2 驱动、遥操、数据录制及夹爪控制能力；`skye_ros2_ws` 是当前项目针对 Skye 结构重新组织后的主线。

推荐的理解顺序：

1. `skye_robot_driver`：机器人如何成为 ROS 2 系统里的可控制设备。
2. `skye_hitl_dagger`：控制指令来自键盘还是策略，以及如何仲裁。
3. `skye_follower_align`：遥操前如何对齐主从臂。
4. `skye_data_recorder`：如何记录训练或操作过程的数据。
5. `launch/` 与 `config/`：实际如何把整套系统启动起来。

## 日常启动：`skye_ros2_ws` 会用到的文件（2026-09-10）

| 场景 | 文件 |
|---|---|
| 主机大臂驱动 | `src/skye_robot_driver/launch/skye_robot_driver.launch.py`、`src/skye_robot_driver/config/skye_robot.yaml`、`src/skye_robot_driver/config/profiles/thor.yaml` 或 `profiles/orin.yaml` |
| 首次/缺少 `install/setup.bash` 时编译驱动 | `scripts/build.sh`、`src/skye_robot_driver/CMakeLists.txt` 及其列出的驱动 C++ 源码/头文件、`SetMode.srv`、`SetMotionRates.srv` |
| `s` 对齐大臂与小臂时 | `src/skye_follower_align/launch/follower_align.launch.py`、`skye_follower_align/follower_align_node.py`、`host_keyboard.py`、`align_logic.py`、`align_keys.py`、`setup.py` |

条件使用、非标准日常路径的配置：

- `config/skye_robot_robotiq_dual.yaml`、`config/skye_robot_robotiq_right.yaml`：仅 Thor 使用遗留 Robotiq launch 参数时加载；正常 Orin 应使用 `robot_profile:=orin`。
- `skye_hitl_dagger/`：文档明确说明「日常纯遥操不要启 HITL launch」。
- `skye_data_recorder/` 与各 `verify_*.sh`：不是日常启动链路。

另发现一处值得核对的配置不一致：文档表格称 Thor 大臂 signs「全 +1」，但实际 `src/skye_robot_driver/config/profiles/thor.yaml` 中右臂 J6/J7 是 `-1`。

## 日常启动：`marvin_ws` 会用到的文件（2026-09-10）

`marvin_ws` 中，日常纯遥操实际会用到的是：

| 文件 | 用途 |
|---|---|
| `fastrtps_no_shm.xml` | 主机与 Docker 内都在启动前设置；禁用 FastDDS SHM、走 UDPv4。 |
| `launch_overlay/start_teleop_m6_dual_gento.launch.py` | Docker 内日常启动的 FACTR 双臂遥操 launch，重映射到 `/gento/*` topic。 |
| `configs/thor/grav_comp_m6_left.yaml` | Thor 左小臂重力补偿/串口及关节配置。 |
| `configs/thor/grav_comp_m6_right.yaml` | Thor 右小臂配置。 |
| `configs/orin/grav_comp_m6_left.yaml` | Orin 左小臂配置。 |
| `configs/orin/grav_comp_m6_right.yaml` | Orin 右小臂配置。 |

选择规则：`ROBOT_PROFILE=thor` 使用 `configs/thor/` 的两份 YAML；`ROBOT_PROFILE=orin` 使用 `configs/orin/`。launch 会优先从这些 profile 目录加载。

运行时还会使用 `marvin_ws/install/` 下由 bootstrap/sync 生成的 ROS 安装产物（如 `install/setup.bash` 与 `factr_teleop` 包内容）；当前目录浏览结果中它尚未存在，因此不是仓库内现成文件。

明确不属于“日常纯遥操”：

- `launch_overlay/start_teleop_m6_dual_gento_hitl.launch.py`：仅 HITL。
- `start_gento_dual_arm_sync.sh`：旧版一键脚本，仅参考。
- `gento_ros2_ws/` 的 `gento_robot_driver`：日常流程明确要求不要与 `skye_robot_driver` 同时启动。
- `tele_operation/`、录制/回放/可视化、URDF、夹爪测试与开发文档：不在该启动链路。

### 补充理解：bootstrap 与 sync

可以把 `marvin_ws` 想成一套“机器人零件和说明书”。

- `launch_overlay/` 和 `configs/`：是手上正在编辑的最新版说明书、参数表。
- `install/`：是系统整理好、可以直接给 ROS 使用的一套“装配完成版”。

`bootstrap` 的意思是“第一次准备”：如果 `marvin_ws/install/` 不存在，启动脚本会下载或解压一份基础的 ROS 程序到这里。其中会有 `install/setup.bash`，相当于告诉终端 ROS 程序放在哪里。

`sync` 的意思是“把最新版小文件覆盖进去”：启动前，脚本会把仓库里的 `launch_overlay/start_teleop_m6_dual_gento.launch.py` 及 `configs/thor/` 或 `configs/orin/` 中的 YAML，复制到 `install/share/factr_teleop/`。因此，即使 ROS 从 `install/` 启动，也会使用仓库中最新的启动方式和当前机器的参数。

日常顺序可理解为：

```text
启动脚本
  ↓
install 不存在？先 bootstrap 准备基础程序
  ↓
sync：复制最新 launch 和配置
  ↓
source install/setup.bash
  ↓
启动小臂遥操
```

直接运行 `launch_overlay/` 内的 launch 也可以；文档推荐这种方式，因为它直接使用当前文件，不依赖已经复制到 `install/` 的版本。

### 补充理解：启动脚本怎样完成 sync

运行 `./scripts/run_marvin_m6_impedance.sh` 后，脚本会自动调用 `scripts/sync_marvin_overlay.sh`。可以把它理解成“把桌上刚改好的作业放进书包”：`launch_overlay/` 和 `configs/` 是桌上的最新版，`install/` 是机器人启动时会读取的书包。

它的动作是：

```text
1. 检查 install/ 在不在
   不在 → bootstrap：先准备好书包和基础程序

2. 选择机器型号
   ROBOT_PROFILE=thor → 选 configs/thor/
   ROBOT_PROFILE=orin → 选 configs/orin/
   没写 → 默认 thor

3. sync：把桌上的新版文件复制进书包
   launch_overlay/*.launch.py
      → install/share/factr_teleop/launch/

   configs/<型号>/grav_comp_m6_*.yaml
      → install/share/factr_teleop/configs/
```

实际复制使用 `cp -f`：书包中已有同名旧文件时，用桌上的新版文件替换它。
