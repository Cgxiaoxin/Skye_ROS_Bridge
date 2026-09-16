# `skye_factr_teleop` 阶段 0 源码与许可基线

日期：2026-09-16

范围：开发计划任务 0.1；本记录不创建或依赖 `skye_factr_teleop` 功能包。

## 1. 上游来源冻结

| 项目 | 冻结值 |
|---|---|
| 上游项目 | `FACTR_Teleop` |
| 上游仓库 URL | <https://github.com/JasonJZLiu/FACTR_Teleop.git> |
| 参考提交 | `7a07ab3629af03a91c0198df7c44a0082dca77ab` |
| 提交日期 | 2025-08-03 |
| 本地核验副本 | `/home/lyy/projects/FACTR_Teleop` |
| 上游许可证 | Apache License 2.0（SPDX：`Apache-2.0`） |

参考提交是后续实现唯一的上游代码基准；本地路径只用于核验，不是运行时或构建时依赖。
可用以下命令复核来源：

```bash
git -C /home/lyy/projects/FACTR_Teleop remote get-url origin
git -C /home/lyy/projects/FACTR_Teleop rev-parse HEAD
git -C /home/lyy/projects/FACTR_Teleop show \
  7a07ab3629af03a91c0198df7c44a0082dca77ab:LICENSE
```

在该提交中，仓库根 `LICENSE` 是 Apache License 2.0 全文，
`src/factr_teleop/package.xml` 和 `src/factr_teleop/setup.py` 均声明
`Apache-2.0`。上游没有 `NOTICE` 文件。计划使用的 FACTR Python 源文件带有
“Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li”及 Apache-2.0 许可头。

## 2. 本仓库与新包的许可证边界

- `leader_for_skye` 仓库根目录的 `LICENSE` 是 MIT License，版权声明为
  `Copyright (c) 2026 alex_cg`；它继续适用于未另行声明许可的仓库内容。
- 计划创建的 `skye_ros2_ws/src/skye_factr_teleop/` 是独立 ROS 包。该包采用
  Apache-2.0，并在包目录内放置自己的 `LICENSE`、第三方声明和包元数据。
- 包内从 FACTR 派生的文件继续遵守 Apache-2.0，保留上游版权和许可声明，并标明
  Skye/M6 修改。新写的包内源码也按 Apache-2.0 发布。
- 因此，仓库根 MIT 与新包 Apache-2.0 是按目录和文件声明区分的，不应把根目录
  MIT 误记为 FACTR 上游或新包源码的许可证。

## 3. 上游文件处置清单

### 3.1 计划迁移并修改

不存在按原路径原样复制的上游运行文件。以下文件作为派生实现的直接源码基线，迁移
时会改名、拆分或适配 Skye/M6：

| 上游文件（均相对于参考提交） | 计划目标/用途 | 必要修改 |
|---|---|---|
| `src/factr_teleop/factr_teleop/dynamixel/driver.py` | `skye_factr_teleop/dynamixel_driver.py` | 改用主机安装的 Python `dynamixel_sdk`；保留串口、同步读写和单位换算逻辑；适配 7 个 leader 关节和 1 个夹爪舵机。保留文件中对 GELLO driver 的来源说明。 |
| `src/factr_teleop/factr_teleop/factr_teleop.py` | 主要拆入 `control_core.py`，状态相关逻辑进入 `state_machine.py` | 移除 `python_utils`、工作区探测和非 Skye 路径；适配 M6 配置、Pinocchio/Ruckig、可测试的纯 Python 状态逻辑以及 Skye 控制边界。 |
| `src/factr_teleop/factr_teleop/factr_teleop_franka_zmq.py` | 仅将其抽象通信分层模式用于 `ros_driver.py` 的设计 | 不迁移 ZMQ 地址、Franka 消息内容或 `python_utils` 代码；通信实现改为 Skye/Gento ROS 2 接口。若形成派生代码，按上游派生文件处理。 |
| `src/factr_teleop/factr_teleop/configs/*.yaml` | 仅参考分组方式，生成 `config/grav_comp_m6_{left,right}.yaml` | 参数改为 M6 左右臂实测配置；不复制 Franka 数值或资源路径。若复制了受版权保护的表达，则按派生文件处理。 |
| `src/factr_teleop/package.xml`、`setup.py`、`setup.cfg`、`resource/factr_teleop` | 新包的 `ament_python` 元数据和资源布局参考 | 包名、入口、依赖、维护者和资源清单改为 `skye_factr_teleop`；不原样复制元数据。 |

新建的 `config.py`、`keyboard_control.py`、launch 文件和测试以项目需求实现；除非实际
实现时引入上游表达，否则不把它们标成 FACTR 派生文件。每个文件的最终归类以实际
内容为准，而不是仅以目标文件名判断。

### 3.2 明确不迁移

| 上游路径/内容 | 决定 |
|---|---|
| `src/factr_teleop/factr_teleop/dynamixel/python/**` | 不迁移内嵌 Dynamixel SDK、副本许可证和示例；使用主机安装的 `dynamixel_sdk` 依赖。 |
| `src/factr_teleop/factr_teleop/factr_teleop_franka_zmq.py` 中的 ZMQ/Franka 专用实现 | 不迁移；只有上一节所述通信分层模式可供参考。 |
| `src/factr_teleop/factr_teleop/factr_teleop_grav_comp_demo.py` | 不迁移独立演示节点。 |
| `src/factr_teleop/factr_teleop/urdf/**` | 不迁移 Franka URDF 和 mesh。Skye/M6 描述资源必须另行确认来源和再分发许可。 |
| `src/factr_teleop/factr_teleop/configs/franka_example.yaml`、`grav_comp_demo.yaml` 的具体参数 | 不迁移 Franka/演示参数。 |
| `src/bc/**`、`src/cameras/**`、`src/python_utils/**` | 不迁移这些上游包。 |
| `launch/**` | 不迁移数据采集、rollout、Franka 遥操和重力补偿演示启动文件。 |
| `assets/**`、上游 README 图片、`src/factr_teleop/resource/figure_*.png` | 不迁移论文和说明材料。 |
| 上游测试模板 `src/*/test/**` | 不迁移；为新包行为编写独立测试。 |

## 4. 源文件声明模板

从 FACTR 源文件迁移或形成派生代码时，保留上游版权，增加 Skye/M6 修改声明，并使用
以下文件头（年份或版权主体发生变化时据实更新）：

```python
# Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li
# Modifications Copyright (c) 2026 lyy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Modified for Skye/M6: <brief description of the material changes>.
```

完全新写且未包含上游表达的包内 Python 文件使用：

```python
# Copyright (c) 2026 lyy
# SPDX-License-Identifier: Apache-2.0
```

若文件保留了 `dynamixel/driver.py` 的相关实现，还必须保留其对 GELLO driver 的
“Based on”来源说明。若未来实际迁移其他第三方内容，必须先保留其原始声明并更新
第三方清单，不能仅套用上述模板。

## 5. 阶段 1 许可文件清单

阶段 1 创建功能包时必须同时创建并填写以下内容：

### `LICENSE`

- 放在 `skye_ros2_ws/src/skye_factr_teleop/LICENSE`；
- 内容为未经删改的 Apache License 2.0 全文；
- 不复制仓库根目录的 MIT License 作为新包许可证。

### `THIRD_PARTY_NOTICES.md`

- 标明 FACTR_Teleop 的项目名、仓库 URL、固定提交和 Apache-2.0；
- 保留 `Copyright (c) 2025 Jason Jingzhou Liu and Yulong Li`；
- 列出实际派生的上游文件及其新文件映射；
- 记录上游 `dynamixel/driver.py` 对 GELLO driver 的来源说明；
- 说明上游参考提交没有 `NOTICE` 文件；
- 只列入实际复制或派生的内容。主机依赖应按其分发要求记录，不能把“依赖”误写成
  “已复制源码”。

### `package.xml`

- 使用 package format 3；
- 包级许可字段必须是 `<license>Apache-2.0</license>`；
- 维护者、作者及其邮箱填写新包的真实责任主体，不沿用上游占位信息；
- 依赖项按实际使用声明，许可证字段不得写成根仓库的 `MIT`，也不得笼统写
  `Apache 2.0` 等非 SPDX 形式。

完成以上文件之前，不发布或分发新包产物。
