# 开发计划与 marvin 对齐

当前主线：`skye_ros2_ws/src/skye_robot_driver`（C++ + `libGentoSDK.so`）。  
参考实现：`marvin_ws/gento_ros2_ws/src/gento_robot_driver`（Position）；本驱动改为 **PD**。  
夹爪后续补齐，本计划不挡臂控闭环。

---

## 开发顺序

### P0 — 先能编过
- [ ] `cd skye_ros2_ws && source /opt/ros/humble/setup.bash`
- [ ] `colcon build --packages-select skye_robot_driver`
- [ ] 修好 CMake / include / `CMPL_LIN` / RPATH（`third_party/gento_sdk/lib/x86_64`）
- [ ] `./scripts/check_sdk_abi.sh`（或手工 `file` / `ldd` / `nm -D`）

### P1 — 最小可跑闭环（v0.1）
- [ ] 启动：Link →（Error 则 ResetError）→ Idle → **PD**
- [ ] 定时 `GetRT` → 发 `/gento/joint_states`（14 轴，**rad**）
- [ ] 订 `/gento/left_joint_control`、`/gento/right_joint_control` → `SetJointPosPDCmd`
- [ ] `/gento/emergency_stop`（或沿用 `/gento/stop_motion`）+ 析构 `Unlink`
- [ ] QoS：控制流 `KeepLast(1)` + `BEST_EFFORT`
- [ ] 逻辑迁移自 `gento_robot_driver`，控制模式 Position → PD

### P2 — 真机冒烟
- [ ] 低速、小幅、**单臂**先测
- [ ] 确认 `/gento/joint_states` 与实机动作一致
- [ ] 再开双臂；`vel_ratio` 先 10（与旧 yaml 一致）

### P3 — 安全层
- [ ] 关节限位（rad）
- [ ] `max_delta_per_cycle`（建议 0.05 rad）
- [ ] `vel_ratio` / `acc_ratio`
- [ ] `SetPDCmdCycleTime`（与控制频率匹配，如 250 Hz → 4 ms）
- [ ] `command_timeout` → hold（停流后保持当前姿）

### P4 — 主手对接
- [ ] 确认 FACTR 仍 remap 到 `/gento/*`（见下表）→ **驱动对齐后主手可少改或不改**
- [ ] 若主手 topic/单位不同，再加薄 `teleop_bridge`；否则直接订

### P5 — 夹爪（后续）
- [ ] 对齐 `/left|right_teleop_gripper/ctrl` 与 state；不进 v0.1

---

## 与 marvin_ws 必须对齐的信息

本地现状：**小臂（FACTR 主手）+ 大臂（Gento 从臂）双臂**，同机 / 同 `ROS_DOMAIN_ID`（脚本常用 `20`）。  
活跃链路（bridge-less）：

```text
factr_teleop_{left,right}
  ← /gento/joint_states          # 14-DOF 反馈，sync 用
  → /gento/{left,right}_joint_control
gento_robot_driver / skye_robot_driver   # 唯一 SDK 客户端
  IP 6.6.7.190
```

启动参考：`marvin_ws/start_gento_dual_arm_sync.sh`  
参数参考：`marvin_ws/gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml`

### 1. Topic / Service（对外契约优先兼容 `/gento/*`）

| 方向 | Topic / Service | 类型 | 约定 |
|------|-----------------|------|------|
| 反馈 | `/gento/joint_states` | `sensor_msgs/JointState` | **14** position(+velocity)，rad |
| 指令 | `/gento/left_joint_control` | `JointState` | **7** position，rad；可忽略 `name` |
| 指令 | `/gento/right_joint_control` | `JointState` | 同上 |
| 服务 | `/gento/hold_current` | `std_srvs/Trigger` | 保持当前反馈位，仍可接新指令 |
| 服务 | `/gento/stop_motion` | `std_srvs/Trigger` | 停轨/切 Idle，拒绝指令直到 hold |
| 建议 | `/gento/emergency_stop` | `Trigger` | v0.1 软件急停（可与 stop 合并实现） |

节点内部可用 `/joint_states`、`/left_joint_control`…，**launch remap 到 `/gento/*`**，与旧 FACTR 脚本兼容。

### 2. 关节名 / 顺序 / 单位

| 项 | 值 |
|----|-----|
| 名字 | `l_j1..l_j7, r_j1..r_j7` |
| 顺序 | 左 `[0..6]`，右 `[7..13]` |
| ROS 单位 | **一律 rad / rad·s⁻¹** |
| SDK 边界 | 仅在 `driver_core` 内 °↔rad |
| 左臂对象 | `FX_OBJ_ARM0` |
| 右臂对象 | `FX_OBJ_ARM1` |

FACTR：`follower_joint_offset` 左=0、右=7（`grav_comp_m6_{left,right}.yaml`）。

### 3. 语义映射（驱动侧，不是舵机接线符号）

两边臂当前一致（来自 `gento_robot.yaml`）：

```text
joint_order:   [0,1,2,3,4,5,6]
joint_signs:   [1,1,1,-1,1,-1,-1]
joint_offsets: [0,0,0,0,0,0,0]
mapped[i] = leader[order[i]] * signs[i] + offsets[i]
```

**不要**照搬 FACTR Dynamixel 的 `joint_signs`（那是小臂舵机方向）。

### 4. 安全默认（先抄后调）

```text
limits_min: [-3.1067,-2.01,-3.1067,-1.0472,-3.1067,-1.0472,-1.5708]
limits_max: [ 3.1067, 2.01, 3.1067, 2.53,  3.1067, 1.0472, 1.5708]
max_delta_per_cycle: 0.05   # rad / 周期
command_timeout_s:   0.20   # 超时 → hold
left/right_velocity_ratio: 10
```

超限：**拒绝下发**（旧驱动行为），不要静默夹紧后当成功。

### 5. 主手模式（FACTR）

| 键 | 行为 |
|----|------|
| `1` sync | 读大臂 `/gento/joint_states`，小臂跟到同步 |
| `2` teleop | 发 `/gento/{left,right}_joint_control` |
| `3` stop | 停主手输出 |

### 6. 夹爪（后续对齐，不进 v0.1）

| Topic | 说明 |
|-------|------|
| `/left_teleop_gripper/ctrl`、`/right_teleop_gripper/ctrl` | 主手夹爪指令 |
| `/left_gripper/state`、`/right_gripper/state` | 夹爪状态 |
| 录数遗留名 | `/left_gripper_state` 等（`tele_operation/` 偏录数，非控制主路径） |

### 7. 与旧驱动差异（本仓库主动改）

| 项 | marvin `gento_robot_driver` | 新 `skye_robot_driver` |
|----|----------------------------|-------------------------|
| 模式 | Position + `SetJointPosCmd` | **PD** + `SetJointPosPDCmd` |
| QoS | 默认 Reliable，depth 10 | 控制流 **KeepLast(1)+BEST_EFFORT** |
| SDK 路径 | 外链绝对路径 | `third_party/gento_sdk` |
| 工作区 | `marvin_ws/gento_ros2_ws` | `skye_ros2_ws` |

对外 topic 契约尽量不变 → 小臂 FACTR **可少改**。

### 8. 关键参考文件

- `marvin_ws/start_gento_dual_arm_sync.sh`
- `marvin_ws/gento_ros2_ws/src/gento_robot_driver/config/gento_robot.yaml`
- `marvin_ws/gento_ros2_ws/src/gento_robot_driver/src/{driver_core,gento_robot_driver_node}.cpp`
- `marvin_ws/docs/superpowers/specs/2026-07-24-gento-factr-teleop-bridge-less-design.md`
- `marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_{left,right}.yaml`

---

## 文档保留说明

| 文件 | 用途 |
|------|------|
| `docs/dev_plan.md` | 本文件：开发顺序 + 对齐清单 |
| `docs/ros_interfaces.md` | 对外接口速查 |
| `docs/teleop_sop.md` | 现场启停 / 急停 |
| 根目录 `遥操高频优化路线.md` | QoS / 高频优化（Orin 侧参考，勿重复开文档） |
| `third_party/gento_sdk/README.md` | SDK vendor / 换架构 so |
