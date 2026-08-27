# 修改日志 (Modification Log)

> 追踪遥操系统相关修改,便于回溯。新修改直接追加到文件末尾。

---

## 2026-08-25 10:30 — J1 自转根因修复:恢复 J1 保护 + deadband + hold keepalive + grav comp 调参

**背景**:遥操时小臂 J1 自转需手扶,大臂 J1 跟转。诊断结论:根因是小臂(leader)侧 J1 重力补偿不足(grav_comp gain 被调低到 0.25),非 follower 映射模式(relative 与厂商 absolute+offset 数学等价)。同时发现 src/skye_robot_driver 的 J1 保护代码(8/24 17:06 build 有)在 8/24 17:51 被 revert,本次从 install 头文件 API + 设计意图重建并落地到 src。

### 1. FACTR 小臂 grav_comp 调参(leader 侧,重启 FACTR 后生效)

| 文件 | 改动 |
|---|---|
| `marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_left.yaml` | `gravity_comp.gain[0]`: 0.25 → **0.6**(恢复原值) |
| `marvin_ws/install/share/factr_teleop/configs/grav_comp_m6_right.yaml` | `gravity_comp.gain[0]`: 0.25 → **0.6**(恢复原值) |

原理:J1 是 base 承重最大关节,gain 0.25 补偿不足 → dynamixel J1 被重力拽转 → relative mapping 1:1 传给大臂。其余 gain 未动。

### 2. skye_robot_driver 恢复被 revert 的 J1 保护(已重新编译进 install)

依据:`install/.../include/.../driver_core.hpp`(8/24 17:04 头文件)的 API 声明 + `backups/skye_robot_20260825_J1params_install.yaml` 参数。

- **`driver_core.{hpp,cpp}`**:恢复 `max_abs_error` / `exceeds_max_error`(gate 工具函数)、`limit_delta` per-joint 重载(relative 路径分轴限速)、`suppress_j1_coupling`(J3 主导帧吸收 J1 耦合行程)
- **`driver_node.{hpp,cpp}`**:
  - `max_enable_error: 0.30` — enable gate:进遥操会话前要求 |leader-follower| < 0.30 rad(对齐厂商 readiness(),强制 FACTR 先 sync 再 teleop)
  - `max_following_error: 0.35` — following 熔断:|last_cmd-feedback| 超限 → hold 该臂 + 重置会话(对齐厂商 auto_disable)
  - `j1_coupling_deadband: 0.02` / `j1_coupling_j3_min: 0.05` — J1/J3 耦合抑制参数
  - `max_delta_per_joint: [0.01, 0.05, ...]` — relative 路径 J1 限速收紧 5 倍(abs/HITL 路径仍用标量 0.05)
  - 新增 `last_feedback_` 缓存(publish_state 250Hz 更新,同 callback group 无竞态)

### 3. 新增功能

- **J1 deadband**(`j1_deadband: 0.02`,新参数):`apply_relative_joint_mapping` 中小臂 J1 相对会话参考 ±0.02 rad 内的行程视为重力下坠/漂移,delta→0 不传给大臂。0 禁用。
- **Hold keepalive**(`hold_after_s: 0.05`,新参数):控制流静默超 50ms 后周期性重发 last_command 作为 hold 帧(参考厂商 `factr_apex_adapter.py:741` publish_hold_once)。不刷新 last_command_time,command_timeout(0.5s)仍正常触发。0 禁用。

### 4. 备份(rebuild 前的旧 install 产物)

- `backups/skye_robot_driver_20260825_J1protected_1706build.bin` — 8/24 17:06 build 二进制(sha256: d564ca40e65ce8...)
- `backups/skye_robot_20260825_J1params_install.yaml` — 带 J1 参数的旧 install yaml

### 5. 构建验证

- `colcon build --packages-select skye_robot_driver` 通过(2026-08-25 10:53)
- `test_driver_core` 4/4 通过
- 新 install 二进制符号验证:`suppress_j1_coupling` / `max_abs_error` / `exceeds_max_error` / per-joint `limit_delta` 全部存在

### 测试注意事项

1. 重启 FACTR docker(`run_marvin_m6_impedance.sh`)使 grav_comp 新参数生效;脚本只 patch joint_signs,不会覆盖 gain。
2. 重启 skye_robot_driver 使用新编译的 install 二进制。
3. **新行为提醒**:现在不先按 1(sync)直接按 2(teleop)会被 enable gate 拒绝(日志 "teleop session refused")。若小臂物理上无法完全同步到 0.30 rad 内,可调大 `max_enable_error` 或设 0 禁用。
4. 若 J1 仍缓慢漂移但 ≤0.02 rad 不再传给大臂(deadband 吸收);若漂移超过 0.02 rad 仍会传出——deadband 只吸收初始 ±0.02 rad,根治靠 grav comp 0.6。
5. 手感变化排查顺序:j1_deadband 0.02→0(禁用)→ max_delta_per_joint J1 0.01→0.05 → hold_after_s 0.05→0,逐项回退定位。

### 已知未做(后续 TODO)

- J1 角度 unwrap / 最短路径 Δ(leader J1 越过 ±π 时 delta 跳变 ~2π;当前 J1 限位 ±3.0 < π,实际风险低,暂不加)
- 状态机联动(非 TELEOP 状态自动停发指令;当前由 enable gate + command timeout 部分覆盖)
- 逐关节 following error(当前用 max,单关节超限即熔断整臂)

---

## 2026-08-25 16:xx — 遥操增量映射问题分析(只梳理不改码)

问题根因与方案梳理已单独成文:**`docs/问题分析_遥操增量映射_20260825.md`**。
要点:漂移直传 / J1 回绕(→大臂猛转,待加 unwrap)/ 大臂跟不上与 sync 重同步失败深挖 / max_delta_per_cycle 语义 / torque_feedback 语义 / velocity_ratio 30 未同步 install。
现场参数调整:left/right null_space → False;right barrier kp → 0.8036;J4 sync 参数增强(ruckig_kp 0.38, gc_gain 0.35)。

---

## 2026-08-25 18:13 — J1 回绕修复:relative 映射连续 unwrap

**背景**:leader J1/J3 报告角跨 ±π 时表示跳变 2π,`delta` 突变 → 大臂猛转(近一圈)。根因见 `docs/问题分析_遥操增量映射_20260825.md` P2。

**改动**(`src/skye_robot_driver`):
- `driver_core.{hpp,cpp}`:`apply_relative_joint_mapping` 增加逐帧连续 unwrap——帧间变化 >π 则 ±2π 修正,累计得到连续 leader 角;`clutch_saturated_joints` 改为重基线连续参考。
- `driver_node.{hpp,cpp}`:每臂状态 `leader_prev_ / leader_continuous_ / leader_cont_ref_` 替换原 `leader_ref_`,进入 teleop 时初始化,每帧消费后更新 prev。
- `test/test_driver_core.cpp`:适配新接口 + 新增 2 用例(`RelativeUnwrapHandlesPiCrossing`、`RelativeKeepsLegitimateLargeSwing`)。

**要点**:不能用最短路径 unwrap(合法大摆动 >π 会被折返);连续逐帧 unwrap 既吸收表示跳变,又不破坏合法大幅摆动。对无回绕的慢速采集行为完全不变。

**验证**:colcon build 通过;`test_driver_core` **6/6 通过**;install 二进制已更新(18:13)。

---

## 2026-08-26 — 遗留僵尸进程 + 小臂零点偏移问题定位

### 待优化项:僵尸进程清理
8/21 HITL 测试遗留进程未清理:
- `control_arbiter` × 65
- `episode_recorder` × 39

影响:占资源、污染 DDS 图(发现/同步被干扰)、可能与 driver 订阅冲突。
**后续优化**:启动脚本加进程互斥检查/清理(`pkill -f control_arbiter; pkill -f episode_recorder`),或一键启动脚本统一管理。

### 小臂零点偏移(根因确认)
- 现象:sync 报 SYNCED 但小臂/大臂绝对位姿对不上;右臂 J3≈+37.4°、J4≈-38.4°,左臂 J3≈+10.8°。
- 依据:J2(肩)≈0 但 J3/J4 非零 → 非重力下垂,是伺服零点与 URDF 0 不一致(标记位即 home,但 URDF 读非零)。
- 方向:伺服重新标零(标记位=URDF 0),或改 FACTR `calibration_joint_pos/initial_match_joint_pos` 为实际 home 角(需实测闭源行为)。
- 备注:仓库无标定脚本,需自写或用 dynamixel_wizard2。

> ⚠️ 更正(2026-08-26 16:xx):**不是零点偏移**。摆到标记位读数 ≈0(右 J3=-1.5°/J4=-1.9°),零点正确。此前 37°/-38° 是小臂真实物理下垂(重力补偿不足挂不住肘/腕)。右臂 J3/J4 gain 已调至 1.0,稍好但仍 J4 对不齐 → 追踪见 `docs/问题追踪_右臂J4对不齐_20260826.md`。
