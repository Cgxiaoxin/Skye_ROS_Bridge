# Thor/Orin 统一代码 + `robot_profile` 设计

**日期:** 2026-09-03  
**状态:** 已与现场确认，待按 plan 落地  
**回退锚点:** `main` @ `d591505`（publish v1.0.2）；功能源分支 `robotiq_teleop`

## 问题

两台硬件差异（关节符号、夹爪协议）被拆成 `main`（Thor / DM4310）与 `robotiq_teleop`（Orin / Robotiq）双分支，修 sync/HITL 需两边 cherry-pick。希望 **一套代码**，用 launch 参数切换。

## 目标

- 在当前 `main` 上合并 `robotiq_teleop` 的夹爪抽象与 Orin 标定能力。
- 日常启动：`robot_profile:=thor`（默认）或 `robot_profile:=orin`。
- Thor 不传 profile 时行为与合并前一致（DM4310 + signs 全 `+1`）。
- Orin 显式 `robot_profile:=orin` 得到 Robotiq + 右臂 J6/J7=`-1`。

## 非目标

- 不改 FACTR 闭源 sync offset（继续用现有 7 轴 side topic workaround）。
- 不统一两台物理夹爪硬件。
- 不强制 hostname 自动探测（可选后续加；首版显式参数）。

## 架构

```text
skye_robot.yaml          # 中性/Thor 安全默认（dm4310, signs 全 +1）
profiles/thor.yaml       # Thor 覆盖（可与默认等价，显式文档化）
profiles/orin.yaml       # Orin 覆盖（robotiq + right J6/J7=-1 + mm 开度）

launch: parameters = [skye_robot.yaml, profiles/${robot_profile}.yaml]

marvin_ws/configs/{thor,orin}/grav_comp_m6_{left,right}.yaml
  → sync_marvin_overlay.sh 按 ROBOT_PROFILE 同步到 install
  → factr launch _grav_comp_config 优先读 overlay
```

| Profile | 大臂 signs | 夹爪 | 小臂 grav_comp |
|---------|------------|------|----------------|
| `thor`（默认） | 左右全 `+1` | `dm4310` / `dm4310` | `configs/thor/` |
| `orin` | 左全 `+1`，右 J6/J7=`-1` | `robotiq` / `robotiq` | `configs/orin/` |

## 合并原则

1. **代码能力**取自 `robotiq_teleop`（`GripperArmBackend`、Robotiq、`link_controller` 等）。
2. **默认参数**保持 Thor 安全；禁止把 Orin 默认写进 `skye_robot.yaml`。
3. **sync 7 轴**两边已有等价修复；冲突时两边都保留 publisher + remap。
4. 旧 `robotiq_dual_gripper:=true` 可映射为加载 `orin` 夹爪段，或 deprecate 并文档指向 `robot_profile:=orin`。

## 验收

- Thor：`robot_profile:=thor` 或不传 → 夹爪 dm4310、腕部同向、sync/teleop 正常。
- Orin：`robot_profile:=orin` → 夹爪 robotiq、右 J6/J7 同向、sync 右臂对右大臂。
- `git checkout d591505` 可回退合并前 Thor 行为。

## 风险

| 风险 | 缓解 |
|------|------|
| 默认 yaml 误留 Orin | Code review + Thor 冒烟必测 |
| sync 冲突解丢 | 验收 echo left/right_joint_states |
| grav_comp 拷错机 | profile 目录隔离 + sync 脚本打日志 |
