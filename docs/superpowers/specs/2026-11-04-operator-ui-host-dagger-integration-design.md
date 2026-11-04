# Operator UI × 主机 HITL DAgger 集成设计

日期：2026-11-04  
状态：已批准（方案 A）  
分支：`dev-Dagger`  
范围：仅 `UiMode.dagger` 启动/预检/退场；不改 arbiter 状态机；不改 teleop_record

## 1. 目标

把 DAgger 会话从「Docker Marvin HITL」改成与 Phase 1 同构路径一致：

```text
driver  → start_skye_for_factr.sh <profile>
leaders → ros2 launch skye_factr_teleop start_teleop_m6_dual_hitl.launch.py
arbiter → start_hitl_host.sh --arbiter-only
dummy   → pub_dummy_policy_chunk（optional）
```

## 2. 决策

| 项 | 选择 |
|----|------|
| Playbook | 分步 A（非 `teleop.sh --hitl`） |
| 小臂 step id | `leaders`（替换 `marvin`） |
| health_key | `teleop`（`/teleop/state` 已收到；保留 `marvin` 别名兼容） |
| 退场默认 | 与 teleop 相同：不 `docker rm`；主机 Dynamixel 去使能 |
| 预检 | 仍 ping + FastDDS 文件 + driver 残留清理；**默认不** docker rm |
| 覆盖 | 继续支持 `playbook_override.dagger`；去掉对 `marvin_start_cmd` 的依赖 |

## 3. 非目标

- Phase 2/3 真机验收
- 改 Hint 启停全文（可在 Operator_UI 使用说明先改；Hint 可链到 dagger_dev）
- ZMQ gateway / 真 VLA

## 4. 验收

- 单测：playbook / teardown / supervisor 预检
- 前端 steps 显示「HITL 小臂」且 playbook id 为 leaders
- 文档不再把 Docker 写成 DAgger 默认路径
