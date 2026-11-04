# Operator UI × 主机一体遥操集成设计

日期：2026-11-03  
状态：草案（待实现计划）  
范围：`teleop_record` 会话后端为主；前端小改以匹配；DAgger 本轮不动

## 1. 背景与目标

现场遥操已切换为仓库主机流程（`scripts/teleop.sh` + `skye_factr_teleop`），**不再**使用 Docker / `marvin_ws` 小臂。Operator UI 仍按旧 playbook 拉起 `start_skye_for_factr.sh` + `run_marvin_m6_impedance.sh`，并包含 `follower_align`。

目标：把 **遥操数采** 会话启动/退场接到一体 `teleop.sh`，去掉 Docker 小臂与 align，保留 sync 前「关左/关右」，并顺手做启动去重与退场清理。

非目标：

- 会话内自动 `build` / `configure` / `bind_leader_arms.py`
- UI 增加小臂 A/B 或启动时 `--side` 选项
- 本轮迁移 DAgger 到主机一体启动

权威操作参考：`skye_ros2_ws/src/skye_factr_teleop/大小臂通用启动指南.md`。

## 2. 产品决策（已锁定）

| 决策 | 选择 |
|---|---|
| 启动方式 | 一体 `teleop.sh start`（大臂+双小臂同一 launch） |
| 机台选择 | 仅 **thor / orin**；大臂与小臂配置按机台配对 |
| 小臂 device a/b | **不**在 UI 暴露；后端/脚本按 `--robot` 映射到原 A/B 配置目录 |
| follower align | **剔除**出 teleop_record 路径 |
| 关左/关右 | **保留**；实现改为主机进程，不再 docker exec |
| DAgger | **本轮不改** playbook |
| 编译 | 会话 **永不** 调用 `build`；缺 install 时预检失败并提示手动 build |

机台配对：

| UI profile | 大臂 | 小臂 YAML | 默认 domain | 控制器 IP |
|---|---|---|---|---|
| `thor` | `profiles/thor.yaml` | `skye_factr_teleop/config/grav_comp_m6_{left,right}.yaml` | 20 | 6.6.7.191 |
| `orin` | `profiles/orin.yaml` | `skye_factr_teleop/config/orin/grav_comp_m6_{left,right}.yaml` | 21 | 6.6.7.190 |

左右臂语义：启动默认双臂一体；运行期仍可用「关左/关右」在 sync 前裁剪一侧。不是「没有左右臂」。

## 3. 架构

```text
IDLE
  → 预检：teleop.sh check --robot <profile> --fix-latency
       （清残留后执行；含 ping / 串口 / latency / install 文件存在性）
  → STARTING
       1) teleop    : teleop.sh start --robot <profile> --skip-precheck
       2) recorder  : 现有 skye_data_recorder launch
  → READY
       health：/gento/joint_states 新鲜 且 /teleop/state 有值（含 IDLE）
  → 操作：switch_sync → switch_teleop → 录制（无 align）
  → 结束：switch_stop → hold_current（最佳努力）
       → 停进程组 → 主机 Dynamixel 去使能
       → 不 docker rm（teleop_record）
```

健康与降级：

- 步骤 `teleop` 就绪：joint_states 新鲜 **且** `_teleop_state is not None`。
- `ros_bridge.health`：新增（或将原 `marvin` 重命名为）`teleop` 键，语义为「`/teleop/state` 已收到」；就绪判定由 supervisor/playbook 组合 joint_states + teleop（实现上可在 `health("teleop")` 内同时要求 joint 新鲜，避免双键竞态）。快照 `health` 列表对外用 `teleop`，不再对 teleop_record 展示 `marvin`。
- 运行期 DEGRADED：可继续以 joint_states 新鲜度监控大臂（一体进程内大臂掉线）。
- `step_timeout_s`（默认 120）仅作上限；正常应远早于超时进入 READY。

## 4. Playbook 与预检

### 4.1 `playbooks.py`（`teleop_record`）

替换 `driver → marvin → align → recorder` 为：

| step id | argv | health_key |
|---|---|---|
| `teleop` | `bash <repo>/scripts/teleop.sh start --robot <profile> --skip-precheck` | `teleop`（见 §3） |
| `recorder` | 现有 recorder launch | `recorder` |

环境：

- 设置 `ROBOT_PROFILE`、`ROS_DOMAIN_ID`（thor=20 / orin=21）。
- **不**强制 `FASTRTPS_DEFAULT_PROFILES_FILE=marvin_ws/...`；与 `teleop.py` 一致：路径无效则 unset。
- 删除对本模式有效的 `marvin_start_cmd` / `marvin_launch_cmd` 依赖（可保留 yaml 键但 teleop_record 忽略）。

`dagger`：`playbook_for` 分支保持现有 Docker 步骤不变。

### 4.2 `supervisor` 预检（`teleop_record`）

1. 按 whitelist 清残留：`skye_robot_driver`、`skye_factr_teleop`（及旧 align/recorder 模式如仍配置）。`cleanup_stale_commands` 对 **teleop_record** 不含 `docker rm`；若全局 yaml 仍保留 docker 清理项，仅在 `mode == dagger` 时执行，避免误伤本模式同时不破坏 DAgger。
2. 执行 `teleop.sh check --robot <profile> --fix-latency`；非 0 则预检失败，向用户展示 stderr。
3. **不再**要求 `marvin_ws/fastrtps_no_shm.xml` 存在（teleop_record）。DAgger 预检同样改为检查驱动包内 `skye_robot_driver/config/fastrtps_teleop.xml`。
4. 避免与 check 重复的二次 ping；teleop_record 以 `teleop.sh check` 为单一预检真相。

### 4.3 `teleop.py` 配套

| 改动 | 行为 |
|---|---|
| robot→配置 | `--robot thor\|orin` 默认选对应配置目录；UI 不传 `--device`。CLI 可暂留 `--device` 作高级覆盖。 |
| `--skip-precheck` | 跳过 ping / 串口 / latency / install 全量复查；仍拒绝已有 driver，防止双开。 |
| install 检查 | 去掉 `ros2 pkg prefix` 两次调用；只检查 `install/<pkg>/lib/<pkg>/<exe>` 是否存在。 |
| latency | `check --fix-latency` 与（未 skip 时的）`start` 行为保持可写回 1。 |

会话 **不** 调用 `build`。

## 5. 关臂（主机化）

保留 API：`leader_left|right_off|on`、门禁与锁定语义（见既有 leader-arm-gate 设计）。

重写 `scripts/leader_arm_gate.sh`（或等价 Python）：

**off**

1. 主机 pkill 该侧节点（`skye_factr_teleop_left|right` 等实际节点名，不以旧 Docker `factr_teleop_*` 为准）。
2. 短暂等待串口释放。
3. 主机 `disable_leader_dynamixel.py --side <side>`；端口从当前 profile 小臂 yaml 的 `dynamixel.dynamixel_port` 解析（可设 env 后调用），不依赖 Marvin 镜像。

**on**

1. 节点已在则成功返回。
2. 否则主机 launch：`start_teleop_m6_<side>.launch.py` + `robot_profile:=<profile>`（domain/env 与会话一致）。
3. 等待节点出现或短超时失败。

失败时 API 报错，UI 不翻转开/关状态（现行为）。

## 6. 退场

`teleop_record` 结束顺序：

1. `switch_stop`（现有 `on_before_stop`）。
2. 最佳努力 `hold_current`（失败只记日志）。
3. 倒序终止 playbook 进程组。
4. **不**执行 `docker rm`（本模式）。
5. `disable_leader: true` 时默认 **主机** 去使能；`disable_via_docker` 默认 `false`（可保留代码路径作显式 fallback）。

DAgger 若仍用容器，可继续其 docker teardown；本轮不强制统一。

`default.yaml`：`cleanup_stale_commands` / `teardown` 按上调整；文档同步。

## 7. 前端与门禁（必改小集）

| 位置 | 改动 |
|---|---|
| `web/src/lib/steps.js` | playbook ids：`teleop`, `recorder`；去掉 marvin/align；步骤文案改为大小臂 Teleop |
| `TeleopPanel.svelte` | 去掉对齐按钮与对齐状态展示 |
| `commands.js` + `commands.py` | `switch_teleop` 仅要求 `SYNCED`；删除 align 门禁依赖 |
| `hints.py` | 同步完成后提示直接开遥操 |
| `Operator_UI使用说明.md` | 主机 `teleop.sh` 流程；去掉「必须 Docker 小臂」限制描述 |

顶栏机台选择保持 thor/orin，不增加 device。

## 8. 错误处理

| 场景 | 行为 |
|---|---|
| check 失败 | 预检失败 → FAILED；展示脚本输出 |
| start 后超时无就绪 | teleop 步失败；可 retry |
| 缺 install | 明确提示手动 `bash scripts/teleop.sh build` |
| 关臂失败 | 不翻转 UI 状态 |
| 去使能失败 | warning，仍回 IDLE |
| FTDI 更换 | 运维手动 `ROBOT_PROFILE=... python3 scripts/bind_leader_arms.py`；不进会话 |

## 9. 测试

- 单测 playbook argv / 步数；teleop_record 预检不依赖 marvin xml。
- teardown：teleop 路径无 docker rm；主机 disable 被调用（可 mock）。
- 关臂：断言主机脚本 argv，不再要求 `MARVIN_CONTAINER_NAME` 成功路径。
- 门禁：无 align 时仅 SYNCED 可 teleop；双关臂挡 sync。
- 前端 steps/commands 与文案冒烟。
- 真机清单：Thor/Orin 各 启动 →（可选关一侧）→ sync → teleop → 短录 → 结束；无 SDK 双客户端残留。

## 10. 文件触点（实现时）

后端 / 脚本：

- `skye_operator_ui/.../playbooks.py`, `supervisor.py`, `ros_bridge.py`, `teardown.py`, `commands.py`, `hints.py`, `config/default.yaml`
- `scripts/teleop.py`（`--skip-precheck`、install 检查、robot→config）
- `scripts/leader_arm_gate.sh`（及必要时 `disable_leader_dynamixel.py` 读 yaml 端口）

前端 / 文档：

- `web/src/lib/steps.js`, `commands.js`, `TeleopPanel.svelte`
- `docs/Operator_UI使用说明.md`

测试：对应 `skye_operator_ui/test/*`、teleop 脚本单测（若有）。

## 11. 启动效率（本轮纳入）

- 预检与 start 去重（`--skip-precheck`）。
- 去掉 `ros2 pkg prefix` 双调用。
- 就绪靠 topic，不空等到满超时。
- 退场去掉无意义 docker rm。

不把 `build` 放进会话；相对「每次编译」，日常就绪从分钟级回到秒～几十秒（硬件连接主导）。
