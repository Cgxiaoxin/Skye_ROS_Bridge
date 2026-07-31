# Marvin M6 机械臂 IP 更新 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将所有机械臂启动与采集入口的默认 IP 从 `192.168.1.190` 更新为 `6.6.7.190`，并验证配置和网络状态。

**Architecture:** 机械臂驱动从 ROS 2 launch 参数 `ip` 接收地址，该参数默认读取 `ROBOT_IP`。Docker 脚本也向容器注入 `ROBOT_IP`，采集配置独立保存 `robot_server.robot_ip`。统一替换这些默认值和对应文档，不改变显式参数覆盖优先级。

**Tech Stack:** Bash、Python ROS 2 launch、YAML、Markdown、`rg`、`ping`。

## Global Constraints

- 将机械臂 IP 精确设为 `6.6.7.190`。
- 保持上位机 `192.168.1.165`、端口、USB 和夹爪配置不变。
- `ROBOT_IP` 和 `ip:=...` 的显式值必须继续覆盖默认值。
- 当前工作区没有 Git 元数据；不得执行 Git 提交命令。

---

### Task 1: 更新默认连接目标与使用文档

**Files:**
- Modify: `run_marvin_m6_impedance.sh:6`
- Modify: `install/share/robot_driver/launch/robot_servo_start_marvin.launch.py:23`
- Modify: `install/share/robot_driver/launch/robot_servo_start_marvin.launch_pos.py:24`
- Modify: `tele_operation/config/real_world_env.yaml:5`
- Modify: `双臂遥操系统使用手册 (Dual-Arm Teleop User Manual).md:19-30`
- Modify: `doc/docker大臂.txt:12-24`
- Modify: `tele_operation/README.md:80`

**Interfaces:**
- Consumes: `ROBOT_IP` environment variable and ROS 2 `ip` launch argument.
- Produces: default IP `6.6.7.190` whenever neither interface supplies an explicit address.

- [ ] **Step 1: Record the expected configuration assertions**

Run:

```bash
rg -n --glob '!outputs/**' --glob '!tele_operation/outputs/**' --glob '!*.log' '192\\.168\\.1\\.190' .
```

Expected: the listed runtime, configuration, and documentation references contain the old IP before the change.

- [ ] **Step 2: Apply the minimal configuration and documentation replacement**

Replace each exact `192.168.1.190` occurrence in the listed files with `6.6.7.190`. Do not modify `192.168.1.165` or unrelated fields.

- [ ] **Step 3: Verify static configuration correctness**

Run:

```bash
bash -n run_marvin_m6_impedance.sh
python3 -m py_compile install/share/robot_driver/launch/robot_servo_start_marvin.launch.py install/share/robot_driver/launch/robot_servo_start_marvin.launch_pos.py
rg -n --glob '!outputs/**' --glob '!tele_operation/outputs/**' --glob '!*.log' '192\\.168\\.1\\.190' .
rg -n --glob '!outputs/**' --glob '!tele_operation/outputs/**' --glob '!*.log' '6\\.6\\.7\\.190' run_marvin_m6_impedance.sh install/share/robot_driver/launch tele_operation/config/real_world_env.yaml '双臂遥操系统使用手册 (Dual-Arm Teleop User Manual).md' doc/docker大臂.txt tele_operation/README.md
```

Expected: syntax commands exit 0, old-IP search returns no matches, and new-IP search returns one or more matches in every listed file.

- [ ] **Step 4: Test network reachability without moving the arm**

Run:

```bash
ping -c 3 -W 2 6.6.7.190
```

Expected: three ICMP replies establish host network reachability. If the command reports packet loss or is unavailable, preserve the configuration result and report the network result exactly; do not launch motion or servo-control nodes as part of this test.

### Task 2: Review the change set

**Files:**
- Modify: `docs/superpowers/plans/2026-07-23-marvin-arm-ip-update.md` (mark verification steps complete only after commands finish)

**Interfaces:**
- Consumes: static checks and ping output from Task 1.
- Produces: an evidence-backed summary of configuration and connectivity status.

- [ ] **Step 1: Inspect every changed line**

Run:

```bash
rg -n -C 1 '6\\.6\\.7\\.190|192\\.168\\.1\\.165' run_marvin_m6_impedance.sh install/share/robot_driver/launch tele_operation/config/real_world_env.yaml '双臂遥操系统使用手册 (Dual-Arm Teleop User Manual).md' doc/docker大臂.txt tele_operation/README.md
```

Expected: every changed machine address is `6.6.7.190`, while the host address remains `192.168.1.165`.

- [ ] **Step 2: Report the verification evidence**

Report the specific files changed, syntax-check exit status, old-IP search result, and ping result. Do not claim physical connectivity unless the ping output contains replies.
