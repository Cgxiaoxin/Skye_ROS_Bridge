# Operator UI 安全退场设计

**日期：** 2026-09-15  
**状态：** approved（用户确认：去使能默认开启）

## 问题

「结束会话」只对 playbook 进程组发 SIGTERM/SIGKILL。`docker run` 常留下孤儿容器，FACTR 仍占串口；且从不对主臂 Dynamixel 写 Torque Enable=0。

## 目标

结束会话后：

1. 小臂 Docker 容器被强制移除  
2. 左右主臂舵机去使能（默认开启）  
3. 大臂 / align / recorder 仍按原倒序 terminate  
4. 上述兜底失败不阻止回到 IDLE，但写日志  

## 方案

固定容器名 + 结束时 `docker rm -f` + 去使能脚本（默认开）。

### 退场顺序

1. `stop_recorder_if_active` + 清会话模式缓存（已有）  
2. 尽力 `switch_stop`（有 bridge 时）  
3. 倒序 `terminate` playbook 步骤（已有）  
4. `docker rm -f skye_marvin_m6`（best-effort）  
5. `scripts/disable_leader_dynamixel.py`（默认经 Docker 一次性容器执行，因主机常无 `dynamixel_sdk`；失败再回退主机 Python；`teardown.disable_leader: false` 可关）  
6. → IDLE  

先杀容器再去使能，避免串口被 FACTR 占用。

### 配置

```yaml
teardown:
  marvin_container_name: skye_marvin_m6
  disable_leader: true
  docker_rm_timeout_s: 20
  disable_timeout_s: 30
```

### 非目标

- 不自动托臂；UI/文档提示操作员托住小臂  
- 不清理历史未命名容器（一次性手清即可）  
- 急停仍只调 `/gento/emergency_stop`，不拆会话、不跑本退场链  

## 文件

| 路径 | 变更 |
|------|------|
| `scripts/run_marvin_m6_impedance.sh` | `--name skye_marvin_m6`；启动前 `docker rm -f` |
| `scripts/disable_leader_dynamixel.py` | 新建；读 `.skye/leader_arms.env` |
| `skye_operator_ui/.../teardown.py` | `safe_teardown()` |
| `supervisor.py` | stop 末尾调用 |
| `operator_ui_node.py` | 注入 bridge switch_stop |
| `config/default.yaml` | `teardown` 段 |
| `docs/Operator_UI使用说明.md` | 退场说明 |
