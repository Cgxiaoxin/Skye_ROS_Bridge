# Marvin M6 机械臂 IP 更新设计

## 目标

将机械臂地址从 `192.168.1.190` 统一更新为 `6.6.7.190`，确保所有项目入口默认连接到当前机械臂。

## 变更范围

- 容器启动脚本 `run_marvin_m6_impedance.sh` 的 `ROBOT_IP` 默认值。
- 已安装的 ROS 2 驱动启动文件 `robot_servo_start_marvin.launch.py` 和 `robot_servo_start_marvin.launch_pos.py` 的 `ip` 默认值。
- 数据采集配置 `tele_operation/config/real_world_env.yaml` 的 `robot_server.robot_ip`。
- 使用手册和 Docker 启动示例中的旧地址，保持配置与文档一致。

## 行为与兼容性

- 显式传入的 `ROBOT_IP` 环境变量或 `ip:=...` launch 参数仍优先于默认值；本次只改变未显式指定时的默认目标。
- 不改变上位机地址 `192.168.1.165`、端口、USB 设备或夹爪参数。

## 验证

1. 对 Shell 脚本运行 `bash -n`。
2. 对 ROS 2 launch 文件运行 Python 语法编译检查。
3. 搜索项目有效文件，确认不存在旧机械臂 IP。
4. 使用 ping 检查新地址的网络可达性；若 ICMP 被网络策略禁用，报告该限制而不将其误判为驱动配置错误。
