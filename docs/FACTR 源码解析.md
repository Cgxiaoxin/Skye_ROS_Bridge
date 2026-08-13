# FACTR 源码解析

## 1. 小臂 ROS2 包源码

原路径：

```text
marvin_ws/src/factr_teleop/scripts/factr_teleop.c
```

相关二进制 / SDK 依赖已落到：

```text
third_party/gento_sdk/lib/x86_64/
```

安装产物（编译后的扩展）：

```text
marvin_ws/install/lib/factr_teleop/factr_teleop.cpython-310-x86_64-linux-gnu.so
```

## 2. 补偿参数（可自由调节）

目录：

```text
marvin_ws/install/share/factr_teleop/configs/
```

常用配置：

| 文件 | 说明 |
|------|------|
| `grav_comp_m6.yaml` | 单配置入口 |
| `grav_comp_m6_left.yaml` | 左臂 |
| `grav_comp_m6_right.yaml` | 右臂 |

修改后**无需重新编译**，重启节点即可生效：

```bash
ros2 run factr_teleop your_teleop_script.py --config configs/grav_comp_m6.yaml
```

## 3. 本地安装（非 Docker）对齐项

1. **Pinocchio 路径**  
   launch 中写死为 `/opt/openrobots/lib/python3.10/site-packages`。可选：
   - 将 Pinocchio 装到该路径；或
   - 改 launch / `export PYTHONPATH` 到实际路径（如 `ros-humble-pinocchio`）。

2. **Dynamixel SDK**  
   需可用：`install/lib/factr_teleop/dynamixel/`，或系统全局安装。

3. **夹爪依赖**（按所用脚本）  
   `pyserial`、`minimalmodbus`、`modbus_tk` 等。

4. **FTDI**  
   `latency_timer=1`。

5. **`ROS_DOMAIN_ID`**  
   与大臂侧一致（文档常用 `20`；Docker 示例可能为 `42`）。
