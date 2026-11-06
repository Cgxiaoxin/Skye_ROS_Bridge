# Thor 侧还缺什么

本目录目前有：**相机 ensure 脚本（自终端恢复）**、**unijepa 包装 env**、**DAgger 桥接骨架**。  
**没有**完整 `remote_inference_async` 源码。

## 已从终端看到的

| 路径 / 文件 | 状态 |
|-------------|------|
| `~/Desktop/wzh/runtime-left/remote_inference_async/ensure_cameras.sh` | **全文已有**（已落入本目录） |
| `~/zph/run_robot_unijepa.sh` | **仅包装 env**（已落入）；真正逻辑在下面的 `run_robot.sh` |
| `runtime-left/remote_inference_async/run_robot.sh` | **未提供** |
| `remote_inference_async/` 其余 Python（观测、ZMQ/TCP、下发控制器） | **未提供** |
| `~/zph/PYTHON_SDK`、`test_loopback.py` | **未提供** |

## 请优先打包给我（或拷进本目录 `vendor/`）

1. **整目录** `remote_inference_async/`（至少含 `run_robot.sh` + 其 `source`/`python -m` 入口）
2. 写臂路径相关文件（搜 `CONTROLLER_IP` / `6.6.7.191` / `stream` / `joint_control`）
3. 与 5090 通信编解码（端口 `5568`、请求/响应字段）
4. （可选）`test_loopback.py`、`PYTHON_SDK` 里被 client 引用的部分

## 有了之后怎么改（给你 / 现场 AI 的指令）

1. **禁用**对 `CONTROLLER_IP` 的任何写臂 / SDK 调用。  
2. 在拿到一段 action trajectory 后，转成绝对关节角（若已是绝对则直接用），填 `PolicyActionChunk`，pub `/skye/policy_action`。  
3. 与 HITL 同 `ROS_DOMAIN_ID`（Thor 建议 **20**）；不要再起第二路 `skye_robot_driver`。  
4. 对照 `policy_chunk_bridge.py` 与仓内 `pub_dummy_policy_chunk_node.py`。
