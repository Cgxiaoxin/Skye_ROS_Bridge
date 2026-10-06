# Q_log:遥操掉帧与 "sequence size exceeds remaining buffer" 问题排查与修复(终版)

> 日期:2026-10-06
> 状态:**已修复并验证**
> 现象:operator_ui 终端持续刷屏 `sequence size exceeds remaining buffer`;遥操中夹爪经常掉帧,臂也会掉帧;driver 日志伴随大量 `command timeout (0.500 s); hold ...`。

---

## 1. 问题根因

整套系统通过 `marvin_ws/fastrtps_no_shm.xml` **禁用了共享内存(SHM)传输,全部 DDS 流量走 UDP 环回**,其中包括 `/gento/joint_states` 这类 **250 Hz** 的高频 topic。

而内核 UDP 缓冲区是 Linux 默认值:

```
net.core.rmem_max = 212992   (仅 208 KB)
net.core.wmem_max = 212992
```

在**节点启停 / 会话切换**(participant churn)瞬间,DDS 发现流量、可靠重传与高频数据流叠加,**瞬间打满 socket 接收缓冲,内核大量丢弃 UDP 数据报**。订阅者因此拿到不完整/错位的样本,反序列化失败。

### 报错机理

`sequence size exceeds remaining buffer` 是 `rosidl_typesupport_fastrtps` 生成代码里的一行**裸 `fprintf(stderr)`**:

- `/opt/ros/humble/share/rosidl_typesupport_fastrtps_c/resource/msg__type_support_c.cpp.em:325`
- `/opt/ros/humble/share/rosidl_typesupport_fastrtps_cpp/resource/msg__type_support.cpp.em:225`

反序列化**变长字段**(数组 / string)时发现声明长度超过剩余字节即打印并 `return false`,**该样本被静默丢弃**。所以:

- 报错不走 ROS 日志系统——只刷出错进程的终端,`~/.ros/log` 里查不到;
- 订阅端表现 = **掉帧**(夹爪、臂);
- driver 侧表现 = `command timeout (0.500 s); hold`(命令流中断 0.5 s 后保持当前姿态)。

## 2. 关键证据

1. **丢包实测**:负载较高时 `nstat -az UdpRcvbufErrors` 在 32 秒内暴涨 **13 万**(每秒约 4000 个 UDP 数据报因接收缓冲不足被内核丢弃),历史累计 126 万。
2. **多进程同步爆发**:多个互不相关的订阅进程(含容器内 factr 左/右臂节点)在**同一纳秒级时刻**集体报错、次数严格相等——单个发布者发坏数据不可能造成这种同步性,只有共享的传输层瓶颈可以。
3. **排除类型不匹配**:宿主机与容器内 `JointState` / `Int16MultiArray` / `ParameterEvent` / `ControlMode` 定义逐字节一致(md5 相同);`ros2 topic info -v` 检查全部候选 topic,类型一致、无异常多发布者;双方同为 ROS 2 Humble。
4. **排除特定发布者/特定进程**:任何 topic 对单个新订阅者始终干净;20 个订阅者空载压测 0 报错;仅在"高负载 + participant churn"组合下复现。
5. **修复前后对照**:调大缓冲区并重启后,丢包归零、报错消失(见第 4 节)——因果链闭环。

### 已排除项

- `teleop_file_event_logger.py`(后台诊断日志程序):纯读日志文件,无 rclpy,**无关**。
- `vitai_sensor_capture` 的 `motion.py` / `recorder_gui.py`、`monitor_app.py`、foxglove:均为受害端(只订阅)或无 ROS 代码。
- SHM 撕裂:已通过 `fastrtps_no_shm.xml` 全局禁用 SHM。
- 消息定义版本不一致:宿主/容器 md5 逐字节一致。

## 3. 修复内容(全部改动)

### 改动 1:内核 UDP 缓冲区调大(机器级,已持久化)

新建 `/etc/sysctl.d/60-ros2-dds-udp.conf`,并已 `sysctl -w` 运行时生效:

```
# ROS 2 DDS over UDP tuning (skye teleop)
net.core.rmem_max=16777216     # 16 MB(原默认 212992 = 208 KB)
net.core.wmem_max=16777216
net.core.rmem_default=1048576  # 1 MB
net.core.wmem_default=1048576
```

> **主仓库对齐说明**:此配置不随代码仓库走,属于机器级部署配置。请在每台遥操/数采主机执行:
> ```bash
> sudo ./scripts/setup_host_tuning.sh
> ```
> (脚本会写入 `/etc/sysctl.d/60-ros2-dds-udp.conf` 并立即生效。)否则换机/重装后问题会复现。

### 改动 2:Fast-DDS UDP transport 显式申请大 socket 缓冲(仓库内文件)

文件:`marvin_ws/fastrtps_no_shm.xml`(docker 容器通过 bind-mount 与宿主机共用同一文件,改一处两端生效)

```xml
<transport_descriptor>
  <transport_id>udp_transport</transport_id>
  <type>UDPv4</type>
  <receiveBufferSize>8388608</receiveBufferSize>   <!-- 新增:8 MB -->
  <sendBufferSize>8388608</sendBufferSize>         <!-- 新增:8 MB -->
</transport_descriptor>
```

> 注意:socket 缓冲区在进程创建 transport 时确定,**所有 ROS 节点(含容器)重启后才生效**。
>
> **本仓库**:换机/重装后请在宿主机执行:
> ```bash
> sudo ./scripts/setup_host_tuning.sh
> ./scripts/setup_host_tuning.sh --check
> ```

### 改动 3(运维操作 + 代码):清理残留重复节点

排查时发现重启会话后,上一会话的 `follower_align`、`host_keyboard_align`、`skye_data_recorder` 共 5 个进程未被清理,同名节点双跑(两个 `follower_align` 同时发布 `/align/status`,有控制风险),已手动清理。

**仓库内已落地**:`skye_operator_ui/config/default.yaml` 的 `cleanup_stale_commands` 增加上述进程的 `pkill`;会话启动预检**总会**执行该列表(不再仅在发现残留 driver 时才清),避免异常退出后累积重复节点。
## 4. 修复后验证(2026-10-06 重启遥操后实测)

| 检查项 | 修复前 | 修复后 |
|---|---|---|
| `UdpRcvbufErrors` 增量(遥操运行中,30 秒) | 32 秒 **+13 万** | **0** |
| 12 个订阅者压测高频 topic(25 秒) | 每个进程 ~17 条报错 | **0 条** |
| `ros2 topic echo` 各 topic | 间歇报错 | 干净 |
| ROS 节点图 | 存在同名节点双跑 | 每个节点单实例 |

## 5. 可选项(暂不做)

- **容器内 Fast-DDS 版本对齐**:宿主机为 2.6.12,容器镜像内为 2.6.11。2.6.12 修复列表含 "Several fixes in TopicPayloadPool"(与本类负载缓冲问题相关)。因容器闭源、重建镜像成本高,且问题已通过缓冲区调优修复,**暂忽略**;下次镜像更新时顺带将 `ros-humble-fastrtps` 升至 2.6.12 即可。
- **高频 topic 降频**:`/gento/joint_states` 以 250 Hz 发布,如后续仍有带宽压力,可按实际消费需求降频。

## 6. 复盘要点(防复发)

1. 换机 / 重装系统后必须重新应用改动 1(sysctl),建议纳入装机脚本。
2. 节点/会话重启后出现报错时,先检查是否有残留同名节点双跑(`ros2 node list` 出现重复节点名)。
3. 排查 DDS 丢包的快捷命令:
   ```bash
   nstat -az UdpRcvbufErrors UdpInErrors   # 看增量,不看绝对值
   sysctl net.core.rmem_max net.core.wmem_max
   ```
