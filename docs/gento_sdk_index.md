# Gento SDK 接口索引文档

> 适用范围：底层厂商 C++ SDK `third_party/gento_sdk/include/` 下的全部头文件。
> 本文为**接口索引**，按模块/功能分组，以表格列出公开 API。详细参数含义以源码 Doxygen 注释为准。
> SDK 版本：v4.4.2（`FXCommon.h` 中 `FX_SDK_MAJOR/MINOR/PATCH_VERSION`）。

---

## 1. 模块索引

| 模块 | 主要头文件 | 用途 | 接口形态 |
|---|---|---|---|
| 公共类型 / 错误码 | `Common/FXCommon.h`, `FXType.h`, `FXErrorCode.h`, `FXCmplOpt.h` | 标量别名、对象/状态枚举、错误码、平台兼容 | 类型/枚举/宏 |
| L0 控制层 | `L0Control/L0Robot.h`, `UdpCommon.h`, `Utility.h` | 底层 UDP 直接控制（系统/参数/状态/配置/实时运动） | C 函数 `FX_L0_*` |
| RobotCtrl 单例 | `L0Control/RobotCtrl.h` | L0 C API 的 C++ 单例封装（镜像 L0） | C++ 类（`static`） |
| L1 高层 API | `L1Robot/L1Robot.h`, `SampleOffsetTable.h` | 高层封装：连接/状态机/反馈/运动学入口 | C 函数 `FX_L1_*` |
| 运动学与规划 | `Kinematics/*` | 正/逆运动学、雅可比、轨迹规划、数学库、负载辨识 | C 函数 + C++ 类 |
| 文件传输 | `FileClient/*` | 向控制器收发文件 | C 函数 + C++ 类 |

**自由度（DOF）约定**：机械臂 Arm=7，头部 Head=3，机身 Body=6，升降 Lift=2，手 Hand=24 路。

---

## 2. 公共类型与错误码（Common）

### 2.1 标量类型别名（`FXType.h`）
跨平台固定宽度别名（Windows/Linux 一致），均为宏定义。

| 别名 | 含义 |
|---|---|
| `FX_BOOL` / `FX_TRUE` / `FX_FALSE` | `unsigned char` 布尔 |
| `FX_CHAR` / `FX_UCHAR` | `char` / `unsigned char` |
| `FX_INT8/16/32/64` | 有符号整型（32=int，64=long long） |
| `FX_UINT8/16/32/64` | 无符号整型 |
| `FX_FLOAT` / `FX_DOUBLE` | 浮点 |
| `FX_VOID` | `void` |
| `FX_CSTR` | `const char *` |
| `FX_INT32L` / `FX_UINT32L` | 平台宽度敏感长整型（64 位 Linux 下为 int） |

### 2.2 对象与状态枚举（`FXCommon.h`）

**`FXObjType`** 控制对象标识：`FX_OBJ_ARM0=0`(左臂) `FX_OBJ_ARM1=1`(右臂) `FX_OBJ_HEAD=2` `FX_OBJ_BODY=3` `FX_OBJ_LIFT=4`；位标志 `FX_OBJ_*_FLAG`（`1<<n`），`FX_OBJ_ALL_FLAG` 全部，`FX_DEFAULT_THREAD_ID=0`。

**`FXRobotType`** 机型：`FX_ROBOT_NULL=0` `FX_ROBOT_MARVIN_PRO_M3=1` `FX_ROBOT_MARVIN_PRO_M6=2` `FX_ROBOT_GENTO_SKYE=3` `FX_ROBOT_GENTO_LUNA=4`。

**`FXStateType`** 全局状态机：`FX_STATE_IDLE=0` `POSITION=1` `IMP_JOINT=2` `IMP_CART=3` `IMP_FORCE=4` `DRAG_JOINT=5` `DRAG_CART_X/Y/Z/R=6..9` `RELEASE=10` `PD=11` `ERROR=100` `TRANSFERRING=101` `UNKNOWN=200`。

**各部件状态机**（值同构）：`ARM_STATE_*` / `HEAD_STATE_*` / `BODY_STATE_*` / `LIFT_STATE_*`：`_IDLE=0` `_POSITION=1` `_TORQUE=2` `_RELEASE=3` `_TRANS_TAG=50` `_ERROR=100` `_TRANS_TO_POSITION=101` `_TRANS_TO_TORQUE=102`(Body/Arm) `_TRANS_TO_RELEASE=103`(Body/Arm) `_TRANS_TO_IDLE=109`。

**手 / 力控 / 阻抗枚举**：

| 枚举 | 取值 |
|---|---|
| `FXHandType` | `FX_HAND_LEFT=0` `FX_HAND_RIGHT=1` |
| `FXHandAction` | `DISABLE=0` `ENABLE=1` `RESET=2` |
| `FXHandState` | `DISABLED=0` `ENABLED=1` `ERROR=100` |
| `FXForceDef` | `DIR_X=0` `DIR_Y=1` `DIR_Z=2` `VALUE=3` `DISTANCE=4`（`FX_FORCE_DEF_NUM=5`） |
| `FXTorqueDef` | `DIR_A=0` `DIR_B=1` `DIR_C=2` `VALUE=3` `ANGLE=4`（`FX_TORQUE_DEF_NUM=5`） |
| `FXImpType` | `NULL=0` `JOINT=1` `CART=2` `FORCE=3` `PD=4` |
| `DragType` | `NULL=0` `JOINT=1` `CART_X/Y/Z/R=2..5` |
| `FXObjPhysicalState` | `NOT_USED=0` `VIRTUAL=1` `REAL=2` |
| `FXTerminalType` | `ARM0=0` `ARM1=1`；`FXChnType`：`CANFD=1` `485A=2` `485B=3` |
| `FXUserDataType` | `char..double`（0..12，4 跳过）；`FXParamType`：`INT`/`FLOAT` |

**主要数据结构**（`_IN`=命令输入 `_OUT`=快反馈 `_GET`=扩展反馈 `_SET`=配置 `_RT`=实时包(1kHz) `_SG`=慢组包(500Hz)）：
`StateCtr`(状态控制) · `ARM_*/HEAD_*/BODY_*/LIFT_*/HAND_*` 四件套 · `OP_SET`(指令集) · `ROBOT_RT`/`ROBOT_SG`(整机反馈) · `DDSS`(终端包)。

### 2.3 错误码（`FXErrorCode.h`）

**`FXFuncReturn`** 函数返回值（成功 `FUNC_RET_SUCCESS=0`；失败负段）：

| 区段 | 代表值 | 含义 |
|---|---|---|
| 通用 | `-1` `OPERATION_FAILED` | 通用失败 |
| 链路 | `-2` `LINK_FAILED` `-3` `LINK_NO_RESPONSE` `-22` `LINK_REJECTED` | 连接/被占用 |
| 版本 | `-4` `VERSION_INCOMPATIABLE` | 版本不匹配 |
| 参数 | `-5..-6` `INVALID_INPUT_ARG/CONDITION` `-11..-13` 参数存取/保存失败 | 参数错误 |
| 通信 | `-16..-19` 等待就绪/发送/回复超时、命令格式化失败 | 通信错误 |
| 对象/线程 | `-9` `INVALID_OBJ` `-20` `INVALID_ROBOT_TYPE` `-21` `INVALID_HAND_TYPE` `-23` `INVALID_THREAD_ID` | 类型/句柄错误 |
| 文件 | `-7` `SEND_FILE_FAILED` `-8` `RECV_FILE_FAILED` | 文件传输 |
| 用户数据 | `-24..-26` 采样项过多/无效/长度无效 | 反馈注册 |
| 运动学(-1000) | `-1000` 未初始化 `-1010` IK 不可达 `-1011` 超限 `-1020` 规划失败 `-1021` 关节限位 `-1022` 笛卡尔不可达 `-1023` 轨迹点溢出 `-1024` 双臂点不一致 `-1030` 动力学辨识失败 `-1099` 内部错误 | 运动学/规划 |

**`FXErrorCode`** 系统/伺服错误：`ERR_Internal=100` `ERR_Emcy=101` `ERR_Servo=102` `ERR_Request/ResponsePositionMode=104/105` `ERR_Request/ResponseTorqueMode=106/107` `ERR_ServoStateAbnormal=111` `ERR_BusLinkDown=114` 等（另有配置类 1..7）。

### 2.4 版本 / 日志宏
| 宏 | 值 | 含义 |
|---|---|---|
| `FX_SDK_MAJOR/MINOR/PATCH_VERSION` | 4 / 4 / 2 | SDK 版本 |
| `MAKE_VERSION(maj,min,sub)` | 打包 32 位版本 | 版本构造 |
| `FX_LOG_NULL/DEBG/INFO/WARN/ERROR/ALL_FLAG` | 位掩码 `1<<0..3` | 日志级别 |

---

## 3. L0 控制层 API（`L0Control/L0Robot.h`）

> C 风格 `extern "C"` 自由函数，命名 `FX_L0_<部件>_<分组>_<动作>`；多数返回 `int`（0 成功 / -1 失败）。`UdpCommon.h` 定义指令枚举 `UdpInsType`/`OpInsType` 作为底层协议参考。

### 3.1 系统管理 / 参数（唯一函数）
| 函数 | 说明 |
|---|---|
| `FX_L0_System_RequestControl(ip1..4)` | 请求控制目标系统 |
| `FX_L0_System_Link(ip1..4)` | 建立 UDP 连接 |
| `FX_L0_System_Unlink()` | 断开 UDP 连接 |
| `FX_L0_System_GetLinkState()` | 链路状态（0断/1通/-1通但100ms无数据） |
| `FX_L0_System_Testconnect()` | 测试连接 |
| `FX_L0_System_CheckVersion()` | 校验 SDK/控制器版本兼容 |
| `FX_L0_System_GetControllerVersion()` / `GetSdkVersion()` | 取控制器/SDK 版本 |
| `FX_L0_System_Reboot()` / `Update()` | 重启/系统更新 |
| `FX_L0_System_Set/GetPDCmdCycleTime(ms)` | 设置/获取 PD 周期(ms) |
| `FX_L0_System_LocalLogOn()` / `LocalLogOff()` | 本地日志开关 |
| `FX_L0_Param_GetInt/GetFloat/GetString(name, *out)` | 按名读参数 |
| `FX_L0_Param_SetInt/SetFloat(name, val)` | 按名写参数 |
| `FX_L0_Param_Save()` | 保存参数到持久存储 |

### 3.2 终端透传（Arm0 / Arm1）
`FX_L0_Arm{0,1}_Terminal_ClearData()` · `GetData(*chn,*buf[64])` · `SetData(chn,*data,len)`（通道：1=CANFD, 2/3=485）。

### 3.3 反馈数据访问（唯一函数）
| 函数 | 说明 |
|---|---|
| `FX_L0_GetRobotRT()` | 取实时反馈 `const ROBOT_RT *`（1kHz） |
| `FX_L0_GetRobotSG()` | 取慢组反馈 `const ROBOT_SG *`（500Hz） |

### 3.4 按部件重复的接口组
下列接口对每个对象以 **`FX_L0_{部件}_{分组}_{动作}`** 形式存在，`{部件}` 与支持下标见下表（缺项表示不支持）。

| 功能组 | 动作 | 支持部件 | 备注 |
|---|---|---|---|
| `State` | `GetServoErrorCode(axis,*code)` | Arm0,Arm1,Head,Body,Lift | 伺服错误码 |
| `State` | `GetServoVersion(axis,ver[30])` | Arm0,Arm1,Head,Body | 伺服版本 |
| `State` | `GetSensorVersion(axis,*ver)` / `GetSensorSerial(axis,*ser)` | Arm0,Arm1,Body | 传感器版本/序列号 |
| `State` | `GetPhysicalState(*st)` | Arm0,Arm1,Head,Body,Lift | 物理使用状态 |
| `State` | `Reset()` | Arm0,Arm1,Head,Body,Lift | 复位错误态 |
| `Config` | `SetBrakeLock/SetBrakeUnlock(axis_mask)` | Arm0,Arm1,Head,Body,Lift | 制动锁/解锁 |
| `Config` | `ResetEncSingleTurn/ResetEncMultiTurn/ClearEncError(axis_mask)` | Arm0,Arm1,Head,Body | 编码器/错误清 |
| `Config` | `DisableSoftLimit(axis_mask)` | 全部 | 关软限位 |
| `Config` | `SetSensorOffset(axis,off)` | Arm0,Arm1,Body | 设传感器偏移 |
| `Config` | `ResetEncOffset(axis_mask)` | Lift | 升降编码器偏移复位 |
| `Runtime` | `EmergencyStop/SetState/SetTag(thread_id,...)` | 全部 | 急停/状态/标签 |
| `Runtime` | `SetJointPosCmd(thread_id,pos[N])` | Arm0/Arm1(7), Head(3), Body(6), Lift(2) | 关节位置指令 |
| `Runtime` | `SetJointTorCmd/SetForceCtrl/SetTorqueCtrl` | Arm0,Arm1 | 力矩/力/扭矩控制 |
| `Runtime` | `SetCmdPDSerial/SetVelRatio/SetAccRatio` | Arm0,Arm1,Head,Body,Lift | PD序列/速度/加速度比 |
| `Runtime` | `SetJointK/SetJointD/SetCartK/SetCartD` | Arm0,Arm1 | 关节/笛卡尔 刚度/阻尼 |
| `Runtime` | `SetToolK[6]/SetToolD[10]` | Arm0,Arm1 | 工具运动学/动力学 |
| `Runtime` | `SetImpType/SetDragType` | Arm0,Arm1 | 阻抗/拖拽类型 |
| `Runtime` | `InitTraj/SetTraj/RunTraj/StopTraj` | Arm0,Arm1,Body,Lift | 轨迹缓冲/执行 |
| `Runtime` | `SetCmdAction/SetCmdPos/SetCmdP/SetCmdD/SetCmdMaxTor` | Hand0,Hand1(24) | 手动作/位置/P/D/最大力矩 |

> 注：`thread_id` 取值 0~7。Hand 相关以 `FX_L0_Hand{0,1}_Runtime_*` 命名。

---

## 4. RobotCtrl 单例（`L0Control/RobotCtrl.h`）

> C++ 单例，方法集镜像 L0 C API（均 `static`，返回 `FX_BOOL`/`FX_INT32`/`FX_VOID`）。`GetIns()` 取实例。内部方法 `DoCnt/DoRecv/DoSend/DoBeat/WaitOpReturn` 标注“用户勿用”，不在对外范围。

### 4.1 连接 / 日志 / 系统 / 参数
| 方法 | 说明 |
|---|---|
| `Link/Unlink(ip1..4)` | 建/断 UDP 链路并启动收发线程 |
| `RequestControl(ip1..4)` | 请求控制 |
| `TestLink()` | 连通性测试(返回耗时ms) |
| `IsLinked()` / `GetLinkState()` | 链路查询 |
| `OnLocalLogOn/Off()` | 本地日志开关 |
| `System_GetControllerVersion/SdkVersion` / `CheckVersion` / `Reboot` / `Update` | 版本/重启/更新 |
| `System_Set/GetPDCmdCycleTime(ms)` | PD 周期 |
| `Para_GetInt/GetFloat/GetString` / `Para_SetInt/SetFloat` / `Para_Save` | 参数读写/保存 |

### 4.2 终端 / 状态 / 配置 / 实时运动
- **终端**：`Arm0/Arm1_Terminal_ClearData` · `GetData(*chn,*buf[64])` · `SetData(chn,*data,len)`。
- **状态/配置/实时运动**：与 §3.4 同构，方法命名为 `RobotCtrl::{部件}_{分组}_{动作}`（如 `Arm0_State_GetServoErrorCode`, `Body_Runtime_SetJointPosCmd[6]`, `Hand0_Runtime_SetCmdPos[24]`）。公共数据成员 `m_RobotRT`(ROBOT_RT)、`m_RobotSG`(ROBOT_SG)。

---

## 5. L1 高层 API（`L1Robot/L1Robot.h`）

> `extern "C"` 高层封装，命名 `FX_L1_<分组>_<动作>`；返回 `int` 状态码或具体枚举/指针。`FX_MotionHandle`（`struct FX_MotionContext *`）为运动学上下文句柄。

### 5.1 系统级 / 反馈
| 函数 | 说明 |
|---|---|
| `FX_L1_System_Link(ip1..4, log_level)` | 建链+版本协商+初始化日志 |
| `FX_L1_System_Unlink()` / `GetLinkState()` | 断链 / 链路状态 |
| `FX_L1_System_Set/GetLogLevel()` | 全局日志级别 |
| `FX_L1_System_GetControllerVersion/SdkVersion` / `Reboot` | 版本/重启 |
| `FX_L1_System_Update(local,ini)` / `SendFile` / `RecvFile` | 更新/文件收发 |
| `FX_L1_Fbk_GetRobotType()` | 取机型 `FXRobotType` |
| `FX_L1_Fbk_CurrentState(obj)` | 取对象当前状态 `FXStateType` |
| `FX_L1_Fbk_GetCtrlObjDof(obj)` | 取对象自由度 |
| `FX_L1_Fbk_GetCtrlObjServoVersion/SensorVersionAndSerial/PhysicalState(obj,...)` | 伺服/传感器/物理态 |
| `FX_L1_Fbk_GetRT()` / `GetSG()` | 取 `ROBOT_RT*`/`ROBOT_SG*` |
| `FX_L1_Fbk_RegisterUserDataSet/ResetUserDataSet/CheckUserDataSet/GetUserData` | 用户反馈数据注册/采样 |

### 5.2 状态机 / 参数 / 终端 / 配置
| 函数 | 说明 |
|---|---|
| `FX_L1_State_GetServoErrorCode(obj,*code[7])` | 伺服错误码 |
| `FX_L1_State_ResetError(obj,timeout,*sysErr)` | 复位错误 |
| `FX_L1_State_SwitchToIdle/PositionMode/ImpJointMode/ImpCartMode/ImpForceMode/PDMode` | 切到各控制模式 |
| `FX_L1_State_SwitchToDragJoint/DragCartX/Y/Z/R` | 切到拖拽模式 |
| `FX_L1_State_SwitchToCollaborativeRelease(obj,timeout)` | 协作释放 |
| `FX_L1_Param_Set/GetInt32` · `Set/GetFloat` · `GetString` | 参数读写 |
| `FX_L1_Terminal_Clear/Get/SetData(term,chn,...)` | 终端透传 |
| `FX_L1_Config_SetBrakeLock/Unlock/ResetEncOffset/ClearEncError/ResetAxisSensorOffset/ResetSensorOffset/DisableSoftLimit(obj,mask)` | 硬件配置 |
| `FX_L1_Config_Set/GetPDCmdCycleTime(ms)` · `SetTraj(obj,n,data)` | PD周期/轨迹配置 |

### 5.3 实时运动控制
| 函数 | 说明 |
|---|---|
| `FX_L1_Runtime_EmergencyStop(thr,obj_mask)` | 急停（返回停止掩码） |
| `FX_L1_Runtime_SetTag(thr,obj,tag)` | 标签指令 |
| `FX_L1_Runtime_SetJointPosCmd/SetJointPosPDCmd(thr,obj,pos[7])` | 关节位置指令 |
| `FX_L1_Runtime_SetForceCtrl/SetTorqueCtrl(thr,obj,f[5]/t[5])` | 力/扭矩控制 |
| `FX_L1_Runtime_SetVelRatio/SetAccRatio/SetSpeedRatio(thr,obj,...)` | 速度/加速度比 |
| `FX_L1_Runtime_SetJointK/D/KD/SetCartK/D/KD(thr,obj,k[7],d[7])` | 刚度/阻尼 |
| `FX_L1_Runtime_SetToolK/D/KD(thr,obj,k[6],d[10])` | 工具运动学/动力学 |
| `FX_L1_Runtime_SetBodyPD/PDD/PD(thr,p[6],d[6])` | 机身 PD 增益 |
| `FX_L1_Runtime_RunTraj/StopTraj(thr,obj_mask)` | 轨迹执行/停止（返回掩码） |
| `FX_L1_Runtime_SetHandAction/Pos/P/D/MaxTor(thr,hand,...)` | 手动作/位置/P/D/最大力矩(24) |

### 5.4 运动学与规划入口
见 §6。L1 以 `FX_L1_Kinematics_*` 暴露初始化/正逆解/规划/姿态转换，句柄 `FX_MotionHandle`。

---

## 6. 运动学与轨迹规划（Kinematics）

> 两套并行 API：**扁平 C API**（`L0KineMotion.h`/`FXKinematics.h`/`FXMath.h`/`FXMatrix.h`）与 **C++ 类 API**（`CFxPln`/`CFxKineIF`/`CFxKineMAX`/`CFxIFEnv`/`CPointSet` 等）。类内部调用 C 函数。

### 6.1 上下文与单臂（C API，`L0KineMotion.h`）
| 函数 | 说明 |
|---|---|
| `FX_L0_Kinematics_create()` | 创建上下文(返回句柄/NULL) |
| `FX_L0_Kinematics_destroy(h)` | 销毁上下文 |
| `FX_L0_Kinematics_set_log_level(lvl)` | 设置日志级别 |
| `FX_L0_Kinematics_init_single_arm(h,RobotSerial,type,DH[8][4],PNVA[8][4],BOUND[4][3],GRV[3],MASS[7],MCP[7][3],I[7][6])` | 初始化单臂运动学+动力学 |
| `FX_L0_Kinematics_set_tool/remove_tool(h,serial,tool[4][4])` | 设置/移除工具变换 |
| `FX_L0_Kinematics_forward_kinematics(h,serial,joints[7],pose[4][4])` | 正运动学 |
| `FX_L0_Kinematics_jacobian(h,serial,joints[7],jcb[6][7])` | 雅可比 |
| `FX_L0_Kinematics_inverse_kinematics(h,serial,*FX_InvKineSolvePara)` | 逆运动学 |

### 6.2 机身（MAX Body）/ 规划 / 转换
| 函数 | 说明 |
|---|---|
| `FX_L0_Kinematics_set_body_condition(h,std_body[3],k_body[3],std_L,k_L,std_R,k_R)` | 设机身刚度参数 |
| `FX_L0_Kinematics_body_forward(h,jv[3],pgL[4][4],pgR[4][4])` | 机身正运动学 |
| `FX_L0_Kinematics_calc_body_position(_with_ref)(h,ref,left_tcp[3],right_tcp[3],out[3])` | 双臂TCP→机身关节 |
| `FX_L0_Kinematics_plan_joint_move(h,serial,start[7],end[7],vel,acc,freq,*pset,*n)` | 关节空间 MoveJ |
| `FX_L0_Kinematics_plan_linear_move(h,serial,startXYZABC[6],endXYZABC[6],refJ[7],vel,acc,freq,*pset,*n)` | 笛卡尔 MoveL |
| `FX_L0_Kinematics_plan_linear_keep_joints(...)` | 保持姿态的线性规划 |
| `FX_L0_Kinematics_multi_points_set_movl_start/next_points/get_movl_path(...)` | 多点连续 MoveL |
| `FX_L0_Kinematics_plan_dual_arm_fixed_body(h,*ArmsSynchronousPlanningParams,*arm0,*arm1,*n)` | 双臂同步固定机身 |
| `FX_L0_Kinematics_dynamics_identification(type,path,*mass,mr[3],I[6])` | 负载动力学辨识 |
| `FX_L0_XYZABC2Matrix/Matrix2XYZABC(xyzabc[6],m[4][4])` | 姿态↔矩阵转换 |
| `FX_L0_CPointSet_Create/Destroy/OnInit/OnGetPointNum/OnGetPoint/OnSetPoint/OnAppendPoint` | 点集对象管理 |

**关键结构体**：`FX_InvKineSolvePara`（IK 输入输出块：目标TCP/参考关节/零空间参数→输出关节）、`ArmsSynchronousPlanningParams`（双臂同步规划块）、`FX_MOTION_RET` 返回值（`OK=0`…`DYNAMICS_IDENT_FAILED=30`）。

### 6.3 C++ 类 API
| 类 | 头文件 | 用途 / 关键方法 |
|---|---|---|
| `CFxPln` | `MotionPlanner/FXMotionPlanner.h` | 单/双臂规划：`OnInitEnv_SingleArm` `OnMovJ` `OnMovL` `OnMovL_KeepJ` `MultiPoints_*` `OnMovL_DualArm_FixBody` `XYZABC2Matrix4_DEG` `GetLastError` |
| `CFxKineIF` | `ArmKinematics/FXArmKinematics.h` | 单臂接口：`OnInitEnv` `OnSetTool` `OnRmvTool` `OnSolveArmFK/Jcb/IK` `OnGetArmLmt` |
| `CFxKineMAX` | `SkyeBodyKinematics/FXSkyeBodyKinematics.h` | 机身运动学：`OnSetCondition` `OnKineLR` `OnCalBody(_withref)` |
| `CFxIFEnv` | `KineCommon/FXEnviroment.h` | 环境容器：`OnInitEnv` `OnGetArmType/Lmt/KinePara/DynPara` `OnCheckEnvValid` |
| `CPointSet` | `KineCommon/PointSet.h` | 点集：`OnInit` `OnGetPointNum` `OnGetPoint` `OnSetPoint` `OnSave/Load(CSV/XFile/Exp)` `OnMult/Add/Sub*`（列运算/滤波/子集） |
| `CFXDG` | `KineCommon/FXDG.h` | 动态数组：`OnInit` `OnAdd` `OnGet` `OnGetNum` |
| `CLog` | `KineCommon/FXLog.h` | 日志：`SetLogOn/Off` `SetLogLevel` `Debug/Info/Warn/Error` |
| `CO3Polynorm` | `MotionPlanner/FXO3Polynorm.h` | 三次多项式：`CalPnY/FD/SD` `CalXPara` `CalPnPara(SoC)`（静态） |
| `CAxisPln` / `CAxisJointPln` / `CMovingAverageFilter` | `MotionPlanner/FXAxisPln.h` | 轴/关节规划器、双臂固定机身、`OnMovL(_ZSP)` `OnMovJoint` `OnSetLmt` `FilterPointSet`（`WINDOW_SIZE=5`） |

### 6.4 扁平 C 运动学 / 数学库
- **`FXKinematics.h`**：`FX_Robot_Init_Type/Kine/Lmt` `FX_Robot_Tool_Set/Rmv` `FX_Robot_Kine_FK/FK_NSP/Jacb/IK(_NSP)`；枚举 `FX_ROBOT_TYPES` `FX_PILOT_NSP_TYPES` `FX_ROTATION_TYPE`；结构体 `FX_Robot` `FX_RobotLmt` `FX_DynBase` `FX_KineBase` `FX_Jacobi` `FX_InvKineSolvePara` `FX_InvDynaSolvePara`；`MAX_RUN_ROBOT_NUM=10`。
- **`FXMatrix.h`**（矩阵/向量库，全部 `FX_DOUBLE*` 指针）：向量拷贝/加减(`Vect3..8`)、矩阵拷贝/转置/乘(`M33..M88`)、矩阵×向量(`MMV3..8`,`MMV677/766`)、加、逆(`MatrixInv33..88`)、阻尼逆(`MatrixInvDP`)、SVD(`FX_SVDM_33..88`)、上三角(`FX_UTM`)、行列式(`FX_DetM`)、点积、对称伪逆 `FX_SPMatInv77`、姿态/欧拉(`FX_Matrix2ZYZ/ZYX`,`FX_ZYZ2Matrix`…)、轴旋 `FX_MatRotAxis`、DH `Tmat`、四元数(`FX_QuatMult/Norm/Conj/Inverse/Slerp/ABC2Quaternions/Matrix2Quaternion`)、欧拉旋转矩阵(12×2 变体)。
- **`FXMath.h`**（标量）：`FX_Value_Sig` `FX_Fabs` `IsZero(_L)` `FX_SIN/COS_ARC/DEG` `FX_ATan2` `FX_ACOS` `FX_Sqrt` `FX_3Root` `FX_MinDif_Circle` `FX_Floor` `FX_Max/Min`。
- **`FxMathType.h`**：类型别名 `Matrix3/4/6/7/8/67/76` `Vect3/4/6/7/8` `Quaternion` `PosGes`；常量 `FXARM_D2R` `R2D` `PI` `HLFPI` `2PI` `EPS` 等。
- **`FXDynaIndentification.h`**：`OnCalLoadDyn(*LoadDynamicPara, RobotType, UserPath)`；结构 `LoadDynamicPara{m,r[3],I[6]}`；枚举 `LoadIdenErrCode`。

---

## 7. 文件传输客户端（FileClient）

> 向控制器文件服务器收发文件。对外推荐入口为扁平 C 函数；底层为 C++ 类分层。

### 7.1 对外入口（推荐，`FXFileClient.h`）
| 函数 | 说明 |
|---|---|
| `FX_FileClient_SendFile(ip1..4, local_file, remote_file)` | 发送本地文件到远程 |
| `FX_FileClient_RecvFile(ip1..4, local_file, remote_file)` | 从远程接收文件到本地 |

> 返回 `int`（通常 1=成功，0=失败）。

### 7.2 类分层与关键方法
`CTCPFileClient`(→`CTCPAgent`→`CParser`→`CFileOp`)，头文件 `TCPFileClient.h`/`TCPAgent.h`/`Parser.h`/`FileOP.h`：

| 类 | 关键方法 | 用途 |
|---|---|---|
| `CTCPFileClient` | `OnSendFile(lpath,rpath)` `OnRecvFile(lpath,rpath)`（继承 `OnLinkTo/OnSend/OnCheckLink/OnQuit`） | 文件传输代理 |
| `CTCPAgent` | `OnLinkTo(ip1..4,port)` `OnSend` `OnCheckLink` `OnQuit` | TCP 连接/收发线程 |
| `CParser` | `OnAddRawData` `OnUnPack` `OnPack` `OnGetContent` | 帧封装/解包 + CRC |
| `CFileOp` | `OnIns` `OnSendFile` `OnRecvFile` `OnCheckStateOK` `Empty` `SetErr` | 文件状态机 |

### 7.3 常量 / 枚举
| 名称 | 值/含义 |
|---|---|
| `MAX_FILE_CELL_SIZE` | 80000（单帧文件块字节） |
| `MAX_TCP_BUFSIZE` / `MAX_PARSER_BUFSIZE` | 81000（TCP 帧缓冲） |
| `InsType` | 帧类型：`Send/Get_Request(_Report)` `Send/Get_File_Cell(_Report)` `Error`(1..9) |
| `File_OP_State` | 状态：`OK=1` `Send_SVR/CLN` `Recv_SVR/CLN`(2..5) |
| `FileIns` | 帧结构：`m_InsType` `m_TotalBlockNum` `m_CurrentBlockSerial` `m_CellSize` `m_ErrorCode` `m_CellContent[80001]` `CRC` |

---

> 文档生成依据：`third_party/gento_sdk/include/` 下全部头文件（Common / L0Control / L1Robot / Kinematics / FileClient）。如需逐函数完整签名，请直接查阅对应头文件源码。
