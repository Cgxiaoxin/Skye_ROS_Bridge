#!/usr/bin/env python3
# coding=utf-8
'''
Author       : Jay jay.zhangjunjie@outlook.com
Date         : 2024-05-16 17:19:52
LastEditTime : 2025-02-13 18:45:34
LastEditors  : error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
Description  : 
'''
from enum import IntEnum
from .modbus import ModbusRTU
import time

DEFAULT_SYNC_TIME = 0.01

# --------------------------------------------------------
# Request 
class RACT(IntEnum):

    Deactivate = 0x0        # reset the gripper and clear any fault status
    Activate   = 0x1        # activate gripper, the first step befor all operation


class RGTO(IntEnum):
    Stop = 0x0              # 
    Go   = 0x1


class RATR(IntEnum):
    Normal       = 0x0
    EAutoRelease = 0x1


class RARD(IntEnum):
    CloseAutoRelease = 0x0
    OpenAutoRelease  = 0x1
# --------------------------------------------------------


# --------------------------------------------------------
# Response
class GACT(IntEnum): 
    GripperReset            = 0x00      # 
    GripperActivation       = 0x01


class GGTO(IntEnum): 
    Stopped                 = 0x00      
    Moving                  = 0x01


class GSTA(IntEnum): 
    InResetOrAutoRelease    = 0x00
    Activating              = 0x01
    NotUsed                 = 0x02
    ActivationCompleted     = 0x03


class GOBJ(IntEnum): 
    MovingAndNoObj          = 0x00
    ObjDetectedOpening      = 0x01
    ObjDetectedClosing      = 0x02
    MovDoneAndNoObj         = 0x03



# --------------------------------------------------------
class FaultStatus(IntEnum):
    NoFault = 0x00



def PosCheck(value):
    minValue = 0x00
    maxValue = 0xFF

    if value <= minValue: return minValue
    if value >= maxValue: return maxValue
    return value



SpeedCheck = PosCheck
ForceCheck = PosCheck




class HandERegister(IntEnum)   : 
    REQUEST_ACTION    = 0x03E8  # low byte, write
    REQUEST_POSITION  = 0x03E9  # high byte, write
    REQUEST_SPEED     = 0x03EA  # low byte, write
    REQUEST_FORCE     = 0x03EA  # high byte, write


    GRIPPER_STATUS    = 0x07D0  # low byte, read
    FAULT_STATUS      = 0x0701  # low byte, read
    ECHO_POSITION     = 0x0701  # high byte, read
    POSITION          = 0x07D2  # low byte,read
    CURRENT           = 0x07D2  # high byte,read


    @classmethod
    def getHighByteValue(cls, value):
        return (value >> 8) & 0xFF

    @classmethod
    def getLowByteValue(cls, value):
        return value & 0xFF


class HandEForRtu:
    DEFAULT_SLAVE_ID = 0x0009
    DEFAULT_BAUDRATE = 115200
    DEFAULT_DATA_BIT = 8
    DEFAULT_STOP_BIT = 1
    DEFAULT_PARITY   = "N"

    FULL_POS = 50
    POS_CONVERSION_RATIO = 0.1953125    # 50mm / 256
    MIN_POS = 50    # 0.001953125mm

    # 力控核心参数（与电流-力映射保持一致）
    FORCE_MIN = 20.0        # 最小夹持力（N）
    FORCE_MAX = 185.0       # 最大夹持力（N）
    FORCE_TO_REG = (255.0 - 0.0) / (FORCE_MAX - FORCE_MIN)  # N → 寄存器值（0~255）转换系数

    def __init__(self, port, autoInit: bool=True) -> None:
        
        self.master = ModbusRTU(port=port,
                                baudrate=self.DEFAULT_BAUDRATE,
                                bytesize=self.DEFAULT_DATA_BIT,
                                parity=self.DEFAULT_PARITY,
                                stopbits=self.DEFAULT_STOP_BIT
                                )
        self.__rAct, self.__rGto, self.__rAtr, self.__rArd = None, None, None, None
        self.__gAct, self.__gGto, self.__gSta, self.__gObj = None, None, None, None
        self.__readGripperAction()
        self.readGripperStatus()

        self.__rAct = self.__gAct

        # 力控状态变量
        self._force_tolerance = 2.0          # 力控精度容忍度（±N），默认±2N
        self._max_force_control_time = 5.0   # 力控最大超时时间（s），防止无限等待
        self._force_stop_threshold = 30.0    # 力阈值停止默认值（N），超过则自动停止

        # reset and activate, must be execute befor first move
        if autoInit:
            self.initGripper()

    # 完整初始化流程
    def initGripper(self):
        if self.__gAct != GACT.GripperActivation or self.__gSta != GSTA.ActivationCompleted:
            self.resetGripper()
            self.activateGripper()
            while True:
                self.readGripperStatus()
                if self.__gSta == GSTA.ActivationCompleted: 
                    break
                time.sleep(DEFAULT_SYNC_TIME)

    # 激活夹爪
    def activateGripper(self):
        self.__rAct = RACT.Activate
        self.master.write_single_register(HandERegister.REQUEST_ACTION, self.__requestAction(), slave=self.DEFAULT_SLAVE_ID)


    def move(self, pos, speed, force, block=True):
        pos = 50 - pos

        self.__rGto   = RGTO.Go
        targetPos     = PosCheck(int(pos / self.POS_CONVERSION_RATIO))
        targetSpeed   = SpeedCheck(speed)
        targetForce   = ForceCheck(force)
        # print(targetPos)

        actionReg     = self.__requestAction()
        positionReg   = targetPos
        speedForceReg = targetSpeed << 8 | targetForce
        self.master.write_multi_registers(HandERegister.REQUEST_ACTION, [actionReg, positionReg, speedForceReg], slave=self.DEFAULT_SLAVE_ID)

        #while block:
            #self.readGripperStatus()
            #if self.__gObj != GOBJ.MovingAndNoObj:
                #break
            #if self.__gGto == GGTO.Stopped:
                #break
            #time.sleep(DEFAULT_SYNC_TIME)

    # 紧急停止运动
    def stop(self):
        self.__rGto = RGTO.Stop
        self.master.write_single_register(HandERegister.REQUEST_ACTION, self.__requestAction(), slave=self.DEFAULT_SLAVE_ID)

    # 紧急释放
    def emergencyAutoRelease(self, releaseDirction:RARD=None):
        if releaseDirction is not None:
            self.__rArd = releaseDirction
        self.__rAtr = RATR.EAutoRelease
        self.master.write_single_register(HandERegister.REQUEST_ACTION, self.__requestAction(), slave=self.DEFAULT_SLAVE_ID)

    # 复位夹爪
    def resetGripper(self):
        self.__rAct = RACT.Deactivate
        self.master.write_single_register(HandERegister.REQUEST_ACTION, self.__requestAction(), slave=self.DEFAULT_SLAVE_ID)

    # 指令打包：将 __rAct/__rGto/__rAtr/__rArd 四个指令合并为 16 位寄存器值
    def __requestAction(self):
        value = 0x0000
        value = (self.__rAct << 0 + 8) | value
        value = (self.__rGto << 3 + 8) | value
        value = (self.__rAtr << 4 + 8) | value
        value = (self.__rArd << 5 + 8) | value
        return value

    # 读取已下发的指令
    def __readGripperAction(self):
        value = self.master.read_holding_registers(HandERegister.REQUEST_ACTION, slave=self.DEFAULT_SLAVE_ID)[0]
        action = HandERegister.getHighByteValue(value)
        self.__rAct = (action & 0b00000001)
        self.__rGto = (action & 0b00001000) >> 3
        self.__rAtr = (action & 0b00010000) >> 4
        self.__rArd = (action & 0b00100000) >> 5

    # 读取夹爪核心状态
    def readGripperStatus(self):
        value = self.master.read_holding_registers(HandERegister.GRIPPER_STATUS, slave=self.DEFAULT_SLAVE_ID)[0]
        state = HandERegister.getHighByteValue(value)
        self.__gAct = (state & 0b00000001)
        self.__gGto = (state & 0b00001000) >> 3
        self.__gSta = (state & 0b00110000) >> 4
        self.__gObj = (state & 0b11000000) >> 6
        return self.__gAct, self.__gGto, self.__gSta, self.__gObj

    # 设置力控精度容忍度
    def set_force_tolerance(self, tolerance: float):
        if 0.1 <= tolerance <= 10.0:
            self._force_tolerance = tolerance
        else:
            raise ValueError(f"容忍度必须在 0.1~10.0 N 之间（当前输入：{tolerance}）")

    # 设置力阈值停止阈值（默认50N）
    def set_force_stop_threshold(self, threshold: float):
        if self.FORCE_MIN <= threshold <= self.FORCE_MAX:
            self._force_stop_threshold = threshold
        else:
            raise ValueError(f"阈值必须在 {self.FORCE_MIN}~{self.FORCE_MAX} N 之间（当前输入：{threshold}）")

    # 核心力控接口：按目标力夹持（优先保证力值，而非位置）
    def move_with_force(self, target_force: float, speed: int=100, block: bool=True):
        if not (self.FORCE_MIN <= target_force <= self.FORCE_MAX):
            raise ValueError(f"目标力必须在 {self.FORCE_MIN}~{self.FORCE_MAX} N 之间（当前输入：{target_force}）")
        speed = SpeedCheck(speed)  # 限制速度范围

        target_force_reg = int((target_force - self.FORCE_MIN) * self.FORCE_TO_REG)
        target_force_reg = ForceCheck(target_force_reg)  # 确保不超出寄存器范围

        orintal_rGto = self.__rGto
        try:
            self.__rGto = RGTO.Go
            actionReg = self.__requestAction()
            positionReg = PosCheck(int(self.MIN_POS / self.POS_CONVERSION_RATIO))  # 目标位置：0mm（闭合）
            speedForceReg = speed << 8 | target_force_reg  # 速度（低字节）+ 力值（高字节）
            self.master.write_multi_registers(
                HandERegister.REQUEST_ACTION,
                [actionReg, positionReg, speedForceReg],
                slave=self.DEFAULT_SLAVE_ID
            )

            start_time = time.time()
            while True:
                current_force = self.ForceValue  # 实时读取实际力值
                current_pos = self.position      # 实时读取当前位置
                self.readGripperStatus()         # 实时读取夹爪状态

                # 退出条件1：力控达标（稳定100ms，避免抖动）
                if abs(current_force - target_force) <= self._force_tolerance:
                    time.sleep(0.1)
                    if abs(self.ForceValue - target_force) <= self._force_tolerance:
                        return True

                # 退出条件2：已闭合到最小位置（无法再施力，力控失败）
                if current_pos <= self.MIN_POS + 0.5:
                    return False

                # 退出条件3：力控超时（防止无限等待）
                if time.time() - start_time > self._max_force_control_time:
                    self.stop()
                    raise TimeoutError("力控超时，未能达到目标力值")

                # 退出条件4：触发力阈值停止（超过预设阈值）
                if current_force > self._force_stop_threshold:
                    self.stop()
                    raise ValueError(f"力控夹持超过安全阈值（{self._force_stop_threshold}N），已停止夹爪")

                # 退出条件5：夹爪故障
                if self.faultStatus != FaultStatus.NoFault:
                    self.stop()
                    raise RuntimeError(f"夹爪发生故障，故障码：{self.faultStatus}")

                time.sleep(DEFAULT_SYNC_TIME)
        finally:
            self.__rGto = orintal_rGto 

    # 计算夹爪当前夹持力（单位：牛顿）
    @property
    def ForceValue(self):
        # 通过电流与力的线性关系计算夹爪当前夹持力（假设它们之间呈现近似线性）
        # 力的范围： F_min = 20N, F_max = 185N
        # 电流的范围： I_min = 0, I_max = 255（非电流实际值，而是电机的读数值）
        # 计算公式： F = F_min + ((F_max-F_min) / (I_max - I_min)) * (I - I_min)
        current_value = self.current
        force = 20.0 + ((185.0 - 20.0) / (255.0 - 0.0)) * (current_value - 0.0)
        return force

    # 读取 CURRENT 寄存器的低字节，获取夹爪当前工作电流（用于判断负载情况）
    @property
    def current(self):
        value = self.master.read_holding_registers(HandERegister.CURRENT, slave=self.DEFAULT_SLAVE_ID)[0]
        return HandERegister.getLowByteValue(value)
        
    # 读取 POSITION 寄存器的高字节，转换为实际毫米数（当前夹爪实际位置）
    @property
    def position(self):
        value = self.master.read_holding_registers(HandERegister.POSITION, slave=self.DEFAULT_SLAVE_ID)[0]
        return HandERegister.getHighByteValue(value) * self.POS_CONVERSION_RATIO

    # 读取 FAULT_STATUS 寄存器的低字节，获取故障码（如过载、通信错误等）
    @property
    def faultStatus(self):
        value = self.master.read_holding_registers(HandERegister.FAULT_STATUS, slave=self.DEFAULT_SLAVE_ID)[0]
        return HandERegister.getLowByteValue(value)

    # 读取 ECHO_POSITION 寄存器的高字节，转换为实际毫米数（反馈之前下发的目标位置，用于校验指令是否被正确接收）
    @property
    def reqPosition(self):
        value = self.master.read_holding_registers(HandERegister.ECHO_POSITION, slave=self.DEFAULT_SLAVE_ID)[0]
        return HandERegister.getHighByteValue(value) * self.POS_CONVERSION_RATIO


