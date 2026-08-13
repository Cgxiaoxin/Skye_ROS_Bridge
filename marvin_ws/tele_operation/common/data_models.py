from pydantic import BaseModel, Field, PrivateAttr
import warnings
import threading
import numpy as np
from collections import deque
from enum import Enum, auto, IntEnum
from typing import List, final, Deque

class BimanualRobotStates(BaseModel):
    leftRobotTCP: List[float] = [0.0] * 7  # (7) (x, y, z, qw, qx, qy, qz)
    rightRobotTCP: List[float] = [0.0] * 7  # (7) (x, y, z, qw, qx, qy, qz)
    leftGripperState: List[float] = [0.0] * 2  # (2) (width, force)
    rightGripperState: List[float] = [0.0] * 2  # (2) (width, force)
    leftRobotJoints: List[float] = [0.0] * 7  # (7) (j1, j2, j3, j4, j5, j6) # this is add for test vr ik
    rightRobotJoints: List[float] = [0.0] * 7

class MoveGripperRequest(BaseModel):
    width: float = 0.05
    velocity: float = 10.0
    force_limit: float = 5.0

class TargetTCPRequest(BaseModel):
    target_tcp: List[float]  # (7) (x, y, z, qw, qx, qy, qz)

class TargetJointsRequest(BaseModel):
    target_joints: List[float]  # (7) (j1, j2, j3, j4, j5, j6, j7)

class Vitai_SensorMessage_Double(BaseModel):
    timestamp: float
    # left robot
    leftRobotTCP: np.ndarray = Field(default_factory=lambda: np.zeros((6, ), dtype=np.float32))  # (6) (x, y, z, r, p, y)
    leftRobotJoints: np.ndarray = Field(default_factory=lambda: np.zeros((7, ), dtype=np.float32))  # (7) joint angles # add this 
    leftRobotGripperState: np.ndarray = Field(default_factory=lambda: np.zeros((2, ), dtype=np.float32))  # (2) gripper (width, force)
    leftRobotAction: np.ndarray = Field(default_factory=lambda: np.zeros((6, ), dtype=np.float32))  # (6) (x, y, z, r, p, y)
    leftRobotGripperAction: np.ndarray = Field(default_factory=lambda: np.zeros((1, ), dtype=np.float32))  # (2) gripper (width, force)
    # right robot
    rightRobotTCP: np.ndarray = Field(default_factory=lambda: np.zeros((6, ), dtype=np.float32))  # (6) (x, y, z, r, p, y)
    rightRobotJoints: np.ndarray = Field(default_factory=lambda: np.zeros((7, ), dtype=np.float32))  # (7) joint angles # add this 
    rightRobotGripperState: np.ndarray = Field(default_factory=lambda: np.zeros((2, ), dtype=np.float32))  # (2) gripper (width, force)
    rightRobotAction: np.ndarray = Field(default_factory=lambda: np.zeros((6, ), dtype=np.float32))  # (6) (x, y, z, r, p, y)
    rightRobotGripperAction: np.ndarray = Field(default_factory=lambda: np.zeros((1, ), dtype=np.float32))  # (2) gripper (width, force)
    # camera
    externalCameraRGB: np.ndarray = Field(default_factory=lambda: np.zeros((48, 64, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    leftWristCameraRGB: np.ndarray = Field(default_factory=lambda: np.zeros((48, 64, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    rightWristCameraRGB: np.ndarray = Field(default_factory=lambda: np.zeros((48, 64, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    # left tactile
    leftGripperCameraRGB1: np.ndarray = Field(
        default_factory=lambda: np.zeros((24, 32, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    leftGripperCameraMarkerOffset1: np.ndarray = Field(
        default_factory=lambda: np.zeros((63, 3), dtype=np.float32))  # (num_markers, 3)(x, y, z)
    leftGripperCameraRGB2: np.ndarray = Field(
        default_factory=lambda: np.zeros((24, 32, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    leftGripperCameraMarkerOffset2: np.ndarray = Field(
        default_factory=lambda: np.zeros((63, 3), dtype=np.float32))  # (num_markers, 2)(x, y,z)
    # right tactile
    rightGripperCameraRGB1: np.ndarray = Field(
        default_factory=lambda: np.zeros((24, 32, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    rightGripperCameraMarkerOffset1: np.ndarray = Field(
        default_factory=lambda: np.zeros((63, 3), dtype=np.float32))  # (num_markers, 3)(x, y, z)
    rightGripperCameraRGB2: np.ndarray = Field(
        default_factory=lambda: np.zeros((24, 32, 3), dtype=np.uint8))  # (H, W, 3) (r, g, b)
    rightGripperCameraMarkerOffset2: np.ndarray = Field(
        default_factory=lambda: np.zeros((63, 3), dtype=np.float32))  # (num_markers, 2)(x, y,z)

    class Config:
        arbitrary_types_allowed = True

class SensorMessageList(BaseModel):
    # 统一预警阈值，超过即报警
    WARNING_THRESHOLD: int = 2000

    _sensor_messages: Deque[Vitai_SensorMessage_Double] = PrivateAttr(default_factory=deque)
    _lock: threading.Lock = PrivateAttr(default_factory=threading.Lock)

    def append(self, msg: Vitai_SensorMessage_Double) -> None:
        with self._lock:
            self._sensor_messages.append(msg)
            self._check_warning()

    def _check_warning(self) -> None:
        count = len(self._sensor_messages)
        if count >= self.WARNING_THRESHOLD:
            warnings.warn(
                f"⚠️ 传感器数据已堆积 {count} 条，消费过慢，存在内存占用过高风险！",
                UserWarning,
                stacklevel=2
            )

    def get_all(self) -> list[Vitai_SensorMessage_Double]:
        with self._lock:
            return list(self._sensor_messages)

    def clear(self) -> None:
        with self._lock:
            self._sensor_messages.clear()

    def __len__(self) -> int:
        with self._lock:
            return len(self._sensor_messages)

    @property
    def count(self) -> int:
        return len(self)
    
## Robot control modes
class ControlMode(IntEnum):
    IDLE = 0        # 下使能/空闲状态
    POSITION = 1    # 位置模式
    PVT = 2         # PVT模式
    TORQUE = 3      # 扭矩模式/阻抗模式
    DRAG = 4        # 拖动模式

class ImpedanceType(IntEnum):
    JOINT = 1       # 关节阻抗
    CARTESIAN = 2   # 笛卡尔阻抗
    FORCE = 3       # 力控

