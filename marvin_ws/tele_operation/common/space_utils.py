import math
import numpy as np
from numba import jit
import transforms3d as t3d
from typing import Tuple, Union
from geometry_msgs.msg import Pose
import scipy.spatial.transform as st
from scipy.spatial.transform import Slerp
from scipy.spatial.transform import Rotation as R

def ros_pose_to_4x4matrix(pose: Pose) -> np.ndarray:
    # Convert ROS Pose message to 4x4 transformation matrix
    mat = np.eye(4)
    quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
    rot_mat = t3d.quaternions.quat2mat(quat)
    mat[:3, :3] = rot_mat
    mat[:3, 3] = np.array([pose.position.x, pose.position.y, pose.position.z])
    return mat

def ros_pose_to_6d_pose(pose: Pose) -> np.ndarray:
    # convert ROS Pose message to 6D pose (x, y, z, r, p, y)
    quat = np.array([pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z])
    euler = t3d.euler.quat2euler(quat, axes='szyx') # changed this
    trans = np.array([pose.position.x, pose.position.y, pose.position.z])
    return np.concatenate([trans, euler])

def pose_6d_to_pose_7d(pose: np.ndarray) -> np.ndarray:
    # convert 6D pose (x, y, z, r, p, y) to 7D pose (x, y, z, qw, qx, qy, qz)
    quat = t3d.euler.euler2quat(pose[3], pose[4], pose[5])
    return np.concatenate([pose[:3], quat])

def pose_6d_to_pose_7d_zyx(pose: np.ndarray) -> np.ndarray:
    # convert 6D pose (x, y, z, r, p, y) to 7D pose (x, y, z, qw, qx, qy, qz)
    quat = t3d.euler.euler2quat(pose[3], pose[4], pose[5],axes='szyx')
    return np.concatenate([pose[:3], quat])

def pose_7d_to_pose_6d(pose: np.ndarray) -> np.ndarray:
    # convert 7D pose (x, y, z, qw, qx, qy, qz) to 6D pose (x, y, z, r, p, y)
    quat = pose[3:]
    euler = t3d.euler.quat2euler(quat)
    return np.concatenate([pose[:3], euler])

def pose_7d_to_4x4matrix(pose: np.ndarray) -> np.ndarray:
    # convert 7D pose (x, y, z, qw, qx, qy, qz) to 4x4 transformation matrix
    mat = np.eye(4)
    mat[:3, :3] = t3d.quaternions.quat2mat(pose[3:])
    mat[:3, 3] = pose[:3]
    return mat

def pose_6d_to_4x4matrix(pose: np.ndarray) -> np.ndarray:
    # convert 6D pose (x, y, z, r, p, y) to 4x4 transformation matrix
    mat = np.eye(4)
    quat = t3d.euler.euler2quat(pose[3], pose[4], pose[5])
    mat[:3, :3] = t3d.quaternions.quat2mat(quat)
    mat[:3, 3] = pose[:3]
    return mat

def pose_6d_to_4x4matrix_zyx(pose: np.ndarray) -> np.ndarray:
    # convert 6D pose (x, y, z, r, p, y) to 4x4 transformation matrix
    mat = np.eye(4)
    quat = t3d.euler.euler2quat(pose[3], pose[4], pose[5], axes='szyx')
    mat[:3, :3] = t3d.quaternions.quat2mat(quat)
    mat[:3, 3] = pose[:3]
    return mat

def matrix4x4_to_pose_6d_zyx(mat: np.ndarray) -> np.ndarray:
    # convert 4x4 transformation matrix to 6D pose (x, y, z, r, p, y)
    quat = t3d.quaternions.mat2quat(mat[:3, :3])
    euler = t3d.euler.quat2euler(quat, axes='szyx')
    trans = mat[:3, 3]
    return np.concatenate([trans, euler])

def matrix4x4_to_pose_6d(mat: np.ndarray) -> np.ndarray:
    # convert 4x4 transformation matrix to 6D pose (x, y, z, r, p, y)
    quat = t3d.quaternions.mat2quat(mat[:3, :3])
    euler = t3d.euler.quat2euler(quat)
    trans = mat[:3, 3]
    return np.concatenate([trans, euler])
    
def matrix4x4_to_pose_7d(mat: np.ndarray) -> np.ndarray:
    # convert 4x4 transformation matrix to 7D pose (x, y, z, qw, qx, qy, qz)
    quat = t3d.quaternions.mat2quat(mat[:3, :3])
    trans = mat[:3, 3]
    return np.concatenate([trans, quat])

def ortho6d_to_rotation_matrix(ortho6d: np.ndarray) -> np.ndarray:
    """
    Compute rotation matrix from ortho6d representation
    """
    x_raw = ortho6d[:, 0:3]  # batch * 3
    y_raw = ortho6d[:, 3:6]  # batch * 3
    x = normalize_vector(x_raw)  # batch * 3
    z = np.cross(x, y_raw)  # batch * 3
    z = normalize_vector(z)  # batch * 3
    y = np.cross(z, x)  # batch * 3

    x = x[:, :, np.newaxis]
    y = y[:, :, np.newaxis]
    z = z[:, :, np.newaxis]

    matrix = np.concatenate((x, y, z), axis=2)  # batch * 3 * 3
    return matrix

def normalize_vector(v: np.ndarray) -> np.ndarray:
    """
    Normalize a vector (batch * 3)
    """
    v_mag = np.linalg.norm(v, axis=1, keepdims=True)  # batch * 1
    v_mag = np.maximum(v_mag, 1e-8)
    v = v / v_mag
    return v

def transform_point_cloud(pcd: np.ndarray, transform_matrix: np.ndarray) -> np.ndarray:
    """
    Transform a point cloud with 4x4 transform_matrix
    Parameters
    ----------
    pcd: (N, 6) or (N, 3) ndarray
    transform_matrix: (4, 4) ndarray
    """
    if pcd.shape[1] == 3: # (x, y, z)
        transformed_xyz = np.matmul(transform_matrix[:3, :3], pcd.T).T + transform_matrix[:3, 3]
        return transformed_xyz
    elif pcd.shape[1] == 6:  # (x, y, z, r, p, y)
        transformed_xyz = np.matmul(transform_matrix[:3, :3], pcd[:, :3].T).T + transform_matrix[:3, 3]
        return np.concatenate([transformed_xyz, pcd[:, 3:]], axis=1)
    else:
        raise NotImplementedError

def pose_6d_to_pose_9d(pose: np.ndarray) -> np.ndarray:
    """
    Convert 6D state to 9D state
    :param pose: np.ndarray (6,), (x, y, z, rx, ry, rz)
    :return: np.ndarray (9,), (x, y, z, rx1, rx2, rx3, ry1, ry2, ry3)
    """
    rot_6d = pose_6d_to_4x4matrix(pose)[:3, :2].T.flatten()
    return np.concatenate((pose[:3], rot_6d), axis=0)

def rot_matrix_to_rot_vec(rot_matrix: np.ndarray) -> np.ndarray:
    """
    Convert rotation matrix to rotation vector
    :param rot_matrix: np.ndarray (3, 3)
    :return: np.ndarray (3,)
    """
    rot_vec = R.from_matrix(rot_matrix).as_rotvec()
    return rot_vec


def pose_3d_9d_to_homo_matrix_batch(pose: np.ndarray) -> np.ndarray:
    """
    Convert 3D / 9D states to 4x4 matrix
    :param pose: np.ndarray (N, 9) or (N, 3)
    :return: np.ndarray (N, 4, 4)
    """
    assert pose.shape[1] in [3, 9], "pose should be (N, 3) or (N, 9)"
    mat = np.eye(4)[None, :, :].repeat(pose.shape[0], axis=0)
    mat[:, :3, 3] = pose[:, :3]
    if pose.shape[1] == 9:
        mat[:, :3, :3] = ortho6d_to_rotation_matrix(pose[:, 3:9])
    return mat

def homo_matrix_to_pose_9d_batch(mat: np.ndarray) -> np.ndarray:
    """
    Convert 4x4 matrix to 9D state
    :param mat: np.ndarray (N, 4, 4)
    :return: np.ndarray (N, 9)
    """
    assert mat.shape[1:] == (4, 4), "mat should be (N, 4, 4)"
    pose = np.zeros((mat.shape[0], 9))
    pose[:, :3] = mat[:, :3, 3]
    pose[:, 3:9] = mat[:, :3, :2].swapaxes(1, 2).reshape(mat.shape[0], -1)
    return pose
    
    
    
@jit
def smoothing_factor(t_e, cutoff):
    r = 2 * math.pi * cutoff * t_e
    return r / (r + 1)


@jit
def exponential_smoothing(a, x, x_prev):
    return a * x + (1 - a) * x_prev


def rotational_exponential_smoothing(a, x, x_prev):
    s = Slerp([0, 1], R.from_quat([x_prev, x]))
    x_hat = s(a)
    return x_hat.as_quat()


class OneEuroFilter:
    def __init__(
        self,
        min_cutoff=1.0,
        beta=0.0,
        d_cutoff=1.0,
    ):
        """Initialize the one euro filter for a 14-dimensional numpy array."""
        self.min_cutoff = float(min_cutoff)
        self.beta = float(beta)
        self.d_cutoff = float(d_cutoff)

        self.data_shape = None

        self.t_prev = None
        self.x_prev = None
        self.dx_prev = None

        self.smoothing_fn = exponential_smoothing

    def next(self, t, x, dx0=None):
        """Compute the filtered signal for a 14-dimensional numpy array."""
        if self.t_prev is None:
            self.data_shape = x.shape
            self.t_prev = float(t)
            self.x_prev = np.array(x, dtype=float)
            if dx0 is None:
                self.dx_prev = np.zeros_like(x)
            else:
                self.dx_prev = np.array(dx0, dtype=float)
            return x

        if x.shape != self.data_shape:
            raise ValueError("Unexpected data shape")

        t_e = t - self.t_prev

        # The filtered derivative of the signal
        a_d = smoothing_factor(t_e, self.d_cutoff)
        dx = (x - self.x_prev) / t_e
        dx_hat = self.smoothing_fn(a_d, dx, self.dx_prev)

        # The filtered signal
        cutoff = self.min_cutoff + self.beta * np.abs(dx_hat)
        a = smoothing_factor(t_e, cutoff)
        x_hat = self.smoothing_fn(a, x, self.x_prev)

        # Memorize the previous values
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t

        return x_hat


class LPRotationFilter:
    """https://github.com/Dingry/bunny_teleop_server/blob/main/bunny_teleop_server/utils/robot_utils.py"""

    def __init__(self, alpha):
        self.alpha = alpha
        self.is_init = False

        self.y = None

    def next(self, x: np.ndarray):
        assert x.shape == (4,)

        if not self.is_init:
            self.y = x
            self.is_init = True
            return self.y.copy()

        self.y = rotational_exponential_smoothing(self.alpha, x, self.y)

        return self.y.copy()

    def next_mat(self, x: np.ndarray):
        """take and return rotation matrix instead of quat"""
        assert x.shape == (3, 3) or x.shape == (4, 4)

        if x.shape == (4, 4):
            x = x[:3, :3]

        x = R.from_matrix(x).as_quat()
        next_x_quat = self.next(x)

        return R.from_quat(next_x_quat).as_matrix()

    def reset(self):
        self.y = None
        self.is_init = False

def print_green(x):
    return print("\033[92m {}\033[00m".format(x))


def print_yellow(x):
    return print("\033[93m {}\033[00m".format(x))