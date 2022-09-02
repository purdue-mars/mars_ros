from typing import List

import numpy as np
from geometry_msgs.msg import Pose, Quaternion, QuaternionStamped
from tf.transformations import (
    euler_from_matrix,
    quaternion_from_euler,
    quaternion_from_matrix,
    quaternion_matrix,
)

import ros_numpy

AXIS_X = np.array([1, 0, 0])
AXIS_Y = np.array([0, 1, 0])
AXIS_Z = np.array([0, 0, 1])

AXES = [AXIS_X, AXIS_Y, AXIS_Z]


def rot(vec: np.ndarray, angles: List[float]):
    r, p, y = angles
    s = np.sin
    c = np.cos
    R = np.array(
        [
            [
                c(y) * c(p),
                c(y) * s(p) * s(r) - s(y) * c(r),
                c(y) * s(p) * c(r) + s(y) * s(r),
            ],
            [
                s(y) * c(p),
                s(y) * s(p) * s(r) + c(y) * c(r),
                s(y) * s(p) * c(r) - c(y) * s(r),
            ],
            [-s(p), c(p) * s(r), c(p) * c(r)],
        ]
    )
    return np.matmul(R, vec)


def tf_mat_to_quat_stamped(tf_mat, frame_id):
    q = quaternion_from_matrix(tf_mat)
    q = q / np.linalg.norm(q)
    ori = QuaternionStamped()
    ori.header.frame_id = frame_id
    ori.quaternion = ros_numpy.msgify(Quaternion, q)
    return ori


def normalize(x):
    return x / np.linalg.norm(x)


def rot_from_a_to_b_preserve_axis(
    a: np.ndarray, b: np.ndarray, axis_to_preserve: np.ndarray
):
    """Gets rotation between vectors without changing direction of preserved axis"""
    comp_vecs = [a.copy() for a in AXES if not np.array_equal(axis_to_preserve, a)]
    ax1, ax2 = comp_vecs
    comp_v1_ax1 = comp_a_along_b(b, ax1)
    comp_v1_ax2 = comp_a_along_b(b, ax2)

    proj_vec = comp_v1_ax1 + comp_v1_ax2
    return rot_from_a_to_b(a, proj_vec)


def comp_a_along_b(a, b):
    return (np.dot(a, b) / np.dot(b, b)) * b


def rotate_mat_from_euler(angles, rot_mat):
    euler = euler_from_matrix(rot_mat)
    euler = np.array(euler) + np.array(angles)
    quat = quaternion_from_euler(*euler)
    quat = quat / np.linalg.norm(quat)
    mat = quaternion_matrix(quat)
    return mat


def vec_to_hom_mat(vec: np.ndarray):
    T = np.identity(4)
    X = np.ones(4)
    X[:3] = vec
    T[:, 3] = X
    return T


def rot_from_a_to_b(a: np.ndarray, b: np.ndarray):
    cross_1_2 = np.cross(a, b)
    skew_symm_cross_1_2 = np.array(
        [
            [0, -cross_1_2[2], cross_1_2[1]],
            [cross_1_2[2], 0, -cross_1_2[0]],
            [-cross_1_2[1], cross_1_2[0], 0],
        ]
    )
    cos = np.dot(a, b)
    R = (
        np.identity(3)
        + skew_symm_cross_1_2
        + np.dot(skew_symm_cross_1_2, skew_symm_cross_1_2) * 1 / (1 + cos + 1e-15)
    )
    T = np.identity(4)
    T[:3, :3] = R
    return T
