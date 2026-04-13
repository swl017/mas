"""Math utilities for policy deployment.

Ports key functions from Isaac Lab's isaaclab.utils.math and iris_ma6 environment
to work with NumPy for deployment without Isaac Lab dependencies.
"""

from __future__ import annotations

import numpy as np


def wrap_to_pi(angles: np.ndarray) -> np.ndarray:
    """Wrap angles to [-pi, pi]."""
    return (angles + np.pi) % (2 * np.pi) - np.pi


def euler_xyz_from_quat(quat: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert quaternion (wxyz) to Euler angles (roll, pitch, yaw).

    Matches isaaclab.utils.math.euler_xyz_from_quat convention.

    Args:
        quat: Quaternion array of shape (..., 4) in (w, x, y, z) order.

    Returns:
        Tuple of (roll, pitch, yaw) arrays, each shape (...,).
    """
    w, x, y, z = quat[..., 0], quat[..., 1], quat[..., 2], quat[..., 3]

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quat_rotate(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate vector by quaternion. Matches isaaclab quat_rotate (wxyz convention).

    Args:
        quat: Quaternion (..., 4) in (w, x, y, z) order.
        vec: Vector (..., 3).

    Returns:
        Rotated vector (..., 3).
    """
    q_w = quat[..., 0:1]
    q_vec = quat[..., 1:4]

    # t = 2 * cross(q_vec, vec)
    t = 2.0 * np.cross(q_vec, vec)
    # result = vec + q_w * t + cross(q_vec, t)
    return vec + q_w * t + np.cross(q_vec, t)


def quat_rotate_inverse(quat: np.ndarray, vec: np.ndarray) -> np.ndarray:
    """Rotate vector by the inverse of a quaternion.

    In Isaac Lab, quat_rotate_inverse(q, v) rotates v from body to world frame
    (i.e., applies the quaternion rotation, not its inverse — the naming is a legacy convention).

    For deployment: this function rotates body-frame vectors to world frame,
    matching the training environment behavior.

    Args:
        quat: Quaternion (..., 4) in (w, x, y, z) order.
        vec: Vector (..., 3) in body frame.

    Returns:
        Rotated vector (..., 3) in world frame.
    """
    # Isaac Lab's quat_rotate_inverse computes: q_conj * v * q
    # which is equivalent to rotating v by conjugate(q).
    # For unit quaternions, conj(q) rotates in the opposite direction of q.
    # But in Isaac Lab convention, body orientation q maps world→body,
    # so conj(q) maps body→world.
    q_w = quat[..., 0:1]
    q_vec = quat[..., 1:4]

    # Using conjugate: negate the vector part
    t = 2.0 * np.cross(-q_vec, vec)
    return vec + q_w * t + np.cross(-q_vec, t)


def gimbal_ray_direction_world(
    yaw_body: np.ndarray,
    pitch_body: np.ndarray,
    q_body: np.ndarray,
) -> np.ndarray:
    """Compute world-frame camera ray direction from body-frame gimbal angles.

    Direct port from iris_ma_env6_test.py:79-103.

    Args:
        yaw_body: (...,) body-frame gimbal yaw (rad), 0=forward.
        pitch_body: (...,) body-frame gimbal pitch (rad).
        q_body: (..., 4) body orientation quaternion (wxyz).

    Returns:
        (..., 3) unit direction vector in world frame.
    """
    cos_p = np.cos(pitch_body)
    dir_body = np.stack(
        [
            cos_p * np.cos(yaw_body),
            cos_p * np.sin(yaw_body),
            -np.sin(pitch_body),
        ],
        axis=-1,
    )
    return quat_rotate(q_body, dir_body)



def ned_to_enu_position(pos_ned: np.ndarray) -> np.ndarray:
    """Convert NED position to ENU. [N,E,D] -> [E,N,U]."""
    return np.array([pos_ned[1], pos_ned[0], -pos_ned[2]])


def ned_to_enu_velocity(vel_ned: np.ndarray) -> np.ndarray:
    """Convert NED velocity to ENU. [vN,vE,vD] -> [vE,vN,vU]."""
    return np.array([vel_ned[1], vel_ned[0], -vel_ned[2]])


def frd_to_flu_angular_velocity(ang_vel_frd: np.ndarray) -> np.ndarray:
    """Convert FRD angular velocity to FLU. [p,q,r]_FRD -> [p,-q,-r]_FLU."""
    return np.array([ang_vel_frd[0], -ang_vel_frd[1], -ang_vel_frd[2]])


def quat_ned_frd_to_enu_flu(quat_ned_frd: np.ndarray) -> np.ndarray:
    """Convert a NED-FRD quaternion to ENU-FLU convention.

    The rotation from NED to ENU is a 180-degree rotation about the
    axis bisecting North and East, which can be expressed as:
    q_NED_to_ENU = [0, 0.70711, 0.70711, 0] in (w,x,y,z)

    Args:
        quat_ned_frd: (4,) quaternion in NED-FRD (wxyz).

    Returns:
        (4,) quaternion in ENU-FLU (wxyz).
    """
    # Rotation from ENU to NED frame
    # q = cos(90)*1 + sin(90)*(0.70711*i + 0.70711*j)
    # In wxyz: [0, 0.70711, 0.70711, 0]
    q_frame = np.array([0.0, 0.70711, 0.70711, 0.0])

    # FRD to FLU is 180 deg about X: [0, 1, 0, 0] in wxyz
    q_body = np.array([0.0, 1.0, 0.0, 0.0])

    # q_enu_flu = q_frame * q_ned_frd * q_body
    q_temp = quat_multiply(q_frame, quat_ned_frd)
    return quat_multiply(q_temp, q_body)


def quat_multiply(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    """Multiply two quaternions in (w, x, y, z) convention.

    Args:
        q1: First quaternion (4,).
        q2: Second quaternion (4,).

    Returns:
        Product quaternion (4,).
    """
    w1, x1, y1, z1 = q1[0], q1[1], q1[2], q1[3]
    w2, x2, y2, z2 = q2[0], q2[1], q2[2], q2[3]

    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])
