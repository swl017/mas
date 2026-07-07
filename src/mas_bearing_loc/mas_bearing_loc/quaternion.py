"""Quaternion utilities.

Convention: q = [w, x, y, z] (wxyz). Rotation R is "body to earth", i.e. for a
vector v_body, v_earth = R @ v_body.
"""
from __future__ import annotations

import numpy as np


def quat_normalize(q: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array([
        [1 - 2 * (yy + zz), 2 * (xy - wz),     2 * (xz + wy)],
        [2 * (xy + wz),     1 - 2 * (xx + zz), 2 * (yz - wx)],
        [2 * (xz - wy),     2 * (yz + wx),     1 - 2 * (xx + yy)],
    ])


def quat_mul(q1: np.ndarray, q2: np.ndarray) -> np.ndarray:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return np.array([
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    ])


def small_angle_quat(omega_dt: np.ndarray) -> np.ndarray:
    """δq for a small body-frame rotation vector ω·dt.

    Exact form: δq = [cos(|ω·dt|/2), sin(|ω·dt|/2) * ω/|ω|].
    Matches paper Eq. 28.
    """
    half = 0.5 * omega_dt
    angle = np.linalg.norm(half)
    if angle < 1e-9:
        return np.array([1.0, half[0], half[1], half[2]])
    s = np.sin(angle) / angle
    return np.array([np.cos(angle), s * half[0], s * half[1], s * half[2]])


def _rx(roll: float) -> np.ndarray:
    cr, sr = np.cos(roll), np.sin(roll)
    return np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])


def _ry(pitch: float) -> np.ndarray:
    cp, sp = np.cos(pitch), np.sin(pitch)
    return np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])


def _rz(yaw: float) -> np.ndarray:
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])


def rpy_zyx_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZYX intrinsic rotation: R = Rz(yaw) Ry(pitch) Rx(roll). Generic aviation RPY."""
    return _rz(yaw) @ _ry(pitch) @ _rx(roll)


def rpy_zxy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """ZXY intrinsic rotation: R = Rz(yaw) Rx(roll) Ry(pitch).

    This is the gimbal convention used by los_rate_controller.py — the published
    `gimbal_state_rpy_deg` is meant to be consumed as Rz(yaw)·Rx(roll)·Ry(pitch).
    """
    return _rz(yaw) @ _rx(roll) @ _ry(pitch)


# Backward-compat alias (default to ZYX for code that called the old name).
rpy_to_rot = rpy_zyx_to_rot


def rot_to_quat(R: np.ndarray) -> np.ndarray:
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2.0 * np.sqrt(1.0 + tr)
        return quat_normalize(np.array([
            0.25 * s,
            (R[2, 1] - R[1, 2]) / s,
            (R[0, 2] - R[2, 0]) / s,
            (R[1, 0] - R[0, 1]) / s,
        ]))
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        return quat_normalize(np.array([
            (R[2, 1] - R[1, 2]) / s, 0.25 * s,
            (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
        ]))
    if i == 1:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        return quat_normalize(np.array([
            (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
            0.25 * s, (R[1, 2] + R[2, 1]) / s,
        ]))
    s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
    return quat_normalize(np.array([
        (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
        (R[1, 2] + R[2, 1]) / s, 0.25 * s,
    ]))


def skew(v: np.ndarray) -> np.ndarray:
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ])
