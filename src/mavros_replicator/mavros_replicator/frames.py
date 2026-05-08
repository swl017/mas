"""Frame and quaternion conversions: PX4 (NED/FRD, wxyz) ↔ ROS (ENU/FLU, xyzw).

See [src/doc/mavros_replicator_spec.md](../../doc/mavros_replicator_spec.md) §5 for the
formal definitions. All matrices/quaternions defined here represent rotations
between proper right-handed coordinate frames; det(R)=+1 for both.

Conventions:
- Quaternions are xyzw (ROS convention); PX4 wire format is wxyz and is converted on entry.
- Vectors are np.ndarray of shape (3,).
- Covariances are returned as 36-element row-major lists (ROS Pose/TwistWithCovariance shape).
"""
from __future__ import annotations

import numpy as np

# World rotation: NED → ENU (180° about (1,1,0)/√2). Involutory: R_W @ R_W = I.
R_W_NED_TO_ENU = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
], dtype=np.float64)

# Body rotation: FRD → FLU (180° about X). Involutory.
R_B_FRD_TO_FLU = np.array([
    [1.0,  0.0,  0.0],
    [0.0, -1.0,  0.0],
    [0.0,  0.0, -1.0],
], dtype=np.float64)

_SQRT_HALF = float(np.sqrt(0.5))
# xyzw quaternion for R_W_NED_TO_ENU.
Q_W_NED_TO_ENU_XYZW = np.array([_SQRT_HALF, _SQRT_HALF, 0.0, 0.0], dtype=np.float64)
# xyzw quaternion for R_B_FRD_TO_FLU.
Q_B_FRD_TO_FLU_XYZW = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float64)


def quat_mul_xyzw(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=np.float64)


def quat_conj_xyzw(q: np.ndarray) -> np.ndarray:
    return np.array([-q[0], -q[1], -q[2], q[3]], dtype=np.float64)


def quat_to_matrix_xyzw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [    2 * (x * y + z * w), 1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [    2 * (x * z - y * w),     2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)


def position_ned_to_enu(p_ned) -> np.ndarray:
    return R_W_NED_TO_ENU @ np.asarray(p_ned, dtype=np.float64)


def vector_world_ned_to_enu(v_ned) -> np.ndarray:
    return R_W_NED_TO_ENU @ np.asarray(v_ned, dtype=np.float64)


def vector_world_enu_to_ned(v_enu) -> np.ndarray:
    # R_W is involutory.
    return R_W_NED_TO_ENU @ np.asarray(v_enu, dtype=np.float64)


def vector_body_frd_to_flu(v_frd) -> np.ndarray:
    return R_B_FRD_TO_FLU @ np.asarray(v_frd, dtype=np.float64)


def attitude_px4_to_ros_xyzw(q_wxyz) -> np.ndarray:
    """PX4 [w,x,y,z] (body-FRD → world-NED) → ROS xyzw (body-FLU → world-ENU).

    q_ENU_FLU = q_W · q_PX4 · q_B^conj
    """
    qw, qx, qy, qz = q_wxyz
    q_px4_xyzw = np.array([qx, qy, qz, qw], dtype=np.float64)
    return quat_mul_xyzw(
        quat_mul_xyzw(Q_W_NED_TO_ENU_XYZW, q_px4_xyzw),
        quat_conj_xyzw(Q_B_FRD_TO_FLU_XYZW),
    )


def _row_major_36(m: np.ndarray) -> list:
    return m.flatten().tolist()


def pose_covariance_ned_to_enu(pos_var_ned, ori_var_ned) -> list:
    """6×6 row-major covariance for [x,y,z,rx,ry,rz] in ENU/FLU.

    PX4 v1.15 only exposes diagonal-only variances. R_W is a 180° rotation about
    (1,1,0)/√2, which on a *diagonal* covariance simply swaps the x and y entries;
    z is unchanged because variance is invariant under sign flip. The same holds
    for orientation about the same world axes (roll↔pitch swap).
    """
    cov = np.zeros((6, 6), dtype=np.float64)
    cov[0, 0] = float(pos_var_ned[1])
    cov[1, 1] = float(pos_var_ned[0])
    cov[2, 2] = float(pos_var_ned[2])
    cov[3, 3] = float(ori_var_ned[1])
    cov[4, 4] = float(ori_var_ned[0])
    cov[5, 5] = float(ori_var_ned[2])
    return _row_major_36(cov)


def velocity_local_covariance_ned_to_enu(vel_var_ned) -> list:
    """6×6 covariance for TwistStamped (linear world-ENU, angular body-FLU).

    PX4 exposes diagonal linear variance only; angular variance is unknown → zeros.
    """
    cov = np.zeros((6, 6), dtype=np.float64)
    cov[0, 0] = float(vel_var_ned[1])
    cov[1, 1] = float(vel_var_ned[0])
    cov[2, 2] = float(vel_var_ned[2])
    return _row_major_36(cov)


def odom_twist_covariance(vel_var_world_ned, q_enu_flu_xyzw) -> list:
    """6×6 covariance for Odometry.twist (linear body-FLU, angular body-FLU).

    Linear: rotate the world-ENU diagonal variance into body-FLU using current attitude.
    Angular: zero (PX4 does not expose).
    """
    sigma_world = np.diag([
        float(vel_var_world_ned[1]),
        float(vel_var_world_ned[0]),
        float(vel_var_world_ned[2]),
    ])
    R_world_body = quat_to_matrix_xyzw(q_enu_flu_xyzw)  # body→world
    R_body_world = R_world_body.T
    sigma_body = R_body_world @ sigma_world @ R_body_world.T
    cov = np.zeros((6, 6), dtype=np.float64)
    cov[:3, :3] = sigma_body
    return _row_major_36(cov)


def velocity_setpoint_enu_flu_to_ned(linear_enu, yawspeed_flu) -> tuple:
    """Convert MAVROS-style velocity setpoint to PX4 TrajectorySetpoint frame.

    linear_enu: world ENU 3-vector → world NED 3-vector.
    yawspeed_flu: body-FLU Z rate → NED frame yawspeed (negated; FLU Z-up vs NED Z-down).
    """
    linear_ned = R_W_NED_TO_ENU @ np.asarray(linear_enu, dtype=np.float64)
    yawspeed_ned = -float(yawspeed_flu)
    return linear_ned, yawspeed_ned
