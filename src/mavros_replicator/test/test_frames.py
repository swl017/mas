"""Sanity checks for the frame conversions. Pure math; no ROS deps."""
import numpy as np
import pytest

from mavros_replicator import frames


def test_R_W_is_involutory():
    assert np.allclose(frames.R_W_NED_TO_ENU @ frames.R_W_NED_TO_ENU, np.eye(3))


def test_R_B_is_involutory():
    assert np.allclose(frames.R_B_FRD_TO_FLU @ frames.R_B_FRD_TO_FLU, np.eye(3))


def test_R_W_is_proper_rotation():
    assert np.isclose(np.linalg.det(frames.R_W_NED_TO_ENU), 1.0)


def test_R_B_is_proper_rotation():
    assert np.isclose(np.linalg.det(frames.R_B_FRD_TO_FLU), 1.0)


def test_position_ned_to_enu_axes():
    assert np.allclose(frames.position_ned_to_enu([1, 0, 0]), [0, 1, 0])  # NED north → ENU north
    assert np.allclose(frames.position_ned_to_enu([0, 1, 0]), [1, 0, 0])  # NED east  → ENU east
    assert np.allclose(frames.position_ned_to_enu([0, 0, 1]), [0, 0, -1])  # NED down  → ENU -up


def test_body_frd_to_flu_axes():
    assert np.allclose(frames.vector_body_frd_to_flu([1, 0, 0]), [1, 0, 0])  # FRD fwd  → FLU fwd
    assert np.allclose(frames.vector_body_frd_to_flu([0, 1, 0]), [0, -1, 0])  # FRD right → FLU -left
    assert np.allclose(frames.vector_body_frd_to_flu([0, 0, 1]), [0, 0, -1])  # FRD down  → FLU -up


def test_attitude_identity_px4_to_ros():
    """PX4 identity attitude (level, body-X aligned with world-X-NED) → ROS quat
    representing body-FLU=ENU world axes mapping. Body forward (FRD x = north_NED)
    maps to body forward (FLU x = north → in ENU = +y). So q_ROS represents a
    rotation from FLU body to ENU world that sends FLU x → ENU y.
    """
    q_ros = frames.attitude_px4_to_ros_xyzw([1.0, 0.0, 0.0, 0.0])  # PX4 wxyz identity
    R = frames.quat_to_matrix_xyzw(q_ros)
    # Body forward (FLU +x) should rotate to world ENU +y (since PX4 identity = facing north;
    # north in NED is +x_ned, which is +y_enu).
    body_fwd = np.array([1.0, 0.0, 0.0])
    world = R @ body_fwd
    assert np.allclose(world, [0.0, 1.0, 0.0]), f"got {world}"
    # Body up (FLU +z) should rotate to world ENU +z (vehicle level → up stays up).
    body_up = np.array([0.0, 0.0, 1.0])
    world = R @ body_up
    assert np.allclose(world, [0.0, 0.0, 1.0]), f"got {world}"


def test_attitude_yaw90_east():
    """PX4 yaw +90° (NED) = vehicle nose pointing east. In ENU that means nose along +x_enu."""
    # PX4 quaternion for yaw+90 about NED-Z (down): w=cos(45), z=sin(45)
    c, s = np.cos(np.pi / 4), np.sin(np.pi / 4)
    q_px4_wxyz = [c, 0.0, 0.0, s]
    q_ros = frames.attitude_px4_to_ros_xyzw(q_px4_wxyz)
    R = frames.quat_to_matrix_xyzw(q_ros)
    body_fwd = np.array([1.0, 0.0, 0.0])  # FLU forward
    world = R @ body_fwd
    assert np.allclose(world, [1.0, 0.0, 0.0], atol=1e-6), f"got {world}"


def test_pose_covariance_diagonal_swap():
    pos_var = [1.0, 4.0, 9.0]
    ori_var = [0.01, 0.04, 0.09]
    cov = frames.pose_covariance_ned_to_enu(pos_var, ori_var)
    M = np.array(cov).reshape(6, 6)
    assert M[0, 0] == 4.0  # x_enu = y_ned var
    assert M[1, 1] == 1.0  # y_enu = x_ned var
    assert M[2, 2] == 9.0  # z unchanged
    assert M[3, 3] == 0.04
    assert M[4, 4] == 0.01
    assert M[5, 5] == 0.09


def test_velocity_setpoint_round_trip():
    # Command 1 m/s east, 0.5 rad/s left (CCW from above) in MAVROS conventions.
    v_enu = [1.0, 0.0, 0.0]
    yawspeed_flu = 0.5
    v_ned, yawspeed_ned = frames.velocity_setpoint_enu_flu_to_ned(v_enu, yawspeed_flu)
    # East in ENU (+x) is east in NED, which is +y_ned.
    assert np.allclose(v_ned, [0.0, 1.0, 0.0])
    # CCW from above (FLU +z) is CW about NED +z (down), which is *negative* yaw rate in NED.
    assert np.isclose(yawspeed_ned, -0.5)
