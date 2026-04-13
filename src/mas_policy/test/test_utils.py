"""Tests for mas_policy.utils math functions."""

import numpy as np
import pytest

from mas_policy.utils import (
    wrap_to_pi,
    euler_xyz_from_quat,
    quat_rotate,
    gimbal_ray_direction_world,
    quat_multiply,
)


class TestWrapToPi:
    def test_basic_values(self):
        angles = np.array([0.0, np.pi, -np.pi, 2 * np.pi, -3 * np.pi])
        result = wrap_to_pi(angles)
        expected = np.array([0.0, -np.pi, -np.pi, 0.0, -np.pi])
        np.testing.assert_allclose(result, expected, atol=1e-10)

    def test_array(self):
        angles = np.array([0.5, 3.5, -3.5, 7.0])
        result = wrap_to_pi(angles)
        assert np.all(result >= -np.pi)
        assert np.all(result <= np.pi)


class TestEulerXyzFromQuat:
    def test_identity(self):
        q_identity = np.array([1.0, 0.0, 0.0, 0.0])  # wxyz
        roll, pitch, yaw = euler_xyz_from_quat(q_identity)
        np.testing.assert_allclose([roll, pitch, yaw], [0, 0, 0], atol=1e-10)

    def test_90deg_yaw(self):
        # 90 deg rotation about Z: q = [cos(45), 0, 0, sin(45)] in wxyz
        angle = np.pi / 2
        q = np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)])
        roll, pitch, yaw = euler_xyz_from_quat(q)
        np.testing.assert_allclose(roll, 0.0, atol=1e-10)
        np.testing.assert_allclose(pitch, 0.0, atol=1e-10)
        np.testing.assert_allclose(yaw, np.pi / 2, atol=1e-10)


class TestQuatRotate:
    def test_identity(self):
        q = np.array([1.0, 0.0, 0.0, 0.0])
        v = np.array([1.0, 2.0, 3.0])
        result = quat_rotate(q, v)
        np.testing.assert_allclose(result, v, atol=1e-10)

    def test_90deg_z(self):
        # 90 deg about Z: +X should become +Y
        angle = np.pi / 2
        q = np.array([np.cos(angle / 2), 0.0, 0.0, np.sin(angle / 2)])
        v = np.array([1.0, 0.0, 0.0])
        result = quat_rotate(q, v)
        np.testing.assert_allclose(result, [0.0, 1.0, 0.0], atol=1e-10)


class TestGimbalRayDirectionWorld:
    def test_forward(self):
        # yaw=0, pitch=0, identity quat → +X direction
        ray = gimbal_ray_direction_world(
            np.array(0.0), np.array(0.0),
            np.array([1.0, 0.0, 0.0, 0.0]),
        )
        np.testing.assert_allclose(ray, [1.0, 0.0, 0.0], atol=1e-10)


class TestQuatMultiply:
    def test_identity(self):
        q1 = np.array([1.0, 0.0, 0.0, 0.0])
        q2 = np.array([0.7071, 0.0, 0.7071, 0.0])
        result = quat_multiply(q1, q2)
        np.testing.assert_allclose(result, q2, atol=1e-4)

    def test_inverse(self):
        # q * q_conj = identity
        q = np.array([0.5, 0.5, 0.5, 0.5])
        q_conj = np.array([0.5, -0.5, -0.5, -0.5])
        result = quat_multiply(q, q_conj)
        np.testing.assert_allclose(result, [1.0, 0.0, 0.0, 0.0], atol=1e-10)
