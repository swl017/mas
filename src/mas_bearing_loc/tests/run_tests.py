#!/usr/bin/env python3
"""Standalone test runner for mas_bearing_loc.

The math modules are pure-numpy and don't need Isaac Sim's AppLauncher; this
runner reuses the same TestResults reporting pattern as the IsaacLab tests.

Run with:
    python3 tests/run_tests.py
    python3 tests/run_tests.py --test-verbose
"""
from __future__ import annotations

import argparse
import sys
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np

# Allow `from mas_bearing_loc import ...` when running in-place.
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from mas_bearing_loc.camera_model import (  # noqa: E402
    CameraIntrinsics,
    R_C_B_ZERO,
    gimbal_R_c_b,
    interaction_matrix,
    project_point,
)
from mas_bearing_loc.dc_ekf import DCEKF, DCEKFConfig  # noqa: E402
from mas_bearing_loc.direct_projection_ekf import (  # noqa: E402
    DirectProjectionEKF,
    DirectProjectionEKFConfig,
    bearing_jacobian_from_delta,
)
from mas_bearing_loc.imu_buffer import RingBufferIMU, RingBufferSnapshot  # noqa: E402
from mas_bearing_loc.quaternion import (  # noqa: E402
    quat_mul,
    quat_normalize,
    quat_to_rot,
    rot_to_quat,
    small_angle_quat,
)


class TestResults:
    def __init__(self):
        self.passed = []
        self.failed = []
        self.errors = []

    def add_pass(self, name):
        self.passed.append(name)
        print(f"  PASS  {name}")

    def add_fail(self, name, err):
        self.failed.append((name, err))
        print(f"  FAIL  {name}")
        for line in str(err).split('\n')[:5]:
            print(f"        {line}")

    def add_error(self, name, err):
        self.errors.append((name, err))
        print(f"  ERROR {name}")
        for line in str(err).split('\n')[:8]:
            print(f"        {line}")

    def summary(self):
        total = len(self.passed) + len(self.failed) + len(self.errors)
        print("\n" + "=" * 78)
        print("TEST SUMMARY")
        print("=" * 78)
        print(f"Total:  {total}")
        print(f"Passed: {len(self.passed)}")
        print(f"Failed: {len(self.failed)}")
        print(f"Errors: {len(self.errors)}")
        return not (self.failed or self.errors)


# ============================================================================
#  Quaternion tests
# ============================================================================

def test_quaternion(r: TestResults):
    print("\n--- Quaternion ---")

    try:
        q = np.array([1.0, 0.0, 0.0, 0.0])
        R = quat_to_rot(q)
        assert np.allclose(R, np.eye(3), atol=1e-9)
        r.add_pass("identity quaternion -> I")
    except Exception as e:
        r.add_fail("identity quaternion -> I", traceback.format_exc())

    try:
        # 90 deg about z: x->y, y->-x
        ang = np.pi / 2
        q = np.array([np.cos(ang / 2), 0.0, 0.0, np.sin(ang / 2)])
        R = quat_to_rot(q)
        v_x = np.array([1.0, 0.0, 0.0])
        v_rot = R @ v_x
        assert np.allclose(v_rot, [0.0, 1.0, 0.0], atol=1e-9)
        r.add_pass("90deg z-rotation: x -> y")
    except Exception as e:
        r.add_fail("90deg z-rotation: x -> y", traceback.format_exc())

    try:
        # quat compose: rotating twice by 45deg about z = once by 90deg
        ang = np.pi / 4
        q45 = np.array([np.cos(ang / 2), 0.0, 0.0, np.sin(ang / 2)])
        q90 = quat_normalize(quat_mul(q45, q45))
        v_rot = quat_to_rot(q90) @ np.array([1.0, 0.0, 0.0])
        assert np.allclose(v_rot, [0.0, 1.0, 0.0], atol=1e-9)
        r.add_pass("quat compose: 45+45 -> 90 about z")
    except Exception as e:
        r.add_fail("quat compose: 45+45 -> 90 about z", traceback.format_exc())

    try:
        omega_dt = np.array([0.0, 0.0, np.pi / 2])  # 90deg about z
        q = small_angle_quat(omega_dt)
        v_rot = quat_to_rot(q) @ np.array([1.0, 0.0, 0.0])
        assert np.allclose(v_rot, [0.0, 1.0, 0.0], atol=1e-9), v_rot
        r.add_pass("small_angle_quat exact at 90deg")
    except Exception as e:
        r.add_fail("small_angle_quat exact at 90deg", traceback.format_exc())

    try:
        # round-trip rot <-> quat for a few random rotations
        rng = np.random.default_rng(0)
        for _ in range(10):
            omega = rng.normal(size=3) * 0.5
            q = small_angle_quat(omega)
            R = quat_to_rot(q)
            q_back = rot_to_quat(R)
            R_back = quat_to_rot(q_back)
            assert np.allclose(R, R_back, atol=1e-9)
        r.add_pass("rot<->quat round-trip on random rotations")
    except Exception as e:
        r.add_fail("rot<->quat round-trip on random rotations", traceback.format_exc())


# ============================================================================
#  Camera model tests
# ============================================================================

def test_camera_model(r: TestResults):
    print("\n--- Camera model ---")

    try:
        intr = CameraIntrinsics(fx=500.0, fy=500.0, cx=320.0, cy=240.0,
                                width=640, height=480)
        x, y = intr.normalize(320.0, 240.0)
        assert abs(x) < 1e-12 and abs(y) < 1e-12
        x, y = intr.normalize(820.0, 240.0)  # 500 px right of center
        assert abs(x - 1.0) < 1e-12 and abs(y) < 1e-12
        # zoom should shrink normalized coords
        x, y = intr.normalize(820.0, 240.0, zoom=2.0)
        assert abs(x - 0.5) < 1e-12
        r.add_pass("CameraIntrinsics.normalize w/ zoom")
    except Exception as e:
        r.add_fail("CameraIntrinsics.normalize w/ zoom", traceback.format_exc())

    try:
        # With zero gimbal angles, camera Z-forward should map to body X-forward
        R_c_b = gimbal_R_c_b(np.zeros(3))
        v_cam_z = np.array([0.0, 0.0, 1.0])
        v_body = R_c_b @ v_cam_z
        assert np.allclose(v_body, [1.0, 0.0, 0.0], atol=1e-9), v_body
        # cam Y-down -> body Z-down
        v_body = R_c_b @ np.array([0.0, 1.0, 0.0])
        assert np.allclose(v_body, [0.0, 0.0, -1.0], atol=1e-9), v_body
        r.add_pass("R_c_b zero-gimbal axis convention (CF->BF)")
    except Exception as e:
        r.add_fail("R_c_b zero-gimbal axis convention (CF->BF)", traceback.format_exc())

    try:
        # ZXY convention sanity (Rz·Rx·Ry): yaw 90deg about body +Z
        # rotates optical axis from body +X to body +Y.
        R_c_b = gimbal_R_c_b(np.array([0.0, 0.0, np.pi / 2]))  # roll=0, pitch=0, yaw=90
        optical = R_c_b @ np.array([0.0, 0.0, 1.0])
        assert np.allclose(optical, [0.0, 1.0, 0.0], atol=1e-9), optical
        r.add_pass("Gimbal yaw=+90 deg: optical axis -> body +Y")
    except Exception as e:
        r.add_fail("Gimbal yaw=+90 deg: optical axis -> body +Y", traceback.format_exc())

    try:
        # Gimbal pitch=-45deg (Ry, positive=down per the los_rate_controller comment):
        # With roll=yaw=0, the gimbal frame is the identity, so pitch about body +Y
        # tilts the body-+X axis by -45deg about body +Y.
        # Ry(pitch=-pi/4) on [1,0,0]: (cos*1, 0, -sin*1) = (cos(-pi/4), 0, -sin(-pi/4))
        #                                                = (sqrt(2)/2, 0, +sqrt(2)/2)  i.e. up
        R_c_b = gimbal_R_c_b(np.array([0.0, -np.pi / 4, 0.0]))
        optical = R_c_b @ np.array([0.0, 0.0, 1.0])
        # Optical axis tilts up (+Z) and forward (+X)
        s = np.sqrt(2.0) / 2.0
        assert np.allclose(optical, [s, 0.0, s], atol=1e-9), optical
        r.add_pass("Gimbal pitch=-45 deg: optical axis tilts up & forward")
    except Exception as e:
        r.add_fail("Gimbal pitch=-45 deg: optical axis tilts up & forward",
                   traceback.format_exc())

    try:
        # Aircraft hovering at origin with identity attitude, gimbal zero.
        # Target at world (10, 0, 0): straight ahead.  Image projection should be (0,0).
        R_b_w = np.eye(3)
        R_c_b = R_C_B_ZERO
        p_t = np.array([10.0, 0.0, 0.0])
        p_o = np.array([0.0, 0.0, 0.0])
        proj = project_point(p_t, p_o, R_b_w, R_c_b)
        assert abs(proj[0]) < 1e-9 and abs(proj[1]) < 1e-9
        assert proj[2] > 0  # in front of camera
        r.add_pass("Project (10,0,0) at identity -> center, +depth")
    except Exception as e:
        r.add_fail("Project (10,0,0) at identity -> center, +depth", traceback.format_exc())

    try:
        # Target at (10, 0, 1): one meter above forward target.
        # In CF (X-right, Y-down, Z-forward): world Y-up = -Y_cam, world X-fwd = +Z_cam.
        # So p_cam = (0, -1, 10), p_bar = (0, -0.1).
        p_t = np.array([10.0, 0.0, 1.0])
        proj = project_point(p_t, np.zeros(3), np.eye(3), R_C_B_ZERO)
        assert abs(proj[0]) < 1e-9
        assert abs(proj[1] + 0.1) < 1e-9, proj
        r.add_pass("Project (10,0,1) -> world-up maps to image-up (-y)")
    except Exception as e:
        r.add_fail("Project (10,0,1) -> world-up maps to image-up", traceback.format_exc())

    try:
        # interaction matrix shape and sign on Z-translation (pure approach)
        L = interaction_matrix(np.array([0.1, 0.2]), depth=10.0)
        assert L.shape == (2, 6)
        # Pure +Z translation: p_bar grows (point sweeps outward in image)
        rate = L @ np.array([0, 0, 1.0, 0, 0, 0])
        # rate[0] = x/Z * 1, rate[1] = y/Z * 1, both positive
        assert rate[0] > 0 and rate[1] > 0
        r.add_pass("L_s pure-Z translation expands image feature")
    except Exception as e:
        r.add_fail("L_s pure-Z translation expands image feature", traceback.format_exc())

    try:
        # Camera-body offset for IrisGimbal3: camera at body (0, -0.10, 0.12).
        # Aircraft at world origin, target at world (10, 0, 0.12).
        # Without offset: target appears 0.12 m up of camera, projected to image up
        #   (p_bar y < 0).
        # With offset: camera is at world (0, -0.10, 0.12), and target is 10 m
        #   forward and 0 m up of CAMERA — projects to image center (0, 0).
        # (The 10 cm right-shift becomes 0.01 in p_bar x: small but verifiable.)
        t_cam = np.array([0.0, -0.10, 0.12])
        R_b_w = np.eye(3)
        proj_off = project_point(np.array([10.0, 0.0, 0.12]), np.zeros(3),
                                 R_b_w, R_C_B_ZERO, t_cam_in_body=t_cam)
        # Vector camera->target in world = (10, 0.10, 0); in camera frame
        # (rotated by R_C_B_ZERO^T): X-right = -Y_body = -0.10, Y-down = -Z_body = 0,
        # Z-forward = X_body = 10.  So p_bar = (-0.01, 0).
        assert abs(proj_off[0] + 0.01) < 1e-9, proj_off
        assert abs(proj_off[1]) < 1e-9, proj_off
        # Sanity: without the offset, same target projects differently
        proj_no = project_point(np.array([10.0, 0.0, 0.12]), np.zeros(3),
                                R_b_w, R_C_B_ZERO)
        assert abs(proj_no[0]) < 1e-9 and proj_no[1] < -1e-3, proj_no
        r.add_pass("Camera offset shifts projection (IrisGimbal3 mount)")
    except Exception as e:
        r.add_fail("Camera offset shifts projection (IrisGimbal3 mount)",
                   traceback.format_exc())


# ============================================================================
#  Buffer tests
# ============================================================================

def test_buffers(r: TestResults):
    print("\n--- Buffers ---")

    try:
        buf = RingBufferIMU(window_sec=0.5, expected_rate_hz=200.0)
        for i in range(50):
            buf.push(t=0.01 * i, omega=np.zeros(3), accel=np.zeros(3))
        win = buf.samples_in(0.1, 0.2)
        assert len(win) == 10  # samples at t=0.11..0.20
        r.add_pass("IMU buffer slice")
    except Exception as e:
        r.add_fail("IMU buffer slice", traceback.format_exc())

    try:
        snap = RingBufferSnapshot()
        for i in range(20):
            snap.push(0.01 * i, np.zeros(18), np.eye(17))
        idx, s = snap.find_at_or_before(0.105)
        assert abs(s.t - 0.10) < 1e-9, s.t
        snap.prune_before(0.05)
        assert len(snap) == 15
        r.add_pass("Snapshot buffer find_at_or_before + prune")
    except Exception as e:
        r.add_fail("Snapshot buffer find_at_or_before + prune", traceback.format_exc())


# ============================================================================
#  EKF tests
# ============================================================================

def test_ekf_basic(r: TestResults):
    print("\n--- EKF basic ---")

    try:
        ekf = DCEKF(DCEKFConfig(init_range=20.0, t_cam_in_body=(0.0, 0.0, 0.0)))
        bearing = np.array([1.0, 0.0, 0.0])  # straight east, world ENU
        ekf.initialize_from_bearing(t=0.0, bearing_world=bearing,
                                    q_wxyz=np.array([1.0, 0.0, 0.0, 0.0]))
        # target should be 20m east of aircraft
        p_t_world = ekf.target_position_world(np.zeros(3))
        assert np.allclose(p_t_world, [20.0, 0.0, 0.0], atol=1e-9), p_t_world
        r.add_pass("init: bearing east + 20m -> target world (20,0,0)")
    except Exception as e:
        r.add_fail("init: bearing east + 20m -> target world (20,0,0)",
                   traceback.format_exc())

    try:
        # Stationary aircraft, no IMU motion: predict for 0.1 s should leave
        # p_r unchanged except for gravity-driven drift.  Since the EKF assumes
        # the *aircraft* is falling under gravity (specific force = 0 → inertial
        # accel = g_world = -9.81 z), v_r drifts downward and p_r drifts down.
        ekf = DCEKF(DCEKFConfig(init_range=10.0, t_cam_in_body=(0.0, 0.0, 0.0)))
        ekf.initialize_from_bearing(0.0, np.array([1.0, 0.0, 0.0]),
                                    np.array([1.0, 0.0, 0.0, 0.0]))
        # Provide IMU samples consistent with hover (specific force = +9.81 z)
        # so inertial accel = 0.
        accel_hover = np.array([0.0, 0.0, 9.81])
        R_c_b = R_C_B_ZERO
        for i in range(20):
            t = 0.005 * (i + 1)
            ekf.predict_imu(t, omega=np.zeros(3), accel=accel_hover, R_c_b=R_c_b)
        p_r = ekf.relative_position
        v_r = ekf.relative_velocity
        assert np.allclose(p_r, [-10.0, 0.0, 0.0], atol=1e-3), p_r
        assert np.allclose(v_r, [0.0, 0.0, 0.0], atol=1e-3), v_r
        r.add_pass("hover IMU keeps p_r, v_r stationary")
    except Exception as e:
        r.add_fail("hover IMU keeps p_r, v_r stationary", traceback.format_exc())

    try:
        # Inject a perfect bearing measurement and verify p_bar update reduces
        # innovation.  Run for a tiny window so the snapshot finds a match.
        ekf = DCEKF(DCEKFConfig(init_range=10.0, t_cam_in_body=(0.0, 0.0, 0.0)))
        ekf.initialize_from_bearing(0.0, np.array([1.0, 0.0, 0.0]),
                                    np.array([1.0, 0.0, 0.0, 0.0]))
        accel_hover = np.array([0.0, 0.0, 9.81])
        for i in range(5):
            ekf.predict_imu(0.005 * (i + 1), np.zeros(3), accel_hover, R_C_B_ZERO)
        # Force a measurement that is 0.05 below the predicted feature
        target_pbar = ekf.p_bar + np.array([0.0, 0.05])
        diag = ekf.update_bearing(0.020, target_pbar, R_C_B_ZERO)
        assert diag is not None, "update should not have returned None"
        # After update p_bar should move toward measurement (Kalman gain < 1)
        post_innov = target_pbar - ekf.p_bar
        assert np.linalg.norm(post_innov) < 0.05, post_innov
        r.add_pass("update_bearing reduces p_bar innovation")
    except Exception as e:
        r.add_fail("update_bearing reduces p_bar innovation",
                   traceback.format_exc())

    try:
        # Sanity check: stationary aircraft + correct bearing measurement
        # repeated many times should converge p_r toward true range.
        ekf = DCEKF(DCEKFConfig(init_range=5.0,    # bad initial guess (true=20m)
                                sigma_pix=0.5,
                                t_cam_in_body=(0.0, 0.0, 0.0)))
        ekf.initialize_from_bearing(0.0, np.array([1.0, 0.0, 0.0]),
                                    np.array([1.0, 0.0, 0.0, 0.0]))
        # True target at world (20, 0, 0).  Aircraft at origin, identity attitude.
        true_p_t = np.array([20.0, 0.0, 0.0])
        accel_hover = np.array([0.0, 0.0, 9.81])
        for k in range(200):
            t = 0.01 * (k + 1)
            ekf.predict_imu(t, np.zeros(3), accel_hover, R_C_B_ZERO)
            if k % 4 == 0:  # 25 Hz image rate
                proj = project_point(true_p_t, np.zeros(3), np.eye(3), R_C_B_ZERO)
                ekf.update_bearing(t, proj[:2], R_C_B_ZERO)
        # In bearing-only on a stationary observer, range is unobservable -
        # the filter cannot find true range from bearing alone.  But the
        # bearing direction should be correct.
        p_r = ekf.relative_position
        p_t_est = -p_r  # = p_target - p_aircraft, aircraft at origin
        bearing_est = p_t_est / np.linalg.norm(p_t_est)
        bearing_true = true_p_t / np.linalg.norm(true_p_t)
        assert np.allclose(bearing_est, bearing_true, atol=1e-2), \
            f"bearing diverged: est={bearing_est}, true={bearing_true}"
        r.add_pass("bearing direction recovered from repeated measurements")
    except Exception as e:
        r.add_fail("bearing direction recovered from repeated measurements",
                   traceback.format_exc())


def test_ekf_with_camera_offset(r: TestResults):
    print("\n--- EKF with IrisGimbal3 camera offset ---")

    try:
        # Aircraft at world (0,0,0), identity attitude, gimbal zero.
        # Camera optical center is at world (0, -0.10, 0.12).
        # Place target at world (20, -0.10, 0.12) so it projects exactly to
        # the image center (bearing along camera +Z = body +X).
        from mas_bearing_loc.camera_model import project_point
        true_p_t = np.array([20.0, -0.10, 0.12])
        # Seed the EKF using that bearing.  Bearing from camera to target
        # in world = (20, 0, 0) / 20 = (1, 0, 0).
        ekf = DCEKF(DCEKFConfig(init_range=20.0,
                                t_cam_in_body=(0.0, -0.10, 0.12)))
        ekf.initialize_from_bearing(0.0, np.array([1.0, 0.0, 0.0]),
                                    np.array([1.0, 0.0, 0.0, 0.0]),
                                    range_guess=20.0)
        # Aircraft is at world origin, so target_position_world should equal true_p_t.
        p_t_est = ekf.target_position_world(np.zeros(3))
        assert np.allclose(p_t_est, true_p_t, atol=1e-9), p_t_est
        r.add_pass("Init w/ offset reconstructs true target world position")
    except Exception as e:
        r.add_fail("Init w/ offset reconstructs true target world position",
                   traceback.format_exc())

    try:
        # With non-zero camera offset, repeated measurements still recover the
        # bearing from camera to target.
        from mas_bearing_loc.camera_model import project_point
        t_cb = np.array([0.0, -0.10, 0.12])
        ekf = DCEKF(DCEKFConfig(init_range=5.0,
                                sigma_pix=0.5,
                                t_cam_in_body=tuple(t_cb)))
        ekf.initialize_from_bearing(0.0, np.array([1.0, 0.0, 0.0]),
                                    np.array([1.0, 0.0, 0.0, 0.0]))
        true_p_t = np.array([20.0, 0.0, 0.0])
        accel_hover = np.array([0.0, 0.0, 9.81])
        for k in range(200):
            t = 0.01 * (k + 1)
            ekf.predict_imu(t, np.zeros(3), accel_hover, R_C_B_ZERO)
            if k % 4 == 0:
                proj = project_point(true_p_t, np.zeros(3), np.eye(3),
                                     R_C_B_ZERO, t_cam_in_body=t_cb)
                ekf.update_bearing(t, proj[:2], R_C_B_ZERO)
        # Recovered camera→target bearing should match the true one.
        # cam world position = aircraft + R_b_w @ t_cb = (0, -0.10, 0.12).
        # True bearing from camera to target = (20, 0.10, -0.12)/norm.
        p_cam_world = np.zeros(3) + t_cb  # R_b_w = I
        true_bearing = (true_p_t - p_cam_world) / np.linalg.norm(true_p_t - p_cam_world)
        p_t_est = ekf.target_position_world(np.zeros(3))
        est_bearing = (p_t_est - p_cam_world) / np.linalg.norm(p_t_est - p_cam_world)
        assert np.allclose(est_bearing, true_bearing, atol=2e-2), \
            f"bearing diverged with offset: est={est_bearing}, true={true_bearing}"
        r.add_pass("Bearing direction recovered w/ camera offset")
    except Exception as e:
        r.add_fail("Bearing direction recovered w/ camera offset",
                   traceback.format_exc())


def test_direct_projection_ekf(r: TestResults):
    print("\n--- Direct-projection EKF ---")

    def _bearing(p_t, p_o):
        d = p_t - p_o
        return d / np.linalg.norm(d)

    try:
        # Jacobian of h(δ)=δ/‖δ‖ is the tangent projector / range.  The
        # line-of-sight direction is in its null space (range unobservable from
        # a single bearing).
        delta = np.array([3.0, 4.0, 0.0])  # range = 5
        u, dh, rad = bearing_jacobian_from_delta(delta)
        assert abs(rad - 5.0) < 1e-9, rad
        assert np.allclose(u, [0.6, 0.8, 0.0], atol=1e-9), u
        # P = (I - uuᵀ) is the projector; dh = P / range.  P·u = 0.
        P = dh * rad
        assert np.allclose(P @ u, np.zeros(3), atol=1e-9), P @ u
        assert np.allclose(P @ P, P, atol=1e-9)  # idempotent
        r.add_pass("bearing_jacobian_from_delta: tangent projector / range")
    except Exception:
        r.add_fail("bearing_jacobian_from_delta: tangent projector / range",
                   traceback.format_exc())

    try:
        # Init: observer at origin, target 20 m east.  q = p_obs - p_target
        # = -20·bearing, and target_position(p_obs) reconstructs the truth.
        ekf = DirectProjectionEKF(DirectProjectionEKFConfig(init_range=20.0))
        ekf.initialize_from_bearing(0.0, np.array([1.0, 0.0, 0.0]))
        assert np.allclose(ekf.rel_position, [-20.0, 0.0, 0.0], atol=1e-9), ekf.rel_position
        p_t = ekf.target_position(np.zeros(3))
        assert np.allclose(p_t, [20.0, 0.0, 0.0], atol=1e-9), p_t
        r.add_pass("init: bearing east + 20m -> relative q, target (20,0,0)")
    except Exception:
        r.add_fail("init: bearing east + 20m -> relative q, target (20,0,0)",
                   traceback.format_exc())

    try:
        # Predict dead-reckons the relative geometry with observer accel as a
        # control input.  Stationary target, observer accelerating +x at 2 m/s²
        # from rest: after 1 s the observer has moved +1 m in x, so the relative
        # vector q = p_obs - p_target gains +1 m in x.
        ekf = DirectProjectionEKF(DirectProjectionEKFConfig(init_range=20.0))
        ekf.initialize_from_bearing(0.0, np.array([1.0, 0.0, 0.0]),
                                    v_observer_world=np.zeros(3))
        q0 = ekf.rel_position.copy()
        a_obs = np.array([2.0, 0.0, 0.0])
        for k in range(100):
            ekf.predict(0.01 * (k + 1), a_obs)
        dq = ekf.rel_position - q0
        assert np.allclose(dq, [1.0, 0.0, 0.0], atol=1e-6), dq
        assert np.allclose(ekf.rel_velocity, [2.0, 0.0, 0.0], atol=1e-6), ekf.rel_velocity
        r.add_pass("predict: observer accel control dead-reckons q")
    except Exception:
        r.add_fail("predict: observer accel control dead-reckons q",
                   traceback.format_exc())

    try:
        # Observability: an orbiting (maneuvering) observer recovers full 3D
        # range for a constant-velocity target.
        rng = np.random.default_rng(7)
        dt = 0.05
        p_t0 = np.array([20.0, 5.0, 2.0])
        v_t = np.array([0.5, -0.3, 0.0])
        Ro, w = 15.0, 0.2

        def obs(t):
            p = np.array([Ro * np.cos(w * t), Ro * np.sin(w * t), 1.0])
            v = np.array([-Ro * w * np.sin(w * t), Ro * w * np.cos(w * t), 0.0])
            a = np.array([-Ro * w * w * np.cos(w * t), -Ro * w * w * np.sin(w * t), 0.0])
            return p, v, a

        sig = 0.005
        ekf = DirectProjectionEKF(DirectProjectionEKFConfig(
            init_range=10.0, sigma_bearing=sig, sigma_target_acc=0.3))
        errs = []
        for k in range(1200):
            t = k * dt
            p_t = p_t0 + v_t * t
            p_o, v_o, a_o = obs(t)
            b = _bearing(p_t, p_o) + rng.normal(0, sig, 3)
            b /= np.linalg.norm(b)
            if not ekf.initialized:
                ekf.initialize_from_bearing(t, b, v_o)
                continue
            ekf.update_bearing(t, b, a_o, sig)
            if t > 5.0:
                errs.append(np.linalg.norm(ekf.target_position(p_o) - p_t))
        med = float(np.median(errs))
        assert med < 2.0, f"median pos err too high: {med:.2f} m"
        assert np.isfinite(ekf.P).all()
        r.add_pass(f"orbiting observer recovers range (median {med:.2f} m)")
    except Exception:
        r.add_fail("orbiting observer recovers range", traceback.format_exc())

    try:
        # Unobservable: a stationary observer cannot recover range, but the
        # bearing direction must stay correct (classic bearing-only result).
        ekf = DirectProjectionEKF(DirectProjectionEKFConfig(
            init_range=5.0, sigma_bearing=0.002, sigma_target_acc=0.05))
        p_t = np.array([20.0, 0.0, 0.0])
        p_o = np.zeros(3)
        for k in range(400):
            t = 0.05 * k
            b = _bearing(p_t, p_o)
            if not ekf.initialized:
                ekf.initialize_from_bearing(t, b, np.zeros(3))
                continue
            ekf.update_bearing(t, b, np.zeros(3), 0.002)
        b_est = ekf.predicted_bearing_world
        assert np.allclose(b_est, [1.0, 0.0, 0.0], atol=1e-2), b_est
        r.add_pass("stationary observer keeps bearing direction (range unobservable)")
    except Exception:
        r.add_fail("stationary observer keeps bearing direction",
                   traceback.format_exc())


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--test-verbose', action='store_true')
    args = parser.parse_args()
    _ = args

    print("=" * 78)
    print("mas_bearing_loc TEST SUITE")
    print("=" * 78)
    print(f"numpy:   {np.__version__}")
    print(f"started: {datetime.now():%Y-%m-%d %H:%M:%S}")

    results = TestResults()
    try:
        test_quaternion(results)
        test_camera_model(results)
        test_buffers(results)
        test_ekf_basic(results)
        test_ekf_with_camera_offset(results)
        test_direct_projection_ekf(results)
    except Exception:
        results.add_error("suite", traceback.format_exc())

    ok = results.summary()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
