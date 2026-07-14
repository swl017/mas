"""Offline unit tests for the ticket-019 mock-cooperative pure logic (no ROS)."""
import math

import numpy as np
import pytest

from mas_coop_mock.core import AlphaBetaVel, viewing_pose, DelayBuffer


# --- AlphaBetaVel (the velocity linchpin) ----------------------------------
def test_alphabeta_recovers_constant_velocity():
    v_true = np.array([2.0, -1.5, 0.0])
    x0 = np.array([10.0, 5.0, 30.0])
    f = AlphaBetaVel(alpha=0.6, beta=0.2)
    t = 0.0
    for _ in range(200):                     # 20 s @ 10 Hz (fused-pose rate)
        z = x0 + v_true * t
        x, v = f.update(t, z)
        t += 0.1
    assert np.allclose(v, v_true, atol=1e-2)
    assert np.allclose(x, x0 + v_true * (t - 0.1), atol=1e-2)


def test_alphabeta_seed_and_out_of_order():
    f = AlphaBetaVel()
    x, v = f.update(1.0, [0.0, 0.0, 0.0])
    assert np.allclose(v, 0.0)               # first sample -> zero velocity
    x2, v2 = f.update(0.5, [9.0, 9.0, 9.0])  # older stamp -> ignored
    assert np.allclose(x2, x) and np.allclose(v2, v)


def test_alphabeta_vmax_clamps():
    f = AlphaBetaVel(alpha=0.9, beta=0.9, v_max=1.0)
    f.update(0.0, [0.0, 0.0, 0.0])
    _, v = f.update(0.1, [100.0, 0.0, 0.0])  # huge jump
    assert np.linalg.norm(v) <= 1.0 + 1e-9


def test_alphabeta_noise_tracks_mean_velocity():
    rng = np.random.default_rng(0)
    v_true = np.array([1.0, 0.0, 0.0])
    f = AlphaBetaVel(alpha=0.4, beta=0.05)
    t = 0.0
    vs = []
    for _ in range(600):
        z = np.array([0.0, 0.0, 30.0]) + v_true * t + rng.normal(0, 0.05, 3)
        _, v = f.update(t, z)
        vs.append(v.copy())
        t += 0.1
    assert np.allclose(np.mean(vs[-100:], axis=0), v_true, atol=0.1)


# --- viewing_pose (observer parallax geometry) -----------------------------
def _parallax_deg(p_int, p_tgt, p_obs):
    a = np.asarray(p_int[:2]) - np.asarray(p_tgt[:2])
    b = np.asarray(p_obs[:2]) - np.asarray(p_tgt[:2])
    cos = float(a @ b) / (np.linalg.norm(a) * np.linalg.norm(b))
    return math.degrees(math.acos(np.clip(cos, -1, 1)))


@pytest.mark.parametrize("off", [0.0, 12.0, 40.0, 75.0, 90.0])
def test_viewing_pose_parallax_and_standoff(off):
    p_int = np.array([0.0, -50.0, 25.0])
    p_tgt = np.array([0.0, 0.0, 30.0])
    p_obs = viewing_pose(p_int, p_tgt, offset_deg=off, standoff_m=45.0)
    assert abs(_parallax_deg(p_int, p_tgt, p_obs) - off) < 1e-6
    assert abs(np.linalg.norm(p_obs[:2] - p_tgt[:2]) - 45.0) < 1e-6   # standoff


def test_viewing_pose_height_override():
    p_obs = viewing_pose([0, -50, 25], [0, 0, 30], 45.0, 45.0, height_m=28.0)
    assert p_obs[2] == pytest.approx(28.0)


# --- DelayBuffer (the AoI knob) --------------------------------------------
def test_delay_releases_after_tau():
    b = DelayBuffer(tau_s=0.3)
    for k in range(3):
        b.push(0.1 * k, f"m{k}")             # rx at 0.0, 0.1, 0.2
    assert b.pop_ready(0.25) == []            # oldest age 0.25 < 0.3
    assert b.pop_ready(0.35) == ["m0"]        # age 0.35 >= 0.3
    assert b.pop_ready(0.55) == ["m1", "m2"]  # ages 0.45, 0.35 -> both, in order


def test_delay_zero_is_passthrough():
    b = DelayBuffer(tau_s=0.0)
    b.push(1.0, "a")
    assert b.pop_ready(1.0) == ["a"]


def test_delay_rejects_negative_tau():
    with pytest.raises(ValueError):
        DelayBuffer(tau_s=-0.1)
