"""Unit tests for the PN guidance law (pin the math; mirror the point-mass)."""
import math

import numpy as np

from mas_pn_guidance.pn_law import (
    proportional_navigation, limit_norm, unit, command_to_dict,
)

N = 3.0
A_MAX = 6.0


def test_pn_accel_is_perpendicular_to_los():
    # Target offset laterally and crossing -> nonzero LOS rate -> PN turns.
    own_p = np.array([0.0, 0.0, 0.0]); own_v = np.array([10.0, 0.0, 0.0])
    tgt_p = np.array([50.0, 10.0, 0.0]); tgt_v = np.array([0.0, 5.0, 0.0])
    cmd = proportional_navigation(own_p, own_v, tgt_p, tgt_v, N, A_MAX)
    los = unit(tgt_p - own_p)
    # PN command must be perpendicular to the line of sight.
    assert abs(float(np.dot(cmd.acceleration_mps2, los))) < 1e-9
    assert np.linalg.norm(cmd.acceleration_mps2) > 0.1


def test_zero_los_rate_gives_zero_command():
    # Pure head-on closing (target directly ahead, closing along LOS) -> Ω = 0.
    own_p = np.array([0.0, 0.0, 0.0]); own_v = np.array([10.0, 0.0, 0.0])
    tgt_p = np.array([50.0, 0.0, 0.0]); tgt_v = np.array([-5.0, 0.0, 0.0])
    cmd = proportional_navigation(own_p, own_v, tgt_p, tgt_v, N, A_MAX)
    assert np.linalg.norm(cmd.acceleration_mps2) < 1e-9
    assert cmd.los_rate_radps < 1e-9
    assert cmd.closing_speed_mps == 15.0   # -dot(v_rel, los) = -(-15) = 15


def test_closing_speed_sign():
    own_p = np.zeros(3); own_v = np.array([5.0, 0.0, 0.0])
    tgt_p = np.array([20.0, 0.0, 0.0])
    # target receding faster than we close -> negative closing speed
    cmd = proportional_navigation(own_p, own_v, tgt_p, np.array([10.0, 0, 0]), N, A_MAX)
    assert cmd.closing_speed_mps < 0.0
    # opening geometry -> max(Vc,0)=0 -> no command
    assert np.linalg.norm(cmd.acceleration_mps2) < 1e-9


def test_accel_limit_saturation():
    # Fast closing (+20 m/s) AND fast crossing at close range -> huge raw PN.
    own_p = np.zeros(3); own_v = np.array([20.0, 0.0, 0.0])
    tgt_p = np.array([10.0, 0.0, 0.0]); tgt_v = np.array([0.0, 30.0, 0.0])
    cmd = proportional_navigation(own_p, own_v, tgt_p, tgt_v, N, A_MAX)
    assert cmd.closing_speed_mps > 0.0
    assert cmd.saturated
    assert np.linalg.norm(cmd.acceleration_mps2) <= A_MAX + 1e-6
    assert np.linalg.norm(cmd.raw_acceleration_mps2) > A_MAX


def test_limit_norm_and_unit():
    assert np.allclose(limit_norm(np.array([3.0, 4.0, 0.0]), 5.0), [3, 4, 0])
    assert np.allclose(limit_norm(np.array([3.0, 4.0, 0.0]), 2.5), [1.5, 2.0, 0.0])
    assert np.allclose(unit(np.zeros(3)), [1, 0, 0])           # fallback
    assert np.allclose(np.linalg.norm(unit(np.array([1.0, 1, 1]))), 1.0)


def test_higher_N_commands_more_turn():
    own_p = np.zeros(3); own_v = np.array([10.0, 0.0, 0.0])
    tgt_p = np.array([50.0, 10.0, 0.0]); tgt_v = np.array([0.0, 5.0, 0.0])
    a3 = np.linalg.norm(proportional_navigation(own_p, own_v, tgt_p, tgt_v, 3.0, 1e9).acceleration_mps2)
    a5 = np.linalg.norm(proportional_navigation(own_p, own_v, tgt_p, tgt_v, 5.0, 1e9).acceleration_mps2)
    assert a5 > a3
    # command_to_dict exposes the diagnostics used by the node
    d = command_to_dict(proportional_navigation(own_p, own_v, tgt_p, tgt_v, 3.0, A_MAX))
    assert set(d) >= {"closing_speed_mps", "los_rate_radps", "saturated", "range_est_m"}


def test_constant_speed_pursuit_converges():
    """End-to-end: v_max pursuer + PN integration intercepts a crossing target
    (mirrors the point-mass propagation: v_init = v_max*û, v += a*dt, |v|<=v_max)."""
    dt = 0.02; v_max = 11.0; a_max = 8.0
    own_p = np.array([0.0, 0.0, 0.0]); tgt_p = np.array([70.0, 0.0, 0.0])
    own_v = v_max * unit(tgt_p - own_p)           # pursuit init
    tgt_v = np.array([0.0, 6.0, 0.0])             # crossing target
    min_range = 1e9
    for _ in range(2000):
        cmd = proportional_navigation(own_p, own_v, tgt_p, tgt_v, 3.0, a_max)
        own_v = limit_norm(own_v + cmd.acceleration_mps2 * dt, v_max)
        own_p = own_p + own_v * dt
        tgt_p = tgt_p + tgt_v * dt
        min_range = min(min_range, float(np.linalg.norm(tgt_p - own_p)))
    assert min_range < 1.0, f"expected intercept, min_range={min_range:.2f}"
