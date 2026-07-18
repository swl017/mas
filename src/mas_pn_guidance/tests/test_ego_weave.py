"""Offline unit tests for the ego active-sensing weave (ticket 019 B1)."""
import math

import numpy as np

from mas_pn_guidance.ego_weave import ego_weave_velocity

LOS = np.array([1.0, 0.0, 0.0])          # horizontal LOS along +x
PEAK_T = 0.25 / 0.2                       # t where sin(2*pi*0.2*t)=1 (freq 0.2 Hz)


def test_default_off_amp():
    v = ego_weave_velocity(LOS, PEAK_T, 0.0, 0.2, 0.0, 50.0)
    assert np.allclose(v, 0.0)


def test_default_off_freq():
    v = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.0, 0.0, 50.0)
    assert np.allclose(v, 0.0)


def test_perpendicular_to_los():
    v = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)
    assert abs(float(np.dot(v, LOS))) < 1e-9


def test_horizontal():
    # LOS with a vertical component -> weave stays in the horizontal plane.
    los = np.array([1.0, 0.0, 0.5])
    v = ego_weave_velocity(los, PEAK_T, 2.0, 0.2, 0.0, 50.0)
    assert abs(v[2]) < 1e-9


def test_amplitude_at_peak():
    v = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)  # sin=1, no taper
    assert abs(np.linalg.norm(v) - 2.0) < 1e-9


def test_sign_flips_half_period():
    v1 = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)
    v2 = ego_weave_velocity(LOS, PEAK_T + 0.5 / 0.2, 2.0, 0.2, 0.0, 50.0)  # +half period
    assert np.allclose(v1, -v2, atol=1e-9)


def test_taper_linear_inside_range():
    # taper_range 20 m; at range 10 m the weave is halved, at >=20 m it is full.
    full = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.2, 20.0, 40.0)
    half = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.2, 20.0, 10.0)
    assert abs(np.linalg.norm(full) - 2.0) < 1e-9
    assert abs(np.linalg.norm(half) - 1.0) < 1e-9


def test_taper_zero_at_contact():
    v = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.2, 20.0, 0.0)
    assert np.allclose(v, 0.0)


def test_vertical_los_returns_zero():
    v = ego_weave_velocity(np.array([0.0, 0.0, 1.0]), PEAK_T, 2.0, 0.2, 0.0, 50.0)
    assert np.allclose(v, 0.0)


def test_direction_is_lateral_left_for_plus_x_los():
    # z_hat x x_hat = +y : the weave excites the cross-LOS (y) axis at the peak.
    v = ego_weave_velocity(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)
    assert v[1] > 1.9 and abs(v[0]) < 1e-9
