"""Ego active-sensing weave — pure math, no ROS (ticket 019 B1).

A lone bearings-only interceptor is range-unobservable on a straight pursuit (the
classic bearings-only / TMA result); the only way a single agent gains range
observability is to maneuver its own line of sight. This module produces a lateral
sinusoidal velocity, perpendicular to the LOS in the horizontal plane, to ADD to the
PN commanded velocity — the deterministic "ego active-sensing" baseline (the
single-drone analog of the mock cooperative observer). It tapers to zero inside a
terminal range so the endgame homes cleanly.

Default-off: amp<=0 or freq<=0 returns a zero vector, so with the guidance
parameters at their defaults the interceptor behaves EXACTLY as before (the
oracle / passive-ego / cooperative arms are unaffected).
"""
from __future__ import annotations

import math

import numpy as np

_Z = np.array([0.0, 0.0, 1.0])


def ego_weave_velocity(n_hat, t_s, amp_mps, freq_hz, taper_range_m, range_m):
    """Lateral weave velocity (3-vector, m/s) to add to v_cmd.

    n_hat          interceptor->target LOS (any length; renormalized here)
    t_s            seconds since engage (phase base)
    amp_mps        peak lateral speed; <=0 disables (returns zeros)
    freq_hz        weave frequency; <=0 disables
    taper_range_m  range below which the weave ramps linearly to 0 (0 = no taper)
    range_m        current interceptor-target range
    """
    if amp_mps <= 0.0 or freq_hz <= 0.0:
        return np.zeros(3)
    n = np.asarray(n_hat, dtype=float)
    nn = float(np.linalg.norm(n))
    if nn < 1e-9:
        return np.zeros(3)
    n = n / nn
    lat = np.cross(_Z, n)                       # horizontal, perpendicular to the LOS
    ln = float(np.linalg.norm(lat))
    if ln < 1e-6:                               # LOS ~vertical -> no horizontal perp
        return np.zeros(3)
    lat = lat / ln
    taper = 1.0
    if taper_range_m > 0.0:
        taper = float(np.clip(range_m / taper_range_m, 0.0, 1.0))
    return amp_mps * taper * math.sin(2.0 * math.pi * freq_hz * t_s) * lat
