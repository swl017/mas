"""Offline unit tests for the active-sensing acceleration classes (ticket 023 S0).

Pure math (no ROS). Class 1 (oepn) is built here; opt_weave/fim_mpc land in S2 and
must raise until then.
"""
import math

import numpy as np
import pytest

from mas_pn_guidance.active_sensing import (
    ActiveSensingContext, active_sensing_accel, oepn_accel, schedule_accel,
    aopn_accel, dev_pursuit_accel,
    lateral_hat, taper_gain, IMPLEMENTED_CLASSES, ACTIVE_SENSING_CLASSES,
)
from mas_pn_guidance.pn_law import limit_norm

LOS = np.array([1.0, 0.0, 0.0])          # horizontal LOS along +x
PEAK_T = 0.25 / 0.2                       # t where sin(2*pi*0.2*t)=1 (freq 0.2 Hz)


def _ctx(n_hat=LOS, t_s=PEAK_T, range_m=50.0, range_rate_mps=0.0, v_cmd=None):
    kw = dict(n_hat=np.asarray(n_hat, float), t_s=t_s, range_m=range_m,
              range_rate_mps=range_rate_mps)
    if v_cmd is not None:
        kw["v_cmd"] = np.asarray(v_cmd, float)
    return ActiveSensingContext(**kw)


def _p(amp=2.0, freq=0.2, taper=0.0):
    return {"amp_mps2": amp, "freq_hz": freq, "taper_range_m": taper}


# ---- helpers --------------------------------------------------------------
def test_lateral_hat_perp_horizontal_unit():
    l = lateral_hat(np.array([1.0, 0.0, 0.5]))
    assert abs(np.linalg.norm(l) - 1.0) < 1e-9
    assert abs(l[2]) < 1e-9                      # horizontal
    assert abs(float(np.dot(l, [1.0, 0.0, 0.5]))) < 1e-9  # ⊥ LOS


def test_lateral_hat_vertical_los_zero():
    assert np.allclose(lateral_hat(np.array([0.0, 0.0, 1.0])), 0.0)


def test_taper_gain():
    assert taper_gain(50.0, 0.0) == 1.0          # disabled -> always 1
    assert taper_gain(40.0, 20.0) == 1.0         # beyond range -> full
    assert abs(taper_gain(10.0, 20.0) - 0.5) < 1e-12
    assert taper_gain(0.0, 20.0) == 0.0


# ---- Class 1 OEPN accel ---------------------------------------------------
def test_oepn_default_off_amp():
    assert np.allclose(oepn_accel(LOS, PEAK_T, 0.0, 0.2, 0.0, 50.0), 0.0)


def test_oepn_default_off_freq():
    assert np.allclose(oepn_accel(LOS, PEAK_T, 2.0, 0.0, 0.0, 50.0), 0.0)


def test_oepn_perpendicular_and_horizontal():
    a = oepn_accel(np.array([1.0, 0.0, 0.5]), PEAK_T, 2.0, 0.2, 0.0, 50.0)
    assert abs(float(np.dot(a, [1.0, 0.0, 0.5]))) < 1e-9
    assert abs(a[2]) < 1e-9


def test_oepn_amplitude_at_peak_is_accel():
    a = oepn_accel(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)   # sin=1, no taper
    assert abs(np.linalg.norm(a) - 2.0) < 1e-9         # m/s^2 magnitude


def test_oepn_sign_flips_half_period():
    a1 = oepn_accel(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)
    a2 = oepn_accel(LOS, PEAK_T + 0.5 / 0.2, 2.0, 0.2, 0.0, 50.0)
    assert np.allclose(a1, -a2, atol=1e-9)


def test_oepn_taper_linear_and_contact():
    full = oepn_accel(LOS, PEAK_T, 2.0, 0.2, 20.0, 40.0)
    half = oepn_accel(LOS, PEAK_T, 2.0, 0.2, 20.0, 10.0)
    zero = oepn_accel(LOS, PEAK_T, 2.0, 0.2, 20.0, 0.0)
    assert abs(np.linalg.norm(full) - 2.0) < 1e-9
    assert abs(np.linalg.norm(half) - 1.0) < 1e-9
    assert np.allclose(zero, 0.0)


def test_oepn_direction_z_cross_x_is_plus_y():
    a = oepn_accel(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)   # z_hat x x_hat = +y
    assert a[1] > 1.9 and abs(a[0]) < 1e-9


def test_oepn_vertical_los_returns_zero():
    assert np.allclose(oepn_accel(np.array([0.0, 0.0, 1.0]), PEAK_T, 2.0, 0.2, 0.0, 50.0), 0.0)


# ---- dispatcher -----------------------------------------------------------
def test_dispatch_none_is_zero():
    assert np.allclose(active_sensing_accel("none", _p(), _ctx()), 0.0)


def test_dispatch_oepn_matches_direct():
    a = active_sensing_accel("oepn", _p(), _ctx())
    direct = oepn_accel(LOS, PEAK_T, 2.0, 0.2, 0.0, 50.0)
    assert np.allclose(a, direct)


def test_all_classes_implemented():
    assert set(IMPLEMENTED_CLASSES) == set(ACTIVE_SENSING_CLASSES)


def test_dispatch_unknown_class_raises():
    with pytest.raises(ValueError):
        active_sensing_accel("bogus", _p(), _ctx())


# ---- Class 2/3 schedule replay -------------------------------------------
def _sp(u, dt=1.0, taper=0.0):
    return {"schedule_u": np.asarray(u, float), "schedule_dt": dt,
            "taper_range_m": taper}


def test_schedule_empty_or_zero_is_inert():
    assert np.allclose(schedule_accel(LOS, 0.5, [], 1.0, 0.0, 50.0), 0.0)
    assert np.allclose(schedule_accel(LOS, 0.5, [0.0, 0.0], 1.0, 0.0, 50.0), 0.0)
    assert np.allclose(schedule_accel(LOS, 0.5, [2.0], 0.0, 0.0, 50.0), 0.0)  # dt<=0


def test_schedule_picks_the_right_knot():
    u = [1.0, -2.0, 3.0]                       # knots over [0,1),[1,2),[2,3)
    a0 = schedule_accel(LOS, 0.5, u, 1.0, 0.0, 50.0)
    a1 = schedule_accel(LOS, 1.5, u, 1.0, 0.0, 50.0)
    a2 = schedule_accel(LOS, 2.5, u, 1.0, 0.0, 50.0)
    assert abs(np.linalg.norm(a0) - 1.0) < 1e-9 and a0[1] > 0     # +1 along +y
    assert abs(np.linalg.norm(a1) - 2.0) < 1e-9 and a1[1] < 0     # -2 along -y
    assert abs(np.linalg.norm(a2) - 3.0) < 1e-9 and a2[1] > 0     # +3 along +y


def test_schedule_zero_before_and_after():
    u = [2.0, 2.0]
    assert np.allclose(schedule_accel(LOS, -0.1, u, 1.0, 0.0, 50.0), 0.0)   # before
    assert np.allclose(schedule_accel(LOS, 2.5, u, 1.0, 0.0, 50.0), 0.0)    # past end


def test_schedule_perp_horizontal_and_tapered():
    a = schedule_accel(np.array([1.0, 0.0, 0.5]), 0.5, [2.0], 1.0, 0.0, 50.0)
    assert abs(float(np.dot(a, [1.0, 0.0, 0.5]))) < 1e-9 and abs(a[2]) < 1e-9
    half = schedule_accel(LOS, 0.5, [2.0], 1.0, 20.0, 10.0)   # taper 20 m, range 10 m
    assert abs(np.linalg.norm(half) - 1.0) < 1e-9


def test_dispatch_schedule_classes():
    for cls in ("opt_weave", "fim_mpc"):
        a = active_sensing_accel(cls, _sp([1.0, -1.0], dt=1.0), _ctx(t_s=0.5))
        direct = schedule_accel(LOS, 0.5, [1.0, -1.0], 1.0, 0.0, 50.0)
        assert np.allclose(a, direct)
    # inert when the schedule is empty/zero (node warns; never a hard error)
    assert np.allclose(active_sensing_accel("opt_weave", _sp([0.0]), _ctx()), 0.0)


# ---- shared-envelope invariant (Q1: sum then clamp to the SAME a_max) ------
def test_shared_envelope_never_exceeds_a_max():
    a_max = 6.0
    a_pn = np.array([3.0, 0.0, 0.0])
    a_obs = active_sensing_accel("oepn", _p(amp=4.0), _ctx())  # 4 m/s^2 ⊥LOS
    a_cmd = limit_norm(a_pn + a_obs, a_max)
    assert np.linalg.norm(a_cmd) <= a_max + 1e-9


def test_shared_envelope_squeezes_weave_when_pursuit_saturates():
    # pursuit already at the limit -> the weave is squeezed (norm stays a_max),
    # i.e. observability is traded away exactly when pursuit demands full authority.
    a_max = 6.0
    a_pn = np.array([6.0, 0.0, 0.0])                          # saturating pursuit
    a_obs = active_sensing_accel("oepn", _p(amp=3.0), _ctx())
    a_cmd = limit_norm(a_pn + a_obs, a_max)
    assert abs(np.linalg.norm(a_cmd) - a_max) < 1e-9
    # the residual cross-LOS component is strictly smaller than the un-clamped weave
    assert abs(a_cmd[1]) < np.linalg.norm(a_obs)


def test_none_path_is_byte_identical_to_bare_pn_clamp():
    # The node's 'none' branch integrates cmd.acceleration_mps2 == limit_norm(a_pn, a_max);
    # the active path with a_obs=0 must reduce to exactly that.
    a_max = 6.0
    a_pn = np.array([2.0, 1.0, 0.0])
    bare = limit_norm(a_pn, a_max)
    via_active = limit_norm(a_pn + np.zeros(3), a_max)
    assert np.array_equal(bare, via_active)


# ---- Ticket 026 F1 — AOPN (aopn_accel), faithful Lim & Li Eq 41 -----------
def _pa(n2=1e-4, sign=1.0):
    return {"aopn_n2": n2, "aopn_sign": sign}


def test_aopn_default_off_n2():
    assert np.allclose(aopn_accel(LOS, 50.0, -7.5, 0.0), 0.0)
    assert np.allclose(aopn_accel(LOS, 50.0, -7.5, -1.0), 0.0)   # non-positive N2 -> off


def test_aopn_perp_horizontal():
    a = aopn_accel(np.array([1.0, 0.0, 0.5]), 40.0, -7.5, 1e-4)
    assert abs(a[2]) < 1e-12                                     # horizontal
    assert abs(float(np.dot(a, [1.0, 0.0, 0.5]))) < 1e-9         # ⊥ LOS


def test_aopn_fixed_sign_positive_along_z_cross_n_when_closing():
    # closing (range_rate<0), sign=+1: a = -N2·Ṙ·R²·(ẑ×n̂) with ẑ×x̂=+ŷ -> +y
    a = aopn_accel(LOS, 50.0, -7.5, 1e-4, sign=1.0)
    assert a[1] > 0.0 and abs(a[0]) < 1e-12


def test_aopn_scales_as_n2_rdot_r2():
    n2, rdot, R = 1e-4, -7.5, 40.0
    a = aopn_accel(LOS, R, rdot, n2)
    assert abs(np.linalg.norm(a) - n2 * abs(rdot) * R * R) < 1e-9
    # quadratic in range: 2R -> 4x magnitude
    a2 = aopn_accel(LOS, 2 * R, rdot, n2)
    assert abs(np.linalg.norm(a2) / np.linalg.norm(a) - 4.0) < 1e-9


def test_aopn_sign_flips_direction():
    ap = aopn_accel(LOS, 50.0, -7.5, 1e-4, sign=1.0)
    am = aopn_accel(LOS, 50.0, -7.5, 1e-4, sign=-1.0)
    assert np.allclose(ap, -am)


def test_aopn_vertical_los_returns_zero():
    assert np.allclose(aopn_accel(np.array([0.0, 0.0, 1.0]), 50.0, -7.5, 1e-4), 0.0)


def test_dispatch_aopn_matches_direct():
    ctx = _ctx(range_m=45.0, range_rate_mps=-7.5)
    a = active_sensing_accel("aopn", _pa(n2=2e-4, sign=1.0), ctx)
    direct = aopn_accel(LOS, 45.0, -7.5, 2e-4, 1.0)
    assert np.allclose(a, direct)


# ---- Ticket 026 F2 — deviated pursuit (dev_pursuit_accel), ADAPTED --------
def _pd(delta_deg=10.0, wash=15.0, gain=1.0):
    return {"dev_delta_deg": delta_deg, "dev_wash_range_m": wash, "dev_gain": gain}


def test_dev_pursuit_default_off_delta():
    v = np.array([9.0, 0.0, 0.0])
    assert np.allclose(dev_pursuit_accel(LOS, v, 40.0, 0.0, 15.0, 1.0), 0.0)


def test_dev_pursuit_perp_horizontal():
    v = np.array([9.0, 0.0, 0.0])
    a = dev_pursuit_accel(np.array([1.0, 0.0, 0.5]), v, 40.0, math.radians(10), 15.0, 1.0)
    assert abs(a[2]) < 1e-12
    assert abs(float(np.dot(a, [1.0, 0.0, 0.5]))) < 1e-9


def test_dev_pursuit_washout_to_zero_at_contact():
    # v_cmd along LOS (no lateral component): as R->0, w->0 -> desired lateral -> 0 -> a->0
    v = np.array([9.0, 0.0, 0.0])
    a_far = dev_pursuit_accel(LOS, v, 30.0, math.radians(10), 15.0, 1.0)   # R>=wash -> full
    a_near = dev_pursuit_accel(LOS, v, 1.5, math.radians(10), 15.0, 1.0)   # R=0.1*wash
    a_contact = dev_pursuit_accel(LOS, v, 1e-6, math.radians(10), 15.0, 1.0)
    assert np.linalg.norm(a_far) > np.linalg.norm(a_near) > np.linalg.norm(a_contact)
    assert np.linalg.norm(a_contact) < 1e-4


def test_dev_pursuit_lead_angle_positive_lateral():
    # v along LOS, full washout (R>=wash): steer toward +lead angle -> +y accel
    v = np.array([9.0, 0.0, 0.0])
    a = dev_pursuit_accel(LOS, v, 30.0, math.radians(10), 15.0, 1.0)
    assert a[1] > 0.0 and abs(a[0]) < 1e-12


def test_dev_pursuit_zero_when_already_at_lead_angle():
    # velocity already at lead angle delta (R>=wash so w=1): no steering needed -> a=0
    d = math.radians(10)
    v = 9.0 * np.array([math.cos(d), math.sin(d), 0.0])   # heading at +delta off LOS(+x)
    a = dev_pursuit_accel(LOS, v, 30.0, d, 15.0, 1.0)
    assert np.linalg.norm(a) < 1e-9


def test_dev_pursuit_vertical_los_returns_zero():
    v = np.array([0.0, 0.0, 9.0])
    assert np.allclose(dev_pursuit_accel(np.array([0.0, 0.0, 1.0]), v, 40.0,
                                         math.radians(10), 15.0, 1.0), 0.0)


def test_dispatch_dev_pursuit_matches_direct():
    v = np.array([9.0, 0.0, 0.0])
    ctx = _ctx(range_m=30.0, v_cmd=v)
    a = active_sensing_accel("dev_pursuit", _pd(delta_deg=10.0, wash=15.0, gain=1.0), ctx)
    direct = dev_pursuit_accel(LOS, v, 30.0, math.radians(10.0), 15.0, 1.0)
    assert np.allclose(a, direct)


def test_dispatch_none_still_zero_with_new_classes():
    assert np.allclose(active_sensing_accel("none", _pa(), _ctx()), 0.0)


# ---- Ticket 026 F3 — online constrained FIM MPC (fim_mpc_online) ----------
from mas_pn_guidance.fim_mpc_online import OnlineFimMpc   # noqa: E402

CROSS_RELP = np.array([50.0, 0.0, 0.0])    # 50 m along +x LOS
CROSS_TGTV = np.array([0.0, 5.0, 0.0])     # target crossing +y (informative)
OWN_V = np.array([9.0, 0.0, 0.0])          # ego v_max toward target


def _fresh_mpc(hit_r=1.0, samples=48):
    return OnlineFimMpc(a_max=6.0, v_max=9.0, n_nav=3.0, samples=samples,
                        hit_r=hit_r, seed=26)


def _mpc_ctx(rel_p=CROSS_RELP, tgt_v=CROSS_TGTV, v_cmd=OWN_V, dt=0.02):
    return ActiveSensingContext(
        n_hat=rel_p / np.linalg.norm(rel_p), t_s=0.0,
        range_m=float(np.linalg.norm(rel_p)), v_cmd=np.asarray(v_cmd, float),
        own_p=np.zeros(3), tgt_p=np.asarray(rel_p, float), tgt_v=np.asarray(tgt_v, float), dt=dt)


def test_fim_online_deterministic():
    a1 = _fresh_mpc().step(CROSS_RELP, OWN_V, CROSS_TGTV, 0.02)
    a2 = _fresh_mpc().step(CROSS_RELP, OWN_V, CROSS_TGTV, 0.02)
    assert np.allclose(a1, a2)


def test_fim_online_crossing_feasible_nonzero():
    p = _fresh_mpc(hit_r=1.0)
    a = p.step(CROSS_RELP, OWN_V, CROSS_TGTV, 0.02)
    assert p.last_diag()["plan_feasible"]
    assert np.linalg.norm(a) > 1e-3            # weaves to gain F_kappa (feasibility headroom)


def test_fim_online_infeasible_returns_pure_pursuit():
    # microscopic hit radius: no candidate reaches it within the predicted engagement,
    # so the feasible set is empty -> the planner falls back to pure pursuit (zero a_obs).
    p = _fresh_mpc(hit_r=1e-4)
    a = p.step(CROSS_RELP, OWN_V, CROSS_TGTV, 0.02)
    assert np.allclose(a, 0.0)
    assert not p.last_diag()["plan_feasible"]


def test_fim_online_hard_filter_keeps_capture():
    # the EXECUTED plan must itself be feasible: re-rolling it predicts CPA <= hit_r
    # (the hard constraint, not a soft penalty).
    p = _fresh_mpc(hit_r=1.0)
    p.step(CROSS_RELP, OWN_V, CROSS_TGTV, 0.02)
    _, _, cpa = p._rollout(CROSS_RELP, OWN_V, CROSS_TGTV, p._plan)
    assert cpa <= p.hit_r + 1e-9


def test_fim_online_dispatch_matches_direct():
    a_disp = active_sensing_accel("fim_mpc_online", {"fim_planner": _fresh_mpc()}, _mpc_ctx())
    a_dir = _fresh_mpc().step(CROSS_RELP, OWN_V, CROSS_TGTV, 0.02)
    assert np.allclose(a_disp, a_dir)


def test_fim_online_dispatch_no_planner_is_inert():
    assert np.allclose(active_sensing_accel("fim_mpc_online", {}, _mpc_ctx()), 0.0)


# ---------------------------------------------------------------- rge (Law A) --
def _rge_ctx_args(r_rel, v_rel, n_hat=None):
    """Helper: (n_hat, own_p, tgt_p, tgt_v, v_cmd) for a believed relative state.
    own at origin flying v_own; target at r_rel with velocity v_rel + v_own."""
    from mas_pn_guidance.active_sensing import lateral_hat  # noqa: F401 (sanity import)
    v_own = np.array([0.0, 8.0, 0.0])
    r_rel = np.asarray(r_rel, float)
    n = r_rel / np.linalg.norm(r_rel) if n_hat is None else np.asarray(n_hat, float)
    return n, np.zeros(3), r_rel, np.asarray(v_rel, float) + v_own, v_own


def test_rge_two_phase_full_excitation_far():
    """Far, closing, on-course: margin >> 0 -> u saturates at gamma_exc*a_max."""
    from mas_pn_guidance.active_sensing import rge_accel
    n, op, tp, tv, vc = _rge_ctx_args([0.0, 50.0, 0.0], [0.0, -8.0, 0.0])
    a = rge_accel(n, op, tp, tv, vc, 6.0, beta=0.5, gamma_exc=0.4, m_soft=2.0, sign=1.0)
    assert np.isclose(np.linalg.norm(a), 0.4 * 6.0, atol=1e-9)


def test_rge_two_phase_shutoff_near_cpa():
    """Small tau (close, fast closing): envelope ~ tau^2 collapses -> u decays to a few
    percent of u_max (exact zero needs margin <= 0; an on-course belief has ZEM = 0, so
    the soft gate approaches zero asymptotically — practical shutoff is the claim)."""
    from mas_pn_guidance.active_sensing import rge_accel
    n, op, tp, tv, vc = _rge_ctx_args([0.0, 3.0, 0.0], [0.0, -8.0, 0.0])
    a_near = rge_accel(n, op, tp, tv, vc, 6.0, beta=0.5, gamma_exc=0.4, m_soft=2.0, sign=1.0)
    n2, op2, tp2, tv2, vc2 = _rge_ctx_args([0.0, 50.0, 0.0], [0.0, -8.0, 0.0])
    a_far = rge_accel(n2, op2, tp2, tv2, vc2, 6.0, beta=0.5, gamma_exc=0.4, m_soft=2.0, sign=1.0)
    assert np.linalg.norm(a_near) < 0.1 * 0.4 * 6.0
    assert np.linalg.norm(a_near) < 0.1 * np.linalg.norm(a_far)


def test_rge_margin_closes_on_large_believed_zem():
    """Off-course belief (large ZEM) at medium tau: margin < 0 -> excitation pauses."""
    from mas_pn_guidance.active_sensing import rge_accel
    n, op, tp, tv, vc = _rge_ctx_args([0.0, 20.0, 0.0], [6.0, -8.0, 0.0])  # big cross vel
    a = rge_accel(n, op, tp, tv, vc, 6.0, beta=0.5, gamma_exc=0.4, m_soft=2.0, sign=1.0)
    n2, op2, tp2, tv2, vc2 = _rge_ctx_args([0.0, 20.0, 0.0], [0.0, -8.0, 0.0])  # on-course
    a2 = rge_accel(n2, op2, tp2, tv2, vc2, 6.0, beta=0.5, gamma_exc=0.4, m_soft=2.0, sign=1.0)
    assert np.linalg.norm(a) < np.linalg.norm(a2)


def test_rge_envelope_cap_and_direction():
    """|a_obs| <= gamma_exc*a_max and a_obs is horizontal-perp to the LOS."""
    from mas_pn_guidance.active_sensing import rge_accel
    n, op, tp, tv, vc = _rge_ctx_args([10.0, 40.0, -3.0], [0.0, -8.0, 0.0])
    a = rge_accel(n, op, tp, tv, vc, 6.0, beta=0.7, gamma_exc=0.3, m_soft=2.0, sign=-1.0)
    assert np.linalg.norm(a) <= 0.3 * 6.0 + 1e-9
    assert abs(float(a @ n)) < 1e-9 and abs(a[2]) < 1e-9


def test_rge_vertical_los_inert():
    from mas_pn_guidance.active_sensing import rge_accel
    n = np.array([0.0, 0.0, 1.0])
    a = rge_accel(n, np.zeros(3), np.array([0, 0, 50.0]), np.array([0, 0, -8.0]),
                  np.array([0, 0, 0.0]), 6.0, 0.5, 0.4, 2.0, 1.0)
    assert np.linalg.norm(a) == 0.0


def test_rge_dispatch():
    from mas_pn_guidance.active_sensing import (ActiveSensingContext,
                                                active_sensing_accel)
    ctx = ActiveSensingContext(n_hat=np.array([0.0, 1.0, 0.0]), t_s=0.0,
                               range_m=50.0, a_max=6.0,
                               v_cmd=np.array([0.0, 8.0, 0.0]),
                               own_p=np.zeros(3), tgt_p=np.array([0.0, 50.0, 0.0]),
                               tgt_v=np.zeros(3), dt=0.02)
    a = active_sensing_accel("rge", {"rge_beta": 0.5, "rge_gamma_exc": 0.4,
                                     "rge_msoft": 2.0, "rge_sign": 1.0}, ctx)
    assert np.isclose(np.linalg.norm(a), 2.4, atol=1e-9)


# ------------------------------------------------- fim_mpc_bs (Law B) --------
def _bs_geom():
    """Crossing-like believed state: target 40 m ahead-right, crossing laterally."""
    rel_p = np.array([12.0, 38.0, 0.0])
    own_v = np.array([0.0, 8.5, 0.0])
    tgt_v = np.array([3.0, 0.0, 0.0])
    return rel_p, own_v, tgt_v


def test_bs_no_cliff_where_ce_falls_back():
    """Empty hard-feasible set (hit_r=1e-4): CE returns the zero plan (pure pursuit);
    belief-space returns a NONZERO executed plan — the measured F3 cliff is removed."""
    from mas_pn_guidance.fim_mpc_online import OnlineFimMpc
    rel_p, own_v, tgt_v = _bs_geom()
    ce = OnlineFimMpc(a_max=6.0, v_max=9.0, n_nav=3.0, hit_r=1e-4, seed=26)
    ce.replan(rel_p, own_v, tgt_v)
    assert not ce._plan_feasible and np.allclose(ce._plan, 0.0)
    bs = OnlineFimMpc(a_max=6.0, v_max=9.0, n_nav=3.0, hit_r=1e-4, seed=26,
                      belief_space=True)
    bs.replan(rel_p, own_v, tgt_v, sigma_r0=8.0)
    assert np.linalg.norm(bs._plan) > 0.0


def test_bs_dual_effect_credit_with_uncertain_belief():
    """Large sigma_r0: the EM-optimal plan gathers information (fkappa > 0)."""
    from mas_pn_guidance.fim_mpc_online import OnlineFimMpc
    rel_p, own_v, tgt_v = _bs_geom()
    bs = OnlineFimMpc(a_max=6.0, v_max=9.0, n_nav=3.0, hit_r=0.5, seed=26,
                      belief_space=True)
    bs.replan(rel_p, own_v, tgt_v, sigma_r0=10.0)
    assert np.linalg.norm(bs._plan) > 0.0
    assert bs._last_em < np.inf


def test_bs_deterministic():
    from mas_pn_guidance.fim_mpc_online import OnlineFimMpc
    rel_p, own_v, tgt_v = _bs_geom()
    a = OnlineFimMpc(a_max=6.0, v_max=9.0, n_nav=3.0, hit_r=0.5, seed=26,
                     belief_space=True)
    b = OnlineFimMpc(a_max=6.0, v_max=9.0, n_nav=3.0, hit_r=0.5, seed=26,
                     belief_space=True)
    a.replan(rel_p, own_v, tgt_v, sigma_r0=5.0)
    b.replan(rel_p, own_v, tgt_v, sigma_r0=5.0)
    assert np.allclose(a._plan, b._plan)


def test_bs_sync_step_executes_infeasible_plan():
    """Synchronous step in belief mode never zeroes on 'not hard-feasible'."""
    from mas_pn_guidance.fim_mpc_online import OnlineFimMpc
    rel_p, own_v, tgt_v = _bs_geom()
    bs = OnlineFimMpc(a_max=6.0, v_max=9.0, n_nav=3.0, hit_r=1e-4, seed=26,
                      belief_space=True, replan_ticks=5)
    a = bs.step(rel_p, own_v, tgt_v, 0.02, sigma_r0=8.0)
    assert np.linalg.norm(a) > 0.0
