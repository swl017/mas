"""Ego active-sensing acceleration — pure math, no ROS (ticket 023).

Single-agent active-sensing guidance CLASSES that add an observability
acceleration ``a_obs`` (perpendicular to the LOS, horizontal plane) to the PN
command. The node sums ``a_obs`` with the PN acceleration and clamps the SUM to
the SAME ``a_max`` (one shared actuation envelope, ticket 023 Q1), so weaving
trades collision-course efficiency for range observability rather than buying
extra authority; when pursuit saturates near terminal the clamp squeezes
``a_obs`` out — that squeeze IS the pursuit-parallax starvation and stays visible.

Classes (ticket 023 i_design):

  none       -> zero (default; the node takes its byte-identical existing path)
  oepn       -> Class 1: open-loop lateral sinusoid ``A·σ(r)·sin(2πf·t)·l̂``
                (Observability-Enhanced PN; Li & Lim), ACCELERATION units m/s².
  opt_weave  -> Class 2: observability-optimal (B-maximizing) schedule  [S2, TBD]
  fim_mpc    -> Class 3: online FIM-optimal CEM MPC                     [S2, TBD]

DISTINCT from ``ego_weave.py`` (ticket 019), which superimposes a VELOCITY weave
on the OUTPUT after the ``v_max`` clamp. Here ``a_obs`` enters at the ACCELERATION
level BEFORE the integrate-and-clamp. Default-off: class ``none`` or a
non-positive amplitude/frequency returns a zero vector, so every non-active arm
is byte-identical to before.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict

import numpy as np

_Z = np.array([0.0, 0.0, 1.0])

# All classes named by the design. Class 1 (oepn) is parametric; Class 2/3
# (opt_weave/fim_mpc) replay an offline-optimized ⊥LOS-accel SCHEDULE emitted by
# research/.../precompute_schedules.py (CV target => the offline global optimum
# equals the online receding-horizon CEM; 023 i_design §3.3).
# ticket 023 proxies: none/oepn/opt_weave/fim_mpc. ticket 026 literature-grade laws:
# aopn (F1, faithful Lim & Li), dev_pursuit (F2, Anjaly-Ratnoo-INSPIRED adaptation),
# fim_mpc_online (F3, faithful online constrained MPC — wired in 026 S2).
ACTIVE_SENSING_CLASSES = ("none", "oepn", "opt_weave", "fim_mpc",
                          "aopn", "dev_pursuit", "fim_mpc_online", "rge",
                          "fim_mpc_bs")
IMPLEMENTED_CLASSES = ACTIVE_SENSING_CLASSES
_SCHEDULE_CLASSES = ("opt_weave", "fim_mpc")


@dataclass(frozen=True)
class ActiveSensingContext:
    """Everything a class needs to compute ``a_obs`` at one control tick.

    Class 1 (oepn) uses only ``n_hat``/``t_s``/``range_m``; the remaining fields
    are populated for the Class 2/3 planners (S2) so their signature is stable now.
    """
    n_hat: np.ndarray            # interceptor->target LOS (any length; renormalized)
    t_s: float                   # seconds since pursuit (re)seed — weave phase base
    range_m: float               # current interceptor-target range
    range_rate_mps: float = 0.0  # ego Ṙ estimate (= -closing_speed); F1/F3 (ticket 026)
    a_pn: np.ndarray = field(default_factory=lambda: np.zeros(3))   # pre-clamp PN accel
    a_max: float = 0.0
    v_cmd: np.ndarray = field(default_factory=lambda: np.zeros(3))
    own_p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tgt_p: np.ndarray = field(default_factory=lambda: np.zeros(3))
    tgt_v: np.ndarray = field(default_factory=lambda: np.zeros(3))
    dt: float = 0.0


def lateral_hat(n_hat) -> np.ndarray:
    """Horizontal unit vector perpendicular to the LOS (``l̂ = ẑ × n̂``).

    Returns a zero vector when the LOS is ~vertical (no horizontal perpendicular)
    or degenerate — mirrors ``ego_weave`` so the two paths agree on direction.
    """
    n = np.asarray(n_hat, dtype=float)
    nn = float(np.linalg.norm(n))
    if nn < 1e-9:
        return np.zeros(3)
    n = n / nn
    lat = np.cross(_Z, n)
    ln = float(np.linalg.norm(lat))
    if ln < 1e-6:                                # LOS ~vertical -> no horizontal perp
        return np.zeros(3)
    return lat / ln


def taper_gain(range_m: float, taper_range_m: float) -> float:
    """Linear range taper in ``[0, 1]``: ramps to 0 as range -> 0 below
    ``taper_range_m`` (``<=0`` disables -> always 1)."""
    if taper_range_m <= 0.0:
        return 1.0
    return float(np.clip(range_m / taper_range_m, 0.0, 1.0))


def oepn_accel(n_hat, t_s, amp_mps2, freq_hz, taper_range_m, range_m) -> np.ndarray:
    """Class 1 OEPN observability ACCELERATION (3-vector, m/s²).

    ``a_obs = A·σ(r)·sin(2πf·t)·l̂``, ``l̂`` horizontal ⊥ LOS. Default-off:
    ``amp<=0`` or ``freq<=0`` -> zeros.
    """
    if amp_mps2 <= 0.0 or freq_hz <= 0.0:
        return np.zeros(3)
    lat = lateral_hat(n_hat)
    if not np.any(lat):
        return np.zeros(3)
    taper = taper_gain(range_m, taper_range_m)
    return amp_mps2 * taper * math.sin(2.0 * math.pi * freq_hz * t_s) * lat


def schedule_accel(n_hat, t_s, u_knots, sched_dt, taper_range_m, range_m) -> np.ndarray:
    """Class 2/3 observability ACCELERATION by replaying a precomputed ⊥LOS-accel
    schedule (m/s²): ``a_obs = u(t_s)·σ(r)·l̂``.

    ``u_knots`` is a piecewise-constant schedule sampled every ``sched_dt`` seconds
    (knot ``k`` covers ``[k·sched_dt, (k+1)·sched_dt)``); 0 before the schedule and
    after its end. Empty/all-zero schedule or ``sched_dt<=0`` -> zeros (inert).
    """
    if u_knots is None:
        return np.zeros(3)
    u_knots = np.asarray(u_knots, dtype=float)
    if u_knots.size == 0 or sched_dt <= 0.0 or not np.any(u_knots):
        return np.zeros(3)
    if t_s < 0.0:
        return np.zeros(3)
    k = int(t_s / sched_dt)
    if k < 0 or k >= u_knots.size:
        return np.zeros(3)
    lat = lateral_hat(n_hat)
    if not np.any(lat):
        return np.zeros(3)
    return float(u_knots[k]) * taper_gain(range_m, taper_range_m) * lat


def aopn_accel(n_hat, range_m, range_rate_mps, n2, sign=1.0) -> np.ndarray:
    """Class F1 — Lim & Li AOPN additive information ACCELERATION (3-vec, m/s²; ticket
    026, FAITHFUL to Lim & Li 2000 Eq 41).

    ``a_obs = -N2·Ṙ·R²·n̂_⊥``, ``n̂_⊥ = ẑ×n̂`` the FIXED horizontal ⊥LOS normal, ``N2=k/γ``.
    ``Ṙ = range_rate_mps`` is < 0 while closing, so the term is a POSITIVE, FIXED-SIGN
    lateral bias — a steady *swing* that donates range parallax, NOT the zero-mean sinusoid
    the 023 ``oepn`` proxy used. Uses the ego-estimated ``range_m``/``range_rate_mps``
    (faithful: Lim & Li close the loop through an EKF) — this is the arm where guidance
    feeds on the very range estimate it perturbs. ``sign`` picks the swing sense (both gain
    equal B, quadratic in d_⊥; the offline twin selects the capture-preserving one).
    Default-off: ``n2<=0`` -> zeros.
    """
    if n2 <= 0.0:
        return np.zeros(3)
    lat = lateral_hat(n_hat)
    if not np.any(lat):
        return np.zeros(3)
    return (-n2 * range_rate_mps * range_m * range_m) * sign * lat


def dev_pursuit_accel(n_hat, v_cmd, range_m, delta_star_rad, wash_range_m, k_delta) -> np.ndarray:
    """Class F2 — Anjaly & Ratnoo-INSPIRED capture-constrained deviated pursuit (3-vec,
    m/s²; ticket 026, ADAPTED — NOT a faithful reproduction).

    Holds a lead angle ``δ(R)=δ*·w(R)`` off the LOS via a lateral steering accel, with a
    terminal WASHOUT ``w(R)=min(1, R/R_wash) -> 0`` as ``R->0`` so the deviation collapses
    into pure pursuit near capture:
    ``a_obs = k_δ·(|v_cmd|·sin δ(R) − v_cmd·n̂_⊥)·n̂_⊥`` (Anjaly-Ratnoo Eq-109 heading-hold
    motif). NOTE the washout is a REPOSITORY-SPECIFIC capture-preserving choice and runs
    *opposite* to Eq 89 ``δ_F=(R0-R)/R0·δ_L`` (which GROWS δ toward terminal for a docking
    rendezvous); the paper supplies no faithful single-agent noncooperative law, hence
    *adapted*. Scalar ``δ*`` only (a free δ(t) would overlap F3). Default-off:
    ``δ*==0`` -> zeros.
    """
    if delta_star_rad == 0.0:
        return np.zeros(3)
    lat = lateral_hat(n_hat)
    if not np.any(lat):
        return np.zeros(3)
    w = min(1.0, range_m / wash_range_m) if wash_range_m > 0.0 else 1.0
    v_cmd = np.asarray(v_cmd, dtype=float)
    v_perp_des = float(np.linalg.norm(v_cmd)) * math.sin(delta_star_rad * w)
    return k_delta * (v_perp_des - float(v_cmd @ lat)) * lat


def rge_accel(n_hat, own_p, tgt_p, tgt_v, v_cmd, a_max, beta, gamma_exc,
              m_soft, sign, tau_min=0.3) -> np.ndarray:
    """Recoverability-governed excitation (RGE) — ticket 026 egofix drafts §1, Law A.

    ZEM-reserve soft governor: excite ⊥LOS at up to ``gamma_exc*a_max`` while the
    BELIEVED zero-effort miss stays within ``beta`` of the PN-reserve lateral-reach
    envelope ``1/2*(1-gamma_exc)*a_max*tau^2`` (believed t_go). Two-phase behavior is
    emergent — the envelope collapses quadratically, so excitation shuts itself off
    approaching CPA (no R_wash schedule). Positioned per the design doc as a
    ZEM-reserve soft-governor SYNTHESIS (Lee'01 / Su'18 / capture-zone lineage), and
    the envelope is a constant-accel lateral-reach APPROXIMATION of the recoverable
    set, not a proved capture zone (modeling qualifier in i_design_egofix_drafts.md).
    All inputs are ego-belief quantities — deployable without an observer.
    """
    l_hat = lateral_hat(n_hat)
    if not np.any(l_hat):
        return np.zeros(3)
    r_rel = np.asarray(tgt_p, float) - np.asarray(own_p, float)
    v_rel = np.asarray(tgt_v, float) - np.asarray(v_cmd, float)
    vv = float(v_rel @ v_rel)
    if vv < 1e-9:
        return np.zeros(3)
    tau = max(float(tau_min), -float(r_rel @ v_rel) / vv)
    zem = float(np.linalg.norm(r_rel + v_rel * tau))
    env = 0.5 * (1.0 - gamma_exc) * a_max * tau * tau
    margin = beta * env - zem
    u = gamma_exc * a_max * min(1.0, max(0.0, margin / max(float(m_soft), 1e-6)))
    return float(sign) * u * l_hat


def active_sensing_accel(cls: str, params: Dict, ctx: ActiveSensingContext) -> np.ndarray:
    """Dispatch to the selected active-sensing class; return ``a_obs`` (m/s², 3-vec).

    ``none`` returns zeros. ``oepn`` is parametric (Class 1). ``opt_weave``/``fim_mpc``
    replay a precomputed schedule (Class 2/3). Unknown class raises ``ValueError``.
    A schedule class with an empty/zero schedule returns zeros (inert) — the node
    warns at arm start so a boot is never silently a no-op.
    """
    if cls == "none":
        return np.zeros(3)
    if cls == "oepn":
        return oepn_accel(
            ctx.n_hat, ctx.t_s,
            float(params.get("amp_mps2", 0.0)),
            float(params.get("freq_hz", 0.0)),
            float(params.get("taper_range_m", 0.0)),
            ctx.range_m)
    if cls in _SCHEDULE_CLASSES:
        return schedule_accel(
            ctx.n_hat, ctx.t_s, params.get("schedule_u"),
            float(params.get("schedule_dt", 0.0)),
            float(params.get("taper_range_m", 0.0)), ctx.range_m)
    if cls == "aopn":                                    # F1 (ticket 026, faithful)
        return aopn_accel(
            ctx.n_hat, ctx.range_m, ctx.range_rate_mps,
            float(params.get("aopn_n2", 0.0)),
            float(params.get("aopn_sign", 1.0)))
    if cls == "dev_pursuit":                             # F2 (ticket 026, adapted)
        return dev_pursuit_accel(
            ctx.n_hat, ctx.v_cmd, ctx.range_m,
            math.radians(float(params.get("dev_delta_deg", 0.0))),
            float(params.get("dev_wash_range_m", 15.0)),
            float(params.get("dev_gain", 0.0)))
    if cls == "rge":                                     # Law A (026 egofix drafts)
        return rge_accel(
            ctx.n_hat, ctx.own_p, ctx.tgt_p, ctx.tgt_v, ctx.v_cmd, ctx.a_max,
            float(params.get("rge_beta", 0.5)),
            float(params.get("rge_gamma_exc", 0.4)),
            float(params.get("rge_msoft", 2.0)),
            float(params.get("rge_sign", 1.0)))
    if cls in ("fim_mpc_online", "fim_mpc_bs"):          # F3 (CE) / Law B (belief-space)
        planner = params.get("fim_planner")
        if planner is None:                              # not constructed -> inert
            return np.zeros(3)
        rel_p = np.asarray(ctx.tgt_p, float) - np.asarray(ctx.own_p, float)
        return planner.step(rel_p, ctx.v_cmd, ctx.tgt_v, ctx.dt,
                            sigma_r0=params.get("sigma_R0"))
    raise ValueError(f"unknown active_sensing_class '{cls}'")
