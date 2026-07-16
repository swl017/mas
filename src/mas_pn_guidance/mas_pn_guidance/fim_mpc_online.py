"""Online receding-horizon FIM-optimal active sensing with a HARD capture-feasibility
filter (ticket 026 F3, faithful). Pure numpy, no ROS.

Distinct from the 023 ``fim_mpc`` proxy (offline REPLAY of a SOFT ``lambda*B - CPA``
score): here the capture constraint is a **hard feasibility FILTER** — every candidate
rollout whose predicted CPA exceeds the hit radius is discarded as *infeasible* BEFORE
observability is scored; the planner maximises predicted range-Fisher over the feasible
set only. If no candidate is feasible, it returns zero (pure pursuit — **capture takes
priority**). Online, receding-horizon, warm-started.

The rollout mirrors the node-faithful point-mass twin in ``precompute_schedules.py``
(true PN N + ``a_cmd=clamp(a_PN + u*l̂, a_max)`` + ``v_max`` clamp, CV target). The
parallax budget ``B=∫(d_⊥/r_eff)²dt`` uses the SAME integrand as the ticket-015
``fim_trace`` (forced displacement off the constant-relative-velocity baseline), so the
offline global optimum can certify this online CEM (online ``F_κ`` must match the offline
global). Deterministic: seeded numpy RNG per replan, no wall-clock / no ``Math.random``.
"""
from __future__ import annotations

import math

import numpy as np

_Z = np.array([0.0, 0.0, 1.0])


def _lat_hat(n):
    lat = np.cross(_Z, n)
    ln = float(np.linalg.norm(lat))
    return lat / ln if ln > 1e-6 else np.zeros(3)


def _clamp(v, lim):
    nv = float(np.linalg.norm(v))
    return v * (lim / nv) if (lim > 0.0 and nv > lim) else v


def _pn_raw(rel_p, rel_v, n_nav):
    """True-PN accel from the relative state (matches precompute_schedules._pn_raw)."""
    rr = float(rel_p @ rel_p)
    if rr < 1e-9:
        return np.zeros(3)
    n = rel_p / math.sqrt(rr)
    vc = -float(rel_v @ n)
    omega = np.cross(rel_p, rel_v) / rr
    return n_nav * max(vc, 0.0) * np.cross(omega, n)


def _parallax_budget(P, V0, dt, r_floor, sigma_theta):
    """B=∫(d_⊥/r_eff)²dt and F_κ=B/σ² from a predicted relative path ``P`` (n×3) and the
    initial relative velocity ``V0`` — the ticket-015 Lemma-B integrand. ``d`` is the
    forced displacement off the constant-relative-velocity baseline; ``d_⊥`` its transverse
    (⊥LOS) component; ``r_eff=max(|P|, r_floor)``."""
    n = P.shape[0]
    if n < 2:
        return 0.0, 0.0
    t = np.arange(n) * dt
    base = P[0][None, :] + V0[None, :] * t[:, None]          # free (CV-relative) baseline
    d = P - base                                             # forced displacement
    rng = np.linalg.norm(P, axis=1)
    r_eff = np.maximum(rng, r_floor)
    n_hat = P / np.maximum(rng, 1e-9)[:, None]
    d_par = np.sum(d * n_hat, axis=1)[:, None] * n_hat       # along-LOS part
    d_perp = np.linalg.norm(d - d_par, axis=1)               # transverse part
    integrand = (d_perp / r_eff) ** 2
    B = float(np.trapz(integrand, dx=dt))
    return B, B / (sigma_theta ** 2)


class OnlineFimMpc:
    """Stateful online receding-horizon CEM. The NODE owns one instance and calls
    ``step`` each control tick; it replans every ``replan_ticks`` ticks and applies the
    cached piecewise-constant ⊥LOS knot in between."""

    def __init__(self, a_max, v_max, n_nav, k_knots=5, horizon_s=2.0, samples=128,
                 elites=16, iters=3, replan_ticks=5, hit_r=1.0, r_floor=2.0,
                 sigma_theta=1.0, seed=26, roll_dt=0.05, max_predict_s=40.0):
        self.a_max = float(a_max)
        self.v_max = float(v_max)
        self.n_nav = float(n_nav)
        self.K = int(k_knots)
        self.horizon_s = float(horizon_s)      # near-term ⊥LOS knot (observability) window
        self.M = int(samples)
        self.elites = int(elites)
        self.iters = int(iters)
        self.replan_ticks = max(1, int(replan_ticks))
        self.hit_r = float(hit_r)
        self.r_floor = float(r_floor)
        self.sigma_theta = float(sigma_theta)
        self.seed = int(seed)
        self.roll_dt = float(roll_dt)          # coarse prediction step (geometry is smooth)
        self.max_predict_s = float(max_predict_s)  # roll-to-intercept cap
        self._plan = np.zeros(self.K)      # cached knot schedule (⊥LOS accel m/s²)
        self._plan_feasible = False
        self._since = None                 # ticks since last replan (None => replan now)
        self._replan_count = 0
        self._last_fkappa = 0.0
        self._last_feasible_frac = 0.0

    # ---- rollout / score ---------------------------------------------------
    def _rollout(self, rel_p0, own_v0, tgt_v, u_knots):
        """Predict the engagement to intercept: PN + the ⊥LOS knot schedule ``u_knots``
        applied over the near-term ``[0, horizon_s)`` observability window, then pure PN to
        CPA. Coarse ``roll_dt``; early-stop once range opens (past CPA) or ``max_predict_s``.
        Returns (P, V0, cpa) with the predicted terminal miss ``cpa`` — so the hard
        feasibility filter is meaningful at ANY current range (not just inside a 2 s box)."""
        dt = self.roll_dt
        n_max = max(2, int(round(self.max_predict_s / dt)))
        knot_dt = self.horizon_s / self.K
        rel_p = np.asarray(rel_p0, float).copy()
        own_v = np.asarray(own_v0, float).copy()
        tgt_v = np.asarray(tgt_v, float)
        V0 = tgt_v - own_v
        P = np.empty((n_max, 3))
        cpa = float(np.linalg.norm(rel_p))
        prev_rng = cpa
        nout = n_max
        for k in range(n_max):
            P[k] = rel_p
            rng = float(np.linalg.norm(rel_p))
            cpa = min(cpa, rng)
            if k > 1 and rng > prev_rng and prev_rng < np.linalg.norm(rel_p0):
                nout = k + 1
                break                                    # past CPA (range opening) -> stop
            prev_rng = rng
            n_hat = rel_p / max(rng, 1e-9)
            rel_v = tgt_v - own_v
            a_pn = _pn_raw(rel_p, rel_v, self.n_nav)
            t = k * dt
            u = float(u_knots[min(int(t / knot_dt), self.K - 1)]) if t < self.horizon_s else 0.0
            a_cmd = _clamp(a_pn + u * _lat_hat(n_hat), self.a_max)
            own_v = _clamp(own_v + a_cmd * dt, self.v_max)
            rel_p = rel_p + (tgt_v - own_v) * dt
        return P[:nout], V0, cpa

    def _score(self, rel_p0, own_v0, tgt_v, u_knots):
        """Predicted (F_κ, feasible) for a candidate schedule. Feasible ⟺ predicted
        CPA ≤ hit radius (the HARD constraint)."""
        P, V0, cpa = self._rollout(rel_p0, own_v0, tgt_v, u_knots)
        _, fkappa = _parallax_budget(P, V0, self.roll_dt, self.r_floor, self.sigma_theta)
        return fkappa, (cpa <= self.hit_r)

    def replan(self, rel_p0, own_v0, tgt_v):
        """CEM over K ⊥LOS knots maximising predicted F_κ over the FEASIBLE set only.
        Empty feasible set ⇒ zero plan (pure pursuit). Deterministic (seeded)."""
        rng = np.random.default_rng(self.seed + self._replan_count)
        mean = self._plan.copy() if self._plan_feasible else np.zeros(self.K)  # warm-start
        std = np.full(self.K, 0.5 * self.a_max)
        best_u, best_f, best_feas = np.zeros(self.K), -np.inf, False
        n_feas_last = 0
        for _ in range(self.iters):
            samples = np.clip(rng.normal(mean, std, size=(self.M, self.K)),
                              -self.a_max, self.a_max)
            scored = []
            n_feas_last = 0
            for u in samples:
                f, feasible = self._score(rel_p0, own_v0, tgt_v, u)
                if not feasible:
                    continue
                n_feas_last += 1
                scored.append((f, u))
                if f > best_f:
                    best_f, best_u, best_feas = f, u.copy(), True
            if not scored:                       # no feasible candidate this iter
                continue
            scored.sort(key=lambda z: z[0], reverse=True)
            elite = np.array([u for _, u in scored[:self.elites]])
            mean, std = elite.mean(0), elite.std(0) + 1e-3
        self._replan_count += 1
        self._last_feasible_frac = n_feas_last / float(self.M)
        if best_feas:
            self._plan, self._plan_feasible, self._last_fkappa = best_u, True, best_f
        else:                                     # capture infeasible to perturb -> pursuit
            self._plan, self._plan_feasible, self._last_fkappa = np.zeros(self.K), False, 0.0
        self._since = 0

    def step(self, rel_p, own_v, tgt_v, dt) -> np.ndarray:
        """One control tick: replan on cadence, else apply the cached knot. Returns the
        ⊥LOS ``a_obs`` (3-vec)."""
        rel_p = np.asarray(rel_p, float)
        own_v = np.asarray(own_v, float)
        tgt_v = np.asarray(tgt_v, float)
        if self._since is None or self._since >= self.replan_ticks:
            self.replan(rel_p, own_v, tgt_v)
        knot_dt = self.horizon_s / self.K
        ki = min(int((self._since * dt) / knot_dt), self.K - 1)
        self._since += 1
        rng = float(np.linalg.norm(rel_p))
        if rng < 1e-9:
            return np.zeros(3)
        return float(self._plan[ki]) * _lat_hat(rel_p / rng)

    def last_diag(self) -> dict:
        """Provenance: online F_κ achieved + feasible fraction at the last replan."""
        return {"fkappa": self._last_fkappa, "feasible_frac": self._last_feasible_frac,
                "plan_feasible": self._plan_feasible, "replans": self._replan_count}
