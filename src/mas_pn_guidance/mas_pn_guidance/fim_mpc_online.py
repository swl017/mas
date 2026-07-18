"""Online receding-horizon FIM-optimal active sensing with a HARD capture-feasibility
filter (ticket 026 F3, faithful). Pure numpy (+ optional stdlib threading), no ROS.

Distinct from the 023 ``fim_mpc`` proxy (offline REPLAY of a SOFT ``lambda*B - CPA``
score): here the capture constraint is a **hard feasibility FILTER** — every candidate
rollout whose predicted CPA exceeds the hit radius is discarded as *infeasible* BEFORE
observability is scored; the planner maximises predicted range-Fisher over the feasible
set only. If no candidate is feasible, it returns zero (pure pursuit — **capture takes
priority**). Online, receding-horizon, warm-started.

The rollout mirrors the node-faithful point-mass twin in ``precompute_schedules.py``
(true PN N + ``a_cmd=clamp(a_PN + u*l̂, a_max)`` + ``v_max`` clamp, CV target). The
parallax budget ``B=∫(d_⊥/r_eff)²dt`` uses the SAME integrand as the ticket-015
``fim_trace`` (forced displacement off the constant-relative-velocity baseline).

Rev1 (ticket 026, rev1.md §3/§7) — REAL-TIME re-architecture. The pre-rev1 planner ran
the full per-candidate CEM synchronously on the 50 Hz guidance callback, blocking it for
~2–3.6 s (realized guidance rate collapsed to ~11–12 Hz with ~1 s gaps). This version:

  * **Vectorizes** the rollout — all M candidates are integrated together as batched numpy
    (``O(n_steps)`` Python iterations instead of ``O(M·n_steps)``), cutting a replan to
    ~tens of ms and, crucially, releasing the GIL inside each numpy op so a 50 Hz timer
    thread interleaves cleanly.
  * Runs the CEM in an optional **background worker** (``background=True``). ``step()`` is
    then NON-BLOCKING: it submits the latest state snapshot, and returns the cached,
    **timestamped** plan knot indexed by plan AGE, with a staleness/deadline fallback to
    pure pursuit. The 50 Hz publisher never waits on the planner.

``background=False`` (default) keeps the original SYNCHRONOUS, deterministic semantics
used by the unit tests and any offline certificate caller. ``time.monotonic`` is only
touched in the ROS-node (background) path — never in the offline/deterministic path.
"""
from __future__ import annotations

import math
import threading
import time

import numpy as np

_Z = np.array([0.0, 0.0, 1.0])


# ------------------------------------------------------------ scalar helpers (kept)
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
    initial relative velocity ``V0`` — the ticket-015 Lemma-B integrand."""
    n = P.shape[0]
    if n < 2:
        return 0.0, 0.0
    t = np.arange(n) * dt
    base = P[0][None, :] + V0[None, :] * t[:, None]
    d = P - base
    rng = np.linalg.norm(P, axis=1)
    r_eff = np.maximum(rng, r_floor)
    n_hat = P / np.maximum(rng, 1e-9)[:, None]
    d_par = np.sum(d * n_hat, axis=1)[:, None] * n_hat
    d_perp = np.linalg.norm(d - d_par, axis=1)
    integrand = (d_perp / r_eff) ** 2
    B = float(np.trapz(integrand, dx=dt))
    return B, B / (sigma_theta ** 2)


# ------------------------------------------------------------ batched helpers (rev1)
def _lat_hat_batch(n):                                   # n (M,3)
    lat = np.cross(np.broadcast_to(_Z, n.shape), n)
    ln = np.linalg.norm(lat, axis=1)
    out = np.zeros_like(n)
    ok = ln > 1e-6
    out[ok] = lat[ok] / ln[ok, None]
    return out


def _clamp_batch(v, lim):                                # v (M,3)
    if lim <= 0.0:
        return v
    nv = np.linalg.norm(v, axis=1)
    scale = np.where(nv > lim, lim / np.maximum(nv, 1e-12), 1.0)
    return v * scale[:, None]


def _pn_raw_batch(rel_p, rel_v, n_nav):                  # (M,3),(M,3)
    rr = np.sum(rel_p * rel_p, axis=1)                   # (M,)
    safe = np.maximum(rr, 1e-9)
    n = rel_p / np.sqrt(safe)[:, None]
    vc = -np.sum(rel_v * n, axis=1)
    omega = np.cross(rel_p, rel_v) / safe[:, None]
    a = n_nav * np.maximum(vc, 0.0)[:, None] * np.cross(omega, n)
    a[rr < 1e-9] = 0.0
    return a


class OnlineFimMpc:
    """Stateful online receding-horizon CEM. The NODE owns one instance and calls
    ``step`` each control tick. With ``background=True`` the CEM runs in a worker thread
    and ``step`` is non-blocking; with ``background=False`` (default) ``step`` replans
    synchronously on ``replan_ticks`` cadence (deterministic — tests / offline)."""

    def __init__(self, a_max, v_max, n_nav, k_knots=5, horizon_s=2.0, samples=128,
                 elites=16, iters=3, replan_ticks=5, hit_r=1.0, r_floor=2.0,
                 sigma_theta=1.0, seed=26, roll_dt=0.05, max_predict_s=40.0,
                 background=False, max_plan_age_s=0.5,
                 belief_space=False, kappa=1.0, c_geo=0.5, q_r=0.05, em_tie_m=0.25):
        self.a_max = float(a_max)
        self.v_max = float(v_max)
        self.n_nav = float(n_nav)
        self.K = int(k_knots)
        self.horizon_s = float(horizon_s)
        self.M = int(samples)
        self.elites = int(elites)
        self.iters = int(iters)
        self.replan_ticks = max(1, int(replan_ticks))
        self.hit_r = float(hit_r)
        self.r_floor = float(r_floor)
        self.sigma_theta = float(sigma_theta)
        self.seed = int(seed)
        self.roll_dt = float(roll_dt)
        self.max_predict_s = float(max_predict_s)
        self.background = bool(background)
        self.max_plan_age_s = float(max_plan_age_s)
        # Law B (026 egofix drafts): approximate belief-space / covariance-aware mode.
        # Replaces the HARD feasibility filter with a risk-adjusted RMS-miss objective
        # EM = sqrt(cpa^2 + (kappa*sigma_miss)^2), sigma_miss = c_geo*sigma_R(t_cpa),
        # where sigma_R^2(t) = 1/(1/sigma_R0^2 + B/sigma_theta^2) + q_r*t  — a SCALAR
        # information-addition approximation (no full covariance projection or
        # cross-covariance; see i_design_egofix_drafts.md modeling qualifiers). The
        # dual-effect credit: candidates that excite EARN range certainty in-rollout.
        # No empty-feasible-set cliff: ranking is lexicographic (EM quantized by
        # em_tie_m, ties broken by max B). CE path (belief_space=False) is unchanged.
        self.belief_space = bool(belief_space)
        self.kappa = float(kappa)
        self.c_geo = float(c_geo)
        self.q_r = float(q_r)
        self.em_tie_m = float(em_tie_m)
        self._last_em = float("inf")
        self._last_sigma_miss = 0.0
        self._last_sigma_r0 = 0.0
        self._plan = np.zeros(self.K)
        self._plan_feasible = False
        self._since = None
        self._replan_count = 0
        self._last_fkappa = 0.0
        self._last_feasible_frac = 0.0
        self._last_cpa_pred = float("inf")
        # timing / diagnostics (background path)
        self._plan_stamp = None            # monotonic time the cached plan was produced
        self._last_solve_s = 0.0
        self._last_plan_age_s = float("inf")
        self._fallback = True              # true until the first feasible plan is applied
        self._deadline_miss = 0
        # worker plumbing (created lazily on first background step)
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None
        self._snapshot = None

    # ---- vectorized rollout / score ---------------------------------------
    def _rollout_batch(self, rel_p0, own_v0, tgt_v, U):
        """Integrate ALL M candidates together. ``U`` is (M,K) ⊥LOS knot accels.
        Returns (P (M,T,3), stop_idx (M,), cpa (M,)) with per-candidate first-CPA stop
        indices — semantics identical to the scalar rollout's break condition."""
        dt = self.roll_dt
        n_max = max(3, int(round(self.max_predict_s / dt)))
        knot_dt = self.horizon_s / self.K
        M = U.shape[0]
        rp = np.broadcast_to(np.asarray(rel_p0, float), (M, 3)).copy()
        ov = np.broadcast_to(np.asarray(own_v0, float), (M, 3)).copy()
        tgt_v = np.asarray(tgt_v, float)
        rng0 = float(np.linalg.norm(rel_p0))
        P = np.empty((M, n_max, 3))
        rng_hist = np.empty((M, n_max))
        done = np.zeros(M, dtype=bool)          # candidate has passed CPA (range opening)
        prev = np.full(M, rng0)
        n_used = n_max
        for k in range(n_max):
            P[:, k] = rp
            rng = np.linalg.norm(rp, axis=1)
            rng_hist[:, k] = rng
            if k >= 2:                          # match scalar break: k>1, opening, prev<rng0
                done |= (~done) & (rng > prev) & (prev < rng0)
                if done.all():
                    n_used = k + 1
                    break
            prev = rng
            n_hat = rp / np.maximum(rng, 1e-9)[:, None]
            rel_v = tgt_v[None, :] - ov
            a_pn = _pn_raw_batch(rp, rel_v, self.n_nav)
            t = k * dt
            if t < self.horizon_s:
                u = U[:, min(int(t / knot_dt), self.K - 1)]
            else:
                u = np.zeros(M)
            a_cmd = _clamp_batch(a_pn + u[:, None] * _lat_hat_batch(n_hat), self.a_max)
            ov = _clamp_batch(ov + a_cmd * dt, self.v_max)
            rp = rp + (tgt_v[None, :] - ov) * dt
        P = P[:, :n_used]
        rng_hist = rng_hist[:, :n_used]
        n_max = n_used
        # first-CPA stop index per candidate: first k>=2 with rng[k]>rng[k-1] & rng[k-1]<rng0
        opening = rng_hist[:, 1:] > rng_hist[:, :-1]          # position j -> compares k=j+1 vs j
        prev_below = rng_hist[:, :-1] < rng0
        cond = opening & prev_below
        cond[:, 0] = False                                   # enforce k>1 (exclude k=1)
        has = cond.any(axis=1)
        stop_idx = np.where(has, cond.argmax(axis=1) + 1, n_max - 1)
        mask = np.arange(n_max)[None, :] <= stop_idx[:, None]
        cpa = np.min(np.where(mask, rng_hist, np.inf), axis=1)
        return P, stop_idx, cpa

    def _rollout(self, rel_p0, own_v0, tgt_v, u_knots):
        """Single-candidate rollout (verification convenience; the planner itself uses the
        batched path). Returns (P, V0, cpa) — identical semantics to ``_rollout_batch`` for
        one schedule, used to re-check that an EXECUTED plan is itself capture-feasible."""
        P, stop_idx, cpa = self._rollout_batch(
            rel_p0, own_v0, tgt_v, np.asarray(u_knots, float)[None, :])
        n = int(stop_idx[0]) + 1
        V0 = np.asarray(tgt_v, float) - np.asarray(own_v0, float)
        return P[0, :n], V0, float(cpa[0])

    def _budget_batch(self, P, V0, stop_idx):
        """B per candidate (M,) over [0, stop_idx] — ticket-015 Lemma-B integrand,
        forced displacement off the CV baseline; masked beyond each candidate's CPA."""
        M, T, _ = P.shape
        t = np.arange(T) * self.roll_dt
        base = P[:, :1, :] + V0[None, None, :] * t[None, :, None]
        d = P - base
        rng = np.linalg.norm(P, axis=2)
        r_eff = np.maximum(rng, self.r_floor)
        n_hat = P / np.maximum(rng, 1e-9)[:, :, None]
        d_par = np.sum(d * n_hat, axis=2)[:, :, None] * n_hat
        d_perp = np.linalg.norm(d - d_par, axis=2)
        integrand = (d_perp / r_eff) ** 2
        mask = np.arange(T)[None, :] <= stop_idx[:, None]
        return np.sum(integrand * mask, axis=1) * self.roll_dt

    def _score_batch(self, rel_p0, own_v0, tgt_v, U, sigma_r0=None):
        P, stop_idx, cpa = self._rollout_batch(rel_p0, own_v0, tgt_v, U)
        B = self._budget_batch(P, np.asarray(tgt_v, float) - np.asarray(own_v0, float),
                               stop_idx)
        fkappa = B / (self.sigma_theta ** 2)
        if not self.belief_space or sigma_r0 is None:
            return fkappa, (cpa <= self.hit_r), cpa, None
        s0sq = max(float(sigma_r0), 1e-3) ** 2
        t_stop = stop_idx * self.roll_dt
        var_r = 1.0 / (1.0 / s0sq + np.maximum(fkappa, 0.0)) + self.q_r * t_stop
        sigma_miss = self.c_geo * np.sqrt(var_r)
        em = np.sqrt(cpa ** 2 + (self.kappa * sigma_miss) ** 2)
        return fkappa, (cpa <= self.hit_r), cpa, em

    def _replan_core(self, rel_p0, own_v0, tgt_v, sigma_r0=None):
        """Vectorized CEM over K ⊥LOS knots maximising predicted F_κ over the FEASIBLE
        set only. Returns (plan, feasible, fkappa, feasible_frac, cpa_pred). Deterministic
        (seeded on the replan count). Empty feasible set ⇒ zero plan (pure pursuit)."""
        rng = np.random.default_rng(self.seed + self._replan_count)
        mean = self._plan.copy() if self._plan_feasible else np.zeros(self.K)
        std = np.full(self.K, 0.5 * self.a_max)
        best_u, best_f, best_feas, cpa_best = np.zeros(self.K), -np.inf, False, float("inf")
        em_best = float("inf")
        frac_last = 0.0
        for _ in range(self.iters):
            U = np.clip(rng.normal(mean, std, size=(self.M, self.K)),
                        -self.a_max, self.a_max)
            fk, feas, cpa, em = self._score_batch(rel_p0, own_v0, tgt_v, U, sigma_r0)
            frac_last = float(feas.mean())
            if em is not None:
                # Law B: lexicographic (quantized EM asc, then B desc) — never empty.
                em_q = np.round(em / self.em_tie_m) * self.em_tie_m
                score = -em_q * 1e6 + np.minimum(fk, 1e5)
                i_best = int(np.argmax(score))
                if score[i_best] > best_f:
                    best_f, best_u = float(score[i_best]), U[i_best].copy()
                    best_feas = bool(feas[i_best])
                    cpa_best, em_best = float(cpa[i_best]), float(em[i_best])
                elite = U[np.argsort(score)[::-1][:max(1, self.elites)]]
                mean, std = elite.mean(0), elite.std(0) + 1e-3
                continue
            n_feas = int(feas.sum())
            if n_feas == 0:
                continue
            fk_feas = np.where(feas, fk, -np.inf)
            i_best = int(np.argmax(fk_feas))
            if fk_feas[i_best] > best_f:
                best_f, best_u, best_feas = float(fk_feas[i_best]), U[i_best].copy(), True
                cpa_best = float(cpa[i_best])
            n_el = max(1, min(self.elites, n_feas))
            elite = U[np.argsort(fk_feas)[::-1][:n_el]]
            mean, std = elite.mean(0), elite.std(0) + 1e-3
        self._last_em = em_best
        if self.belief_space:
            return best_u, best_feas, max(best_f, 0.0), frac_last, cpa_best
        if best_feas:
            return best_u, True, best_f, frac_last, cpa_best
        return np.zeros(self.K), False, 0.0, frac_last, cpa_best

    def replan(self, rel_p0, own_v0, tgt_v, sigma_r0=None):
        """Synchronous replan (deterministic path + worker body). Updates the cached plan."""
        plan, feasible, fk, frac, cpa = self._replan_core(rel_p0, own_v0, tgt_v, sigma_r0)
        with self._lock:
            self._plan, self._plan_feasible, self._last_fkappa = plan, feasible, fk
            self._last_feasible_frac, self._last_cpa_pred = frac, cpa
            self._plan_stamp = time.monotonic()
            self._since = 0
        self._replan_count += 1

    # ---- worker (background path) -----------------------------------------
    def _ensure_worker(self):
        if self._thread is None:
            self._thread = threading.Thread(target=self._worker, daemon=True,
                                            name="fim_mpc_cem")
            self._thread.start()

    def _worker(self):
        while not self._stop.is_set():
            if not self._wake.wait(timeout=0.5):
                continue
            self._wake.clear()
            with self._lock:
                snap = self._snapshot
            if snap is None:
                continue
            t0 = time.monotonic()
            plan, feasible, fk, frac, cpa = self._replan_core(*snap)
            solve = time.monotonic() - t0
            with self._lock:
                self._plan, self._plan_feasible, self._last_fkappa = plan, feasible, fk
                self._last_feasible_frac, self._last_cpa_pred = frac, cpa
                self._plan_stamp = time.monotonic()
                self._last_solve_s = solve
            self._replan_count += 1

    def shutdown(self):
        """Stop the worker thread (idempotent). Call from the node's destroy path."""
        self._stop.set()
        self._wake.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

    # ---- control-tick entry -----------------------------------------------
    def step(self, rel_p, own_v, tgt_v, dt, sigma_r0=None) -> np.ndarray:
        """One control tick. Returns the ⊥LOS ``a_obs`` (3-vec). ``sigma_r0`` (Law B):
        current believed range std — consumed only in belief_space mode."""
        rel_p = np.asarray(rel_p, float)
        own_v = np.asarray(own_v, float)
        tgt_v = np.asarray(tgt_v, float)
        rng = float(np.linalg.norm(rel_p))
        if rng < 1e-9:
            return np.zeros(3)
        knot_dt = self.horizon_s / self.K

        if not self.background:
            # deterministic synchronous path (tests / offline)
            if self._since is None or self._since >= self.replan_ticks:
                self.replan(rel_p, own_v, tgt_v, sigma_r0)
            ki = min(int((self._since * dt) / knot_dt), self.K - 1)
            self._since += 1
            return float(self._plan[ki]) * _lat_hat(rel_p / rng)

        # NON-BLOCKING background path: submit snapshot, apply the timestamped cached plan
        self._ensure_worker()
        with self._lock:
            self._snapshot = (rel_p.copy(), own_v.copy(), tgt_v.copy(),
                              sigma_r0)
            plan = self._plan.copy()
            feasible = self._plan_feasible
            stamp = self._plan_stamp
        self._wake.set()
        age = (time.monotonic() - stamp) if stamp is not None else float("inf")
        self._last_plan_age_s = age
        # Law B: a not-hard-feasible plan is still EXECUTED (no binary cliff) —
        # only staleness triggers the pure-pursuit fallback in belief_space mode.
        if ((not feasible) and not self.belief_space) or (age > self.max_plan_age_s):
            if age > self.max_plan_age_s and stamp is not None:
                self._deadline_miss += 1
            self._fallback = True
            return np.zeros(3)                       # capture-first: pure pursuit
        self._fallback = False
        ki = min(int(age / knot_dt), self.K - 1)     # knot indexed by PLAN AGE
        return float(plan[ki]) * _lat_hat(rel_p / rng)

    def last_diag(self) -> dict:
        """Per-tick provenance for pn/fim_diagnostics (rev1.md §4): predicted F_κ + CPA,
        feasible fraction, plan-feasible flag, plan age, solve time, deadline misses,
        fallback status, replan count."""
        with self._lock:
            return {"fkappa": self._last_fkappa,
                    "cpa_pred": self._last_cpa_pred,
                    "feasible_frac": self._last_feasible_frac,
                    "plan_feasible": self._plan_feasible,
                    "plan_age_s": self._last_plan_age_s,
                    "solve_s": self._last_solve_s,
                    "deadline_miss": self._deadline_miss,
                    "fallback": self._fallback,
                    "replans": self._replan_count,
                    "em": self._last_em,             # Law B risk-adjusted RMS miss
                    "sigma_r0": self._last_sigma_r0}
