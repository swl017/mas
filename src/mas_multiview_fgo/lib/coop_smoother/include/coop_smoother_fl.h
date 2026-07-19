/**
 * @file coop_smoother_fl.h
 * @brief RAL ticket 024 S7 — FIXED-LAG backend (Q3=a): gtsam_unstable
 *        IncrementalFixedLagSmoother (iSAM2 + timestamp-driven marginalization) behind the
 *        same ingest/query/diagnostics surface as the batch CoopSmoother, so the node, the
 *        offline replay, and the S5 output gate drive either backend unchanged.
 *
 * Differences from the batch backend (by design):
 *  - PERSISTENT: measurements are added ONCE (on arrival); update(now) folds pending factors
 *    into the Bayes tree incrementally; states older than the lag are MARGINALIZED into a
 *    prior (information retained, not discarded — the batch discards).
 *  - Old-information forgetting is provided by the CV process noise q_c (exponential decay of
 *    stale information into the current state — the "filter-style decay" mitigation for
 *    permanently-baked wrong commitments; an explicit fading factor is the follow-up knob if
 *    the A/B ANEES shows the prior mis-calibrating).
 *  - OOSM: a late measurement attaches to the NEAREST active keyframe (ego keyframes are
 *    ~15 ms apart, so the attach error is ≤ ~8 ms — negligible vs the 100–200 ms latency being
 *    compensated); measurements older than the lag margin are dropped (counted).
 *  - Query covariance uses per-key marginals for the position/velocity blocks (cross-block
 *    zero — an approximation, fine at ms-scale forward-prediction; NEES_p unaffected).
 *  - diagnostics(): `iterations` = iSAM2 variables relinearized this update; `error_before` is
 *    not computed (0). error_after / max_factor_error are evaluated over the active factors.
 *  - Failure containment: any gtsam exception during update() resets the smoother and
 *    re-bootstraps from the retained raw-measurement window (acquisition seed = midpoint,
 *    identical policy to the batch backend).
 */
#ifndef MAS_MULTIVIEW_FGO_COOP_SMOOTHER_FL_H
#define MAS_MULTIVIEW_FGO_COOP_SMOOTHER_FL_H

#include "coop_smoother.h"

#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

#include <deque>
#include <list>
#include <map>
#include <vector>

namespace mas_fgo {

class CoopSmootherFL {
public:
    using Params = CoopSmoother::Params;         // window_s doubles as the smoother lag
    using Query = CoopSmoother::Query;
    using Diagnostics = CoopSmoother::Diagnostics;

    explicit CoopSmootherFL(const Params& prm, double reset_period_s = 0.0)
        : prm_(prm), reset_period_s_(reset_period_s) { resetSmoother(); }

    // --- persistent ingestion: call ONCE per measurement, in arrival order ------------------
    void addEgoPixel(double t, const Eigen::Vector2d& px, const Eigen::Matrix3d& K,
                     const Eigen::Matrix3d& R_cam, const Eigen::Vector3d& t_cam, double sigma_px)
    {
        RawMeas m; m.t = t; m.ego = true; m.px = px; m.K = K; m.R = R_cam; m.o = t_cam;
        m.sigma = sigma_px;
        ingest(m);
    }
    void addPeerBearing(double t, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                        const Eigen::Matrix2d& cov2)
    {
        RawMeas m; m.t = t; m.ego = false; m.o = origin; m.dir = dir.normalized(); m.cov2 = cov2;
        ingest(m);
    }

    // --- per-tick update (the timer body) ---------------------------------------------------
    bool update(double now)
    {
        diag_ = Diagnostics{};
        pruneRaw(now);
        // Finite-memory decay (the "filter-style forgetting" mitigation): the marginalized
        // prior accumulates history information as if errors were white, which our
        // time-correlated ego error violates (measured: ANEES ~45 without this). A periodic
        // SOFT reset re-bootstraps from the retained raw window seeded at the current
        // estimate — information memory is bounded to the raw window (batch-equivalent
        // calibration) while iSAM2's incremental cost is kept between resets.
        if (reset_period_s_ > 0.0 && solved_ && now - last_reset_t_ >= reset_period_s_) {
            const bool keep = have_cache_;
            const Eigen::Vector3d cp = cache_p_, cv = cache_v_; const double ct = cache_t_;
            resetSmoother();
            have_cache_ = keep; cache_p_ = cp; cache_v_ = cv; cache_t_ = ct;
            if (keep) boot_seed_ = cp + cv * (now - ct);
            last_reset_t_ = now;
            ++n_soft_resets_;
        }
        if (last_reset_t_ < -1e200) last_reset_t_ = now;
        if (!started_ && !bootstrap()) return false;
        try {
            const auto res = isam_.update(new_factors_, new_values_, new_times_);
            diag_.iterations = static_cast<int>(res.getIterations());
            new_factors_.resize(0); new_values_.clear(); new_times_.clear();
            est_ = isam_.calculateEstimate();
            // prune our key map to the smoother's lag (mirror of its marginalization rule)
            const double cut = t_newest_ - prm_.window_s;
            for (auto it = key_by_time_.begin(); it != key_by_time_.end();) {
                if (it->first < cut) it = key_by_time_.erase(it); else ++it;
            }
            if (!key_by_time_.empty()) {   // refresh the propagation cache from the estimate
                const auto& [tn, kn] = *key_by_time_.rbegin();
                cache_p_ = est_.at<gtsam::Point3>(pkey(kn));
                cache_v_ = est_.at<gtsam::Vector3>(vkey(kn));
                cache_t_ = tn;
                have_cache_ = true;
            }
        } catch (const std::exception&) {
            // reset + re-bootstrap next tick from the retained raw window
            resetSmoother();
            ++n_resets_;
            return false;
        }
        fillDiagnostics();
        solved_ = true;
        diag_.solved = true;
        return true;
    }

    Query query(double t) const
    {
        Query q;
        if (!solved_ || key_by_time_.empty()) return q;
        const auto& [tn, kn] = *key_by_time_.rbegin();
        const Eigen::Vector3d p = est_.at<gtsam::Point3>(pkey(kn));
        const Eigen::Vector3d v = est_.at<gtsam::Vector3>(vkey(kn));
        const double dt = t - tn;
        q.p = p + v * dt;
        q.v = v;
        q.valid = true;
        q.cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
        try {
            Eigen::Matrix<double, 6, 6> P = Eigen::Matrix<double, 6, 6>::Zero();
#ifdef MAS_FGO_FL_FAST_MARGINALS
            // Fast per-key path — MEASURED MIS-CALIBRATED in GTSAM 4.2 (pos-cov ~11x too
            // tight at far range, ~calibrated terminal): ISAM2::marginalCovariance served
            // through the fixed-lag marginalized structure is geometry-stale. Kept only as a
            // future-optimization stub; do NOT ship without re-verifying ANEES.
            P.block<3, 3>(0, 0) = isam_.marginalCovariance(pkey(kn));
            P.block<3, 3>(3, 3) = isam_.marginalCovariance(vkey(kn));
#else
            // Default: full joint marginals over the smoother's current factors — verified
            // batch-equivalent calibration (ANEES_p 1.38 vs 1.37); O(n) per query but still
            // ~12x cheaper end-to-end than the batch backend.
            gtsam::Marginals marg(isam_.getFactors(), est_);
            gtsam::KeyVector keys{pkey(kn), vkey(kn)};
            P = marg.jointMarginalCovariance(keys).fullMatrix();
#endif
            Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
            F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
            q.cov = F * P * F.transpose();
            if (dt > 0) q.cov += CoopSmoother::wnoaCov(prm_.q_c, dt);
        } catch (const std::exception&) {}
        if (prm_.vel_cov_inflation > 1.0) {
            const double sf = std::sqrt(prm_.vel_cov_inflation);
            q.cov.block<3, 3>(3, 3) *= prm_.vel_cov_inflation;
            q.cov.block<3, 3>(0, 3) *= sf;
            q.cov.block<3, 3>(3, 0) *= sf;
        }
        return q;
    }

    const Diagnostics& diagnostics() const { return diag_; }
    int numResets() const { return n_resets_; }
    int numSoftResets() const { return n_soft_resets_; }
    int numDroppedOld() const { return n_drop_old_; }

private:
    struct RawMeas {
        double t; bool ego;
        Eigen::Vector2d px; Eigen::Matrix3d K, R; Eigen::Vector3d o, dir;
        Eigen::Matrix2d cov2; double sigma = 1.0;
    };

    static gtsam::Key pkey(int i) { return gtsam::Symbol('x', i).key(); }
    static gtsam::Key vkey(int i) { return gtsam::Symbol('v', i).key(); }

    void resetSmoother()
    {
        isam_ = gtsam::IncrementalFixedLagSmoother(prm_.window_s);
        new_factors_.resize(0); new_values_.clear(); new_times_.clear();
        key_by_time_.clear(); est_.clear();
        started_ = false; solved_ = false;
        next_key_ = 0; t_newest_ = -1e300;
        fac_stamps_.clear();
    }

    void ingest(const RawMeas& m)
    {
        raw_.push_back(m);                        // retained for bootstrap/reset
        if (!started_) return;                    // bootstrap() will consume raw_
        addFactorFor(m);
    }

    void pruneRaw(double now)
    {
        const double cut = now - prm_.window_s - 0.2;
        while (!raw_.empty() && raw_.front().t < cut) raw_.pop_front();
    }

    // World ray from an ego pixel (same math as CoopSmoother::egoRay / EgoPixelFactor).
    static void egoRay(const RawMeas& m, Eigen::Vector3d& origin, Eigen::Vector3d& dir)
    {
        const Eigen::Matrix3d W2C = (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished();
        Eigen::Vector3d Xc((m.px.x() - m.K(0, 2)) / m.K(0, 0),
                           (m.px.y() - m.K(1, 2)) / m.K(1, 1), 1.0);
        origin = m.o;
        dir = (m.R * W2C.transpose() * Xc).normalized();
    }
    static bool midpoint(const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
                         const Eigen::Vector3d& p2, const Eigen::Vector3d& v2, Eigen::Vector3d& out)
    {
        const double a = v1.dot(v1), b = v1.dot(v2), c = v2.dot(v2);
        const Eigen::Vector3d w0 = p1 - p2;
        const double d = v1.dot(w0), e = v2.dot(w0);
        const double den = a * c - b * b;
        if (std::abs(den) < 1e-9) return false;
        const double sc = (b * e - c * d) / den, tc = (a * e - b * d) / den;
        out = 0.5 * (p1 + sc * v1 + p2 + tc * v2);
        return true;
    }

    bool bootstrap()
    {
        // Keys created in one update must all be INSIDE the lag, or the smoother marginalizes
        // them in the same update they appear (throws). Clamp the re-added span accordingly.
        const double t_last_raw = raw_.empty() ? 0.0 : raw_.back().t;
        const double t_min_add = t_last_raw - (prm_.window_s - 0.05);
        // Soft re-bootstrap: seed from the propagation cache (no cold-start transient).
        if (have_cache_) {
            if (raw_.size() < 2) return false;
            diag_.seed = boot_seed_;
            started_ = true;
            bool anchored = false;
            for (const auto& m : raw_) {
                if (m.t < t_min_add) continue;
                addFactorFor(m, &anchored);
            }
            return anchored;
        }
        // Need one ego + one peer for the midpoint acquisition seed (batch-identical policy).
        const RawMeas* ego = nullptr; const RawMeas* peer = nullptr;
        for (const auto& m : raw_) {
            if (m.ego && !ego) ego = &m;
            if (!m.ego && !peer) peer = &m;
        }
        Eigen::Vector3d seed;
        if (ego && peer) {
            Eigen::Vector3d o1, d1; egoRay(*ego, o1, d1);
            if (!midpoint(o1, d1, peer->o, peer->dir, seed))
                seed = peer->o + 30.0 * peer->dir;
        } else if (peer) { seed = peer->o + 30.0 * peer->dir; }
        else if (ego)    { Eigen::Vector3d o1, d1; egoRay(*ego, o1, d1); seed = o1 + 30.0 * d1; }
        else return false;
        if (raw_.size() < 2) return false;
        diag_.seed = seed;
        started_ = true;
        boot_seed_ = seed;
        bool anchored = false;
        for (const auto& m : raw_) {
            if (m.t < t_min_add) continue;
            addFactorFor(m, &anchored);
        }
        return anchored;
    }

    // Create/lookup the keyframe for stamp t; new keys only extend the chain forward.
    int keyFor(double t, bool* fresh)
    {
        *fresh = false;
        auto it = key_by_time_.find(t);
        if (it != key_by_time_.end()) return it->second;
        if (t > t_newest_ || key_by_time_.empty()) {
            const int k = next_key_++;
            // init at the CV propagation of the cached estimate (or the bootstrap seed) —
            // never read est_ here: keys created since the last update() are not in it yet.
            Eigen::Vector3d p = boot_seed_, v = Eigen::Vector3d::Zero();
            if (have_cache_) {
                p = cache_p_ + cache_v_ * (t - cache_t_);
                v = cache_v_;
            }
            new_values_.insert(pkey(k), gtsam::Point3(p));
            new_values_.insert(vkey(k), gtsam::Vector3(v));
            new_times_[pkey(k)] = t; new_times_[vkey(k)] = t;
            if (!key_by_time_.empty()) {
                const auto& [tp, kp] = *key_by_time_.rbegin();
                const double dt = t - tp;
                new_factors_.add(CVMotionFactor(
                    pkey(kp), vkey(kp), pkey(k), vkey(k), dt,
                    gtsam::noiseModel::Gaussian::Covariance(CoopSmoother::wnoaCov(prm_.q_c, dt))));
            }
            key_by_time_[t] = k;
            t_newest_ = t;
            *fresh = true;
            return k;
        }
        // OOSM: attach to the nearest ACTIVE key inside the lag margin.
        auto up = key_by_time_.lower_bound(t);
        int best = -1; double bd = 1e300;
        if (up != key_by_time_.end() && std::abs(up->first - t) < bd) { bd = std::abs(up->first - t); best = up->second; }
        if (up != key_by_time_.begin()) {
            --up;
            if (std::abs(up->first - t) < bd) { bd = std::abs(up->first - t); best = up->second; }
        }
        if (best < 0 || t < t_newest_ - prm_.window_s + 0.05) { ++n_drop_old_; return -1; }
        return best;
    }

    void addFactorFor(const RawMeas& m, bool* anchor_pending = nullptr)
    {
        bool fresh = false;
        const int k = keyFor(m.t, &fresh);
        if (k < 0) return;
        if (anchor_pending && !*anchor_pending) {
            // weak gauge anchor on the first bootstrap state (batch-identical)
            new_factors_.addPrior(pkey(k), gtsam::Point3(boot_seed_),
                                  gtsam::noiseModel::Isotropic::Sigma(3, prm_.anchor_pos_sigma));
            new_factors_.addPrior(vkey(k), gtsam::Vector3(0, 0, 0),
                                  gtsam::noiseModel::Isotropic::Sigma(3, prm_.anchor_vel_sigma));
            *anchor_pending = true;
        }
        if (m.ego) {
            gtsam::SharedNoiseModel en = gtsam::noiseModel::Isotropic::Sigma(2, m.sigma);
            if (prm_.use_robust_ego) {
                en = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(prm_.huber_k), en);
            }
            new_factors_.add(EgoPixelFactor(pkey(k), m.px, m.K, m.R, m.o, en));
        } else {
            gtsam::SharedNoiseModel bn = gtsam::noiseModel::Gaussian::Covariance(m.cov2);
            if (prm_.use_robust) {
                bn = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Huber::Create(prm_.huber_k), bn);
            }
            new_factors_.add(PeerBearingFactor(pkey(k), m.o, m.dir, bn));
        }
        fac_stamps_.push_back({m.t, m.ego});
    }

    void fillDiagnostics()
    {
        diag_.n_keyframes = static_cast<int>(key_by_time_.size());
        const double cut = t_newest_ - prm_.window_s;
        int ne = 0, np = 0;
        for (auto it = fac_stamps_.begin(); it != fac_stamps_.end();) {
            if (it->first < cut) { it = fac_stamps_.erase(it); continue; }
            (it->second ? ne : np)++;
            ++it;
        }
        diag_.n_ego = ne; diag_.n_peer = np;
        diag_.t_oldest = key_by_time_.empty() ? 0.0 : key_by_time_.begin()->first;
        diag_.t_newest = key_by_time_.empty() ? 0.0 : key_by_time_.rbegin()->first;
        diag_.max_iterations = 0;   // no LM iteration cap in iSAM2 — convergence gate inert
        try {
            const auto& fg = isam_.getFactors();
            double total = 0.0, mfe = 0.0;
            for (const auto& f : fg) {
                if (!f) continue;
                const double e = f->error(est_);
                total += e; mfe = std::max(mfe, e);
            }
            diag_.error_after = total;
            diag_.max_factor_error = mfe;
        } catch (const std::exception&) {}
    }

    Params prm_;
    gtsam::IncrementalFixedLagSmoother isam_{0.6};
    gtsam::NonlinearFactorGraph new_factors_;
    gtsam::Values new_values_, est_;
    gtsam::FixedLagSmoother::KeyTimestampMap new_times_;
    std::map<double, int> key_by_time_;           // active (unmarginalized) keys
    std::deque<RawMeas> raw_;                     // rolling raw window (bootstrap/reset)
    std::list<std::pair<double, bool>> fac_stamps_;
    Eigen::Vector3d boot_seed_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d cache_p_ = Eigen::Vector3d::Zero();   // propagation cache for key init
    Eigen::Vector3d cache_v_ = Eigen::Vector3d::Zero();
    double cache_t_ = 0.0;
    bool have_cache_ = false;
    double t_newest_ = -1e300;
    double reset_period_s_ = 0.0, last_reset_t_ = -1e300;
    int next_key_ = 0, n_resets_ = 0, n_soft_resets_ = 0, n_drop_old_ = 0;
    bool started_ = false, solved_ = false;
    Diagnostics diag_;
};

}  // namespace mas_fgo

#endif  // MAS_MULTIVIEW_FGO_COOP_SMOOTHER_FL_H
