/**
 * @file coop_smoother.h
 * @brief Cooperative target-trajectory smoother (ticket 024) — GTSAM factor graph over target
 *        states x_k=(p_k,v_k) with a CV motion factor and MIXED measurement factors (ego pixel +
 *        peer bearing) placed at each measurement's CAPTURE TIME. Event-keyframed, full-window LM
 *        re-solve each tick (Q3=b; marked for the gtsam_unstable fixed-lag upgrade). Forward-predicts
 *        the newest state to a query time (guidance tick). Publishes pose + velocity (019 blocker).
 *
 * Header-only for the ticket-024 S0 offline core; a thin .cpp / node wraps it for ROS in S1.
 */
#ifndef MAS_MULTIVIEW_FGO_COOP_SMOOTHER_H
#define MAS_MULTIVIEW_FGO_COOP_SMOOTHER_H

#include "factors.h"

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/linear/NoiseModel.h>

#include <algorithm>
#include <map>
#include <vector>

namespace mas_fgo {

class CoopSmoother {
public:
    struct Params {
        double q_c = 4.0;          // white-noise-accel spectral density [ (m/s^2)^2 / Hz ]
        double window_s = 0.6;     // fixed-lag window [s]
        double anchor_pos_sigma = 200.0;   // weak gauge prior on x0 [m]
        double anchor_vel_sigma = 50.0;    // weak gauge prior on v0 [m/s]
        bool use_robust = false;   // Q10 Tier-1 robust kernel on peer bearings
        // RAL 024 S5: robust kernel on the EGO pixel factor — the ego carries the
        // episode-varying bias (S4 audit), so the kernel belongs here; peer-only Huber
        // was measured counterproductive twice (S3 v1 + v2).
        bool use_robust_ego = false;
        double huber_k = 1.345;
        // RAL 024 S4/S5: declared velocity-covariance inflation (structural handoff — the
        // white-noise CV chain cannot represent the time-correlated ego error, leaving the
        // velocity ~10-38x overconfident at any q_c; this scales the published vel block
        // (and cross-block by sqrt) so downstream consumers see an honest vel cov). 1 = off.
        double vel_cov_inflation = 1.0;
        // RAL ticket 028: acquisition fallback range [m] along the first ray when the
        // two-ray midpoint seed is unavailable (ego-only windows always take this path).
        // 30.0 preserves the 024 behavior; the ego-only mode sets the deployed BO-EKF
        // init_range_guess for prior parity.
        double init_range_m = 30.0;
        // RAL ticket 028 S2c: model the episode-varying DC pixel offset as a shared
        // 2-DOF bias state (one key per solve, zero-mean prior sigma bias_sigma_px)
        // instead of pretending per-ray noise is white. Off = 024 behavior.
        bool use_bias_state = false;
        double bias_sigma_px = 200.0;
    };

    struct Query {
        Eigen::Vector3d p;
        Eigen::Vector3d v;
        Eigen::Matrix<double, 6, 6> cov;  // [p;v]
        bool valid = false;
    };

    // Ticket 024 S3 track B (rev1 §11.1): per-solve diagnostics so a live capture
    // (and the offline replay) can SEE the cold-start / local-minimum / no-gate
    // failures rev1 §§4-5 describe, instead of only the published pose/twist.
    struct Diagnostics {
        bool solved = false;
        int iterations = 0;
        int max_iterations = 0;
        int n_keyframes = 0;
        int n_ego = 0;
        int n_peer = 0;
        double error_before = 0.0;   // graph.error(init)
        double error_after = 0.0;    // graph.error(result)
        double max_factor_error = 0.0;   // max single-factor whitened error at the solution
        double t_oldest = 0.0;
        double t_newest = 0.0;
        Eigen::Vector3d seed = Eigen::Vector3d::Zero();  // init seed (midpoint or warm hint)
        bool warm_started = false;                       // S6: init came from the hint
        Eigen::Vector2d bias_est = Eigen::Vector2d::Zero();  // S2c: solved pixel bias
    };
    const Diagnostics& diagnostics() const { return diag_; }

    CoopSmoother() {}
    explicit CoopSmoother(const Params& prm) : prm_(prm) {}

    // RAL 024 S6 warm-start: seed the next solve from a prior belief (p,v at time t) —
    // every keyframe initializes at the CV propagation p + v(t_i - t) instead of the
    // static midpoint, avoiding the cold-start / zero-Jacobian local-minimum trap
    // (rev1 §4). The caller supplies the hint (typically the last GATE-ACCEPTED belief,
    // age-limited); without a hint the midpoint acquisition seed is used unchanged.
    void setInitHint(double t, const Eigen::Vector3d& p, const Eigen::Vector3d& v)
    {
        hint_valid_ = true; hint_t_ = t; hint_p_ = p; hint_v_ = v;
    }

    // RAL ticket 028 S2c (A4, IDP-inspired ADAPTED): 1-DOF depth-memory prior on the
    // newest state — |X - o_ego| ~ N(r_mem, sigma_r^2). One-shot: consumed by the next
    // solve(); the caller re-arms per tick. Self-confirming-feedback risk by design.
    void setRangeMemory(const Eigen::Vector3d& o_ego, double r_mem, double sigma_r)
    {
        rmem_valid_ = true; rmem_o_ = o_ego; rmem_r_ = r_mem; rmem_sigma_ = sigma_r;
    }

    // --- measurement ingestion (each carries its own capture time) --------------------------
    void addEgoPixel(double t, const Eigen::Vector2d& px, const Eigen::Matrix3d& K,
                     const Eigen::Matrix3d& R_cam, const Eigen::Vector3d& t_cam, double sigma_px)
    {
        Meas m; m.t = t; m.type = EGO_PIXEL;
        m.px = px; m.K = K; m.R = R_cam; m.o = t_cam; m.sigma = sigma_px;
        meas_.push_back(m);
    }

    void addPeerBearing(double t, const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
                        const Eigen::Matrix2d& cov2)
    {
        Meas m; m.t = t; m.type = PEER_BEARING;
        m.o = origin; m.dir = dir.normalized(); m.cov2 = cov2;
        meas_.push_back(m);
    }

    // --- solve the windowed graph -----------------------------------------------------------
    bool solve()
    {
        diag_ = Diagnostics{};
        if (meas_.size() < 2) return false;

        // Window prune.
        double t_max = -1e300;
        for (const auto& m : meas_) t_max = std::max(t_max, m.t);
        const double t_cut = t_max - prm_.window_s;
        std::vector<Meas> win;
        for (const auto& m : meas_) if (m.t >= t_cut) win.push_back(m);
        if (win.size() < 2) return false;

        // Unique sorted keyframe times.
        std::vector<double> times;
        for (const auto& m : win) times.push_back(m.t);
        std::sort(times.begin(), times.end());
        times.erase(std::unique(times.begin(), times.end(),
                    [](double a, double b) { return std::abs(a - b) < 1e-9; }), times.end());
        if (times.size() < 2) return false;
        std::map<double, int> kf;  // time -> keyframe index
        for (size_t i = 0; i < times.size(); ++i) kf[times[i]] = static_cast<int>(i);
        const int n = static_cast<int>(times.size());
        diag_.n_keyframes = n;
        diag_.t_oldest = times.front();
        diag_.t_newest = times.back();

        // Initialization: warm hint (CV-propagated prior belief) when supplied, else the
        // midpoint acquisition seed (first ego+peer ray pair).
        Eigen::Vector3d p0_guess;
        Eigen::Vector3d v0_guess = Eigen::Vector3d::Zero();
        bool warm = hint_valid_;
        if (warm) {
            p0_guess = hint_p_ + hint_v_ * (times.front() - hint_t_);
            v0_guess = hint_v_;
        } else if (!initialGuess(win, p0_guess)) {
            return false;
        }
        diag_.seed = p0_guess;
        diag_.warm_started = warm;

        gtsam::NonlinearFactorGraph graph;
        gtsam::Values init;
        for (int i = 0; i < n; ++i) {
            const Eigen::Vector3d pi =
                warm ? Eigen::Vector3d(hint_p_ + hint_v_ * (times[i] - hint_t_)) : p0_guess;
            init.insert(pkey(i), gtsam::Point3(pi));
            init.insert(vkey(i), gtsam::Vector3(v0_guess));
        }
        // RAL 028 S2c: shared pixel-bias state (one per solve) with a zero-mean prior.
        if (prm_.use_bias_state) {
            init.insert(bkey(), gtsam::Vector2(0.0, 0.0));
            graph.addPrior(bkey(), gtsam::Vector2(0.0, 0.0),
                           gtsam::noiseModel::Isotropic::Sigma(2, prm_.bias_sigma_px));
        }

        // CV motion chain.
        for (int i = 0; i + 1 < n; ++i) {
            const double dt = times[i + 1] - times[i];
            graph.add(CVMotionFactor(pkey(i), vkey(i), pkey(i + 1), vkey(i + 1), dt,
                                     cvNoise(dt)));
        }

        // Weak anchor prior (gauge / conditioning) on the first state (at the init values,
        // warm or cold — the anchor is a conditioner, not an information source).
        graph.addPrior(pkey(0), gtsam::Point3(p0_guess),
                       gtsam::noiseModel::Isotropic::Sigma(3, prm_.anchor_pos_sigma));
        graph.addPrior(vkey(0), gtsam::Vector3(v0_guess),
                       gtsam::noiseModel::Isotropic::Sigma(3, prm_.anchor_vel_sigma));

        // Measurement factors at their keyframe.
        for (const auto& m : win) {
            const int i = kf[nearestTime(times, m.t)];
            if (m.type == EGO_PIXEL) {
                gtsam::SharedNoiseModel en = gtsam::noiseModel::Isotropic::Sigma(2, m.sigma);
                if (prm_.use_robust_ego) {
                    en = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Huber::Create(prm_.huber_k), en);
                }
                if (prm_.use_bias_state) {  // RAL 028 S2c
                    graph.add(EgoPixelBiasFactor(pkey(i), bkey(), m.px, m.K, m.R, m.o, en));
                } else {
                    graph.add(EgoPixelFactor(pkey(i), m.px, m.K, m.R, m.o, en));
                }
                ++diag_.n_ego;
            } else {
                gtsam::SharedNoiseModel bn = gtsam::noiseModel::Gaussian::Covariance(m.cov2);
                if (prm_.use_robust) {
                    bn = gtsam::noiseModel::Robust::Create(
                        gtsam::noiseModel::mEstimator::Huber::Create(prm_.huber_k), bn);
                }
                graph.add(PeerBearingFactor(pkey(i), m.o, m.dir, bn));
                ++diag_.n_peer;
            }
        }

        // RAL 028 S2c (A4): one-shot depth-memory range prior on the newest state.
        if (rmem_valid_) {
            graph.add(RangePriorFactor(pkey(n - 1), rmem_o_, rmem_r_,
                                       gtsam::noiseModel::Isotropic::Sigma(1, rmem_sigma_)));
            rmem_valid_ = false;
        }

        gtsam::LevenbergMarquardtParams lm;
        lm.setMaxIterations(100);
        diag_.max_iterations = 100;
        diag_.error_before = graph.error(init);
        gtsam::LevenbergMarquardtOptimizer opt(graph, init, lm);
        result_ = opt.optimize();
        diag_.iterations = static_cast<int>(opt.iterations());
        diag_.error_after = graph.error(result_);
        if (prm_.use_bias_state) diag_.bias_est = result_.at<gtsam::Vector2>(bkey());
        double mfe = 0.0;
        for (const auto& f : graph) if (f) mfe = std::max(mfe, f->error(result_));
        diag_.max_factor_error = mfe;
        graph_ = graph;
        times_ = times;
        solved_ = true;
        diag_.solved = true;
        return true;
    }

    // --- query the belief at time t (forward-predicted from the newest keyframe) -------------
    Query query(double t) const
    {
        Query q;
        if (!solved_ || times_.empty()) return q;
        const int n = static_cast<int>(times_.size()) - 1;
        const Eigen::Vector3d p = result_.at<gtsam::Point3>(pkey(n));
        const Eigen::Vector3d v = result_.at<gtsam::Vector3>(vkey(n));
        const double dt = t - times_.back();
        q.p = p + v * dt;
        q.v = v;
        q.valid = true;

        // Covariance: joint 6x6 marginal of the newest state, CV-propagated to t.
        try {
            gtsam::Marginals marg(graph_, result_);
            gtsam::KeyVector keys{pkey(n), vkey(n)};
            Eigen::Matrix<double, 6, 6> P = marg.jointMarginalCovariance(keys).fullMatrix();
            Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
            F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
            q.cov = F * P * F.transpose();
            if (dt > 0) q.cov += cvCov(dt);
        } catch (const std::exception&) {
            q.cov = Eigen::Matrix<double, 6, 6>::Identity() * 1e6;
        }
        if (prm_.vel_cov_inflation > 1.0) {
            // cov' = S cov S with S = diag(I, sqrt(f) I): vel block x f, cross x sqrt(f) — PSD kept.
            const double sf = std::sqrt(prm_.vel_cov_inflation);
            q.cov.block<3, 3>(3, 3) *= prm_.vel_cov_inflation;
            q.cov.block<3, 3>(0, 3) *= sf;
            q.cov.block<3, 3>(3, 0) *= sf;
        }
        return q;
    }

    // Belief at the newest keyframe (no forward prediction) — for offline checks.
    Query queryLatest() const { return solved_ ? query(times_.back()) : Query(); }

    size_t numKeyframes() const { return times_.size(); }

private:
    enum MeasType { EGO_PIXEL, PEER_BEARING };
    struct Meas {
        double t; MeasType type;
        Eigen::Vector2d px;
        Eigen::Matrix3d K, R;
        Eigen::Vector3d o, dir;
        Eigen::Matrix2d cov2;
        double sigma = 1.0;
    };

    static gtsam::Key pkey(int i) { return gtsam::Symbol('x', i).key(); }
    static gtsam::Key vkey(int i) { return gtsam::Symbol('v', i).key(); }
    static gtsam::Key bkey()      { return gtsam::Symbol('b', 0).key(); }  // S2c bias

    static double nearestTime(const std::vector<double>& times, double t)
    {
        double best = times[0], bd = 1e300;
        for (double tt : times) { double d = std::abs(tt - t); if (d < bd) { bd = d; best = tt; } }
        return best;
    }

    // 6x6 white-noise-acceleration process covariance over dt (public static: the S5 output
    // gate's fallback forward-prediction grows the held belief with the same process noise).
public:
    static Eigen::Matrix<double, 6, 6> wnoaCov(double q_c, double dt)
    {
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 6, 6> Q = Eigen::Matrix<double, 6, 6>::Zero();
        const double d = std::max(dt, 1e-4);
        Q.block<3, 3>(0, 0) = q_c * (d * d * d / 3.0) * I;
        Q.block<3, 3>(0, 3) = q_c * (d * d / 2.0) * I;
        Q.block<3, 3>(3, 0) = q_c * (d * d / 2.0) * I;
        Q.block<3, 3>(3, 3) = q_c * d * I;
        return Q;
    }
private:
    Eigen::Matrix<double, 6, 6> cvCov(double dt) const { return wnoaCov(prm_.q_c, dt); }

    gtsam::SharedNoiseModel cvNoise(double dt) const
    {
        return gtsam::noiseModel::Gaussian::Covariance(cvCov(dt));
    }

    // Ego pixel -> world ray (origin + unit dir): X - t = M^{-1} Xc, M = W2C R^T, M^{-1} = R W2C^T.
    static void egoRay(const Meas& m, Eigen::Vector3d& origin, Eigen::Vector3d& dir)
    {
        const Eigen::Matrix3d W2C = (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished();
        Eigen::Vector3d Xc((m.px.x() - m.K(0, 2)) / m.K(0, 0),
                           (m.px.y() - m.K(1, 2)) / m.K(1, 1), 1.0);
        origin = m.o;
        dir = (m.R * W2C.transpose() * Xc).normalized();
    }

    // Midpoint of two skew rays (closest points of approach), like ticket 020 estimateInitialPosition.
    static bool midpoint(const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
                         const Eigen::Vector3d& p2, const Eigen::Vector3d& v2, Eigen::Vector3d& out)
    {
        const double a = v1.dot(v1), b = v1.dot(v2), c = v2.dot(v2);
        const Eigen::Vector3d w0 = p1 - p2;
        const double d = v1.dot(w0), e = v2.dot(w0);
        const double den = a * c - b * b;
        if (std::abs(den) < 1e-9) return false;   // near-parallel
        const double sc = (b * e - c * d) / den;
        const double tc = (a * e - b * d) / den;
        out = 0.5 * (p1 + sc * v1 + p2 + tc * v2);
        return true;
    }

    bool initialGuess(const std::vector<Meas>& win, Eigen::Vector3d& out) const
    {
        const Meas* ego = nullptr; const Meas* peer = nullptr;
        for (const auto& m : win) {
            if (!ego && m.type == EGO_PIXEL) ego = &m;
            if (!peer && m.type == PEER_BEARING) peer = &m;
        }
        if (ego && peer) {
            Eigen::Vector3d o1, d1; egoRay(*ego, o1, d1);
            if (midpoint(o1, d1, peer->o, peer->dir, out)) return true;
        }
        // Fallback: nominal range along the first available ray (RAL ticket 028: prm_.init_range_m).
        if (peer) { out = peer->o + prm_.init_range_m * peer->dir; return true; }
        if (ego)  { Eigen::Vector3d o1, d1; egoRay(*ego, o1, d1);
                    out = o1 + prm_.init_range_m * d1; return true; }
        return false;
    }

    Params prm_;
    std::vector<Meas> meas_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values result_;
    std::vector<double> times_;
    bool solved_ = false;
    Diagnostics diag_;
    bool hint_valid_ = false;            // S6 warm-start hint
    double hint_t_ = 0.0;
    Eigen::Vector3d hint_p_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d hint_v_ = Eigen::Vector3d::Zero();
    bool rmem_valid_ = false;            // S2c A4 depth-memory prior (one-shot)
    Eigen::Vector3d rmem_o_ = Eigen::Vector3d::Zero();
    double rmem_r_ = 0.0, rmem_sigma_ = 30.0;
};

}  // namespace mas_fgo

#endif  // MAS_MULTIVIEW_FGO_COOP_SMOOTHER_H
