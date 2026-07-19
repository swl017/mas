/**
 * @file output_gate.h
 * @brief RAL ticket 024 S5 — output-safety gate + last-valid fallback for the cooperative
 *        smoother belief (rev1 §5: never publish an unvetted solve to PN).
 *
 * Shared by coop_smoother_node (live) and replay_coop_smoother (offline) so the replay drives
 * the byte-identical gate logic. Threshold defaults are DATA-DERIVED from the S4-accepted
 * config on the v2 replay (5 nominal capture bags):
 *   - the >10 m error ticks are ego-only (n_peer=0) with pos-cov trace 3450–7395 m² vs
 *     p95 ≤ 82 m² on good ticks → min_peer + cov-trace are the load-bearing gates;
 *   - solver-quality tails are benign at the S4 config (avg whitened err p99 ≤ 3.9,
 *     max factor err p99 ≤ 69, never hits max iterations) → loose safety-net thresholds.
 * Fallback: re-publish the last accepted belief forward-predicted with the CV model, its
 * covariance grown by the same WNOA process noise, for at most hold_s; then go silent
 * (matching the incumbent pipeline's behavior when its estimator drops).
 */
#ifndef MAS_MULTIVIEW_FGO_OUTPUT_GATE_H
#define MAS_MULTIVIEW_FGO_OUTPUT_GATE_H

#include "coop_smoother.h"

#include <Eigen/Eigen>
#include <cmath>

namespace mas_fgo {

struct GateParams {
    int min_peer = 1;               // structural: ego-only windows are range-unobservable
    double max_avg_werr = 10.0;     // err_after / n_factors safety net (S4 p99 ≤ 3.9)
    double max_fac_err = 200.0;     // max single whitened factor error (S4 p99 ≤ 69)
    double max_pos_cov_tr = 100.0;  // m^2 — the observability gate (S4 good-tick p95 ≤ 82)
    double v_max = 30.0;            // m/s — incumbent-pipeline parity cap
    double max_jump_m = 5.0;        // vs the forward-predicted last accepted belief
    double min_range_m = 1.0;       // plausibility vs ego position (skipped if ego unknown)
    double max_range_m = 300.0;
    double hold_s = 1.0;            // fallback horizon
    double q_c = 4.0;               // WNOA density for fallback cov growth
};

enum GateReason {
    GATE_PASS = 0,
    GATE_NOT_SOLVED = 1 << 0,
    GATE_MIN_PEER = 1 << 1,
    GATE_NOT_CONVERGED = 1 << 2,
    GATE_AVG_WERR = 1 << 3,
    GATE_MAX_FAC = 1 << 4,
    GATE_COV_TR = 1 << 5,
    GATE_VEL_CAP = 1 << 6,
    GATE_JUMP = 1 << 7,
    GATE_RANGE = 1 << 8,
};

struct GateState {
    bool have_last = false;
    double t_last = 0.0;
    Eigen::Vector3d p_last = Eigen::Vector3d::Zero();
    Eigen::Vector3d v_last = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> cov_last = Eigen::Matrix<double, 6, 6>::Identity();
};

struct GateOutput {
    bool publish = false;    // anything to publish this tick
    bool fallback = false;   // published belief is the held/predicted one
    int reason = GATE_PASS;  // bitmask of failed checks (0 = accepted)
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Matrix<double, 6, 6> cov = Eigen::Matrix<double, 6, 6>::Identity();
};

/**
 * @brief Vet this tick's solve; on acceptance update `st`, else produce the held fallback.
 * @param ego_pos     latest ego position for the range plausibility check
 * @param have_ego    false -> skip the range check
 */
inline GateOutput applyOutputGate(const CoopSmoother::Diagnostics& d, bool solved,
                                  const CoopSmoother::Query& q, double now,
                                  const Eigen::Vector3d& ego_pos, bool have_ego,
                                  const GateParams& gp, GateState& st)
{
    GateOutput out;
    int r = GATE_PASS;
    if (!solved || !q.valid) r |= GATE_NOT_SOLVED;
    if (d.n_peer < gp.min_peer) r |= GATE_MIN_PEER;
    if (d.max_iterations > 0 && d.iterations >= d.max_iterations) r |= GATE_NOT_CONVERGED;
    const int n_fac = d.n_ego + d.n_peer + std::max(0, d.n_keyframes - 1) + 2;
    if (n_fac > 0 && d.error_after / n_fac > gp.max_avg_werr) r |= GATE_AVG_WERR;
    if (d.max_factor_error > gp.max_fac_err) r |= GATE_MAX_FAC;
    if (solved && q.valid) {
        const double ctr = q.cov.block<3, 3>(0, 0).trace();
        if (!std::isfinite(ctr) || ctr > gp.max_pos_cov_tr) r |= GATE_COV_TR;
        if (q.v.norm() > gp.v_max) r |= GATE_VEL_CAP;
        if (st.have_last) {
            const double dt = now - st.t_last;
            if (dt >= 0 && dt <= gp.hold_s) {
                const Eigen::Vector3d pred = st.p_last + st.v_last * dt;
                if ((q.p - pred).norm() > gp.max_jump_m) r |= GATE_JUMP;
            }
        }
        if (have_ego) {
            const double rng = (q.p - ego_pos).norm();
            if (rng < gp.min_range_m || rng > gp.max_range_m) r |= GATE_RANGE;
        }
    }
    out.reason = r;

    if (r == GATE_PASS) {
        out.publish = true;
        out.p = q.p; out.v = q.v; out.cov = q.cov;
        st.have_last = true; st.t_last = now;
        st.p_last = q.p; st.v_last = q.v; st.cov_last = q.cov;
        return out;
    }
    // Fallback: hold + CV-predict the last accepted belief within the hold horizon.
    if (st.have_last) {
        const double dt = now - st.t_last;
        if (dt >= 0.0 && dt <= gp.hold_s) {
            out.publish = true; out.fallback = true;
            out.p = st.p_last + st.v_last * dt;
            out.v = st.v_last;
            Eigen::Matrix<double, 6, 6> F = Eigen::Matrix<double, 6, 6>::Identity();
            F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity() * dt;
            out.cov = F * st.cov_last * F.transpose() + CoopSmoother::wnoaCov(gp.q_c, dt);
        }
    }
    return out;
}

}  // namespace mas_fgo

#endif  // MAS_MULTIVIEW_FGO_OUTPUT_GATE_H
