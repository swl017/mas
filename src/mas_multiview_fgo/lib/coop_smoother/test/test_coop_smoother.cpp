/**
 * @file test_coop_smoother.cpp
 * @brief Ticket 024 S0 offline unit test (GPU-free) for the cooperative GTSAM smoother.
 *
 * Decisive scenario — measurement-time-aware fusion beats a latency-blind snapshot:
 *   - CV target p(t) = p0 + v0*t (constant velocity).
 *   - Ego camera (raw pixel factor) observes the target FRESH, at times up to t_now.
 *   - Peer transmits bearings, but its latest bearing is STALE by tau (last peer at t_now - tau).
 *   - The smoother places each measurement at its capture time + a CV motion factor; a snapshot
 *     fuses fresh-ego(t_now) x stale-peer(t_now - tau) as if simultaneous (the ticket 020 v*tau bias).
 * Checks: recover (p,v) at t_now within tol; BEAT the snapshot; covariance PD; pose-interp (Q8);
 * Q9 hybrid-noise builder runs + PD. Process returns non-zero on any failure.
 */
#include "coop_smoother.h"
#include "pose_interp.h"
#include "meas_noise.h"

#include <cmath>
#include <iostream>
#include <deque>

using namespace mas_fgo;

namespace {
int g_fail = 0;
void check(bool c, const std::string& name, const std::string& detail = "")
{
    std::cout << (c ? "  [PASS] " : "  [FAIL] ") << name;
    if (!detail.empty()) std::cout << "  (" << detail << ")";
    std::cout << std::endl;
    if (!c) ++g_fail;
}

const Eigen::Matrix3d W2C =
    (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished();

Eigen::Matrix3d intrinsics()
{
    Eigen::Matrix3d K;
    K << 600, 0, 320, 0, 600, 240, 0, 0, 1;
    return K;
}

Eigen::Vector2d projectPixel(const Eigen::Matrix3d& K, const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& t, const Eigen::Vector3d& X)
{
    const Eigen::Vector3d Xc = W2C * R.transpose() * (X - t);
    return Eigen::Vector2d(K(0, 0) * Xc.x() / Xc.z() + K(0, 2),
                           K(1, 1) * Xc.y() / Xc.z() + K(1, 2));
}

// Midpoint of two rays (for the snapshot baseline).
Eigen::Vector3d midpoint(const Eigen::Vector3d& p1, const Eigen::Vector3d& v1,
                         const Eigen::Vector3d& p2, const Eigen::Vector3d& v2)
{
    const double a = v1.dot(v1), b = v1.dot(v2), c = v2.dot(v2);
    const Eigen::Vector3d w0 = p1 - p2;
    const double d = v1.dot(w0), e = v2.dot(w0);
    const double den = a * c - b * b;
    const double sc = (b * e - c * d) / den, tc = (a * e - b * d) / den;
    return 0.5 * (p1 + sc * v1 + p2 + tc * v2);
}
}  // namespace

int main()
{
    std::cout << "=== Ticket 024 S0: cooperative GTSAM smoother ===" << std::endl;

    // --- scene ---------------------------------------------------------------------------------
    const Eigen::Vector3d p0(40.0, -10.0, 2.0);
    const Eigen::Vector3d v0(-1.0, 8.0, 0.0);           // moving mainly +Y
    auto gt = [&](double t) { return p0 + v0 * t; };

    const Eigen::Matrix3d K = intrinsics();
    const Eigen::Matrix3d Rego = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d ego_cam(0.0, 0.0, 0.0);       // ego camera at origin, looking +X
    const Eigen::Vector3d peer_pos(40.0, 40.0, 0.0);    // peer off to +Y -> good parallax

    const double t_now = 0.50;                          // peer stale by tau = 0.20 s (last at 0.30)
    const std::vector<double> ego_t = {0.30, 0.35, 0.40, 0.45, 0.50};   // fresh
    const std::vector<double> peer_t = {0.20, 0.25, 0.30};              // last = t_now - 0.20

    // --- build the smoother ---------------------------------------------------------------------
    CoopSmoother::Params prm;
    prm.q_c = 4.0; prm.window_s = 1.0;
    CoopSmoother sm(prm);

    PeerNoiseParams np; np.sigma_static_rad = 0.5 * M_PI / 180.0;  // 0.5 deg detector-grade

    for (double t : ego_t)
        sm.addEgoPixel(t, projectPixel(K, Rego, ego_cam, gt(t)), K, Rego, ego_cam, 1.0);
    for (double t : peer_t) {
        const Eigen::Vector3d dir = (gt(t) - peer_pos).normalized();
        // Q9 hybrid noise (isotropic R_static here; exercise the builder path).
        PeerBearingFactor tmp(0, peer_pos, dir, gtsam::noiseModel::Isotropic::Sigma(2, 1.0));
        Eigen::Matrix2d R = buildPeerBearingCov(np, tmp.u1(), tmp.u2(), peer_pos, dir, gt(t));
        sm.addPeerBearing(t, peer_pos, dir, R);
    }

    const bool ok = sm.solve();
    check(ok, "smoother solve() succeeds", "keyframes=" + std::to_string(sm.numKeyframes()));
    if (!ok) { std::cout << "=== ABORT ===" << std::endl; return 1; }

    // --- Check 1: recover position + velocity at t_now ------------------------------------------
    std::cout << "[1] recover (p,v) at t_now = " << t_now << std::endl;
    const CoopSmoother::Query q = sm.query(t_now);
    const Eigen::Vector3d gt_now = gt(t_now);
    const double perr = (q.p - gt_now).norm();
    const double verr = (q.v - v0).norm();
    check(q.valid, "query valid");
    check(perr < 0.5, "fused position within 0.5 m of GT",
          "err=" + std::to_string(perr) + " m  p=(" + std::to_string(q.p.x()) + "," +
          std::to_string(q.p.y()) + "," + std::to_string(q.p.z()) + ")");
    check(verr < 1.0, "recovered velocity within 1.0 m/s of GT (019 blocker: v is produced)",
          "err=" + std::to_string(verr) + " m/s  v=(" + std::to_string(q.v.x()) + "," +
          std::to_string(q.v.y()) + "," + std::to_string(q.v.z()) + ")");

    // --- Check 2: beat the latency-blind snapshot ----------------------------------------------
    std::cout << "[2] smoother beats the fresh-ego x stale-peer snapshot" << std::endl;
    // Ego ray at t_now (fresh) and peer ray at its stale time, fused as if simultaneous.
    Eigen::Vector3d ego_o = ego_cam;
    Eigen::Vector3d ego_dir;
    {
        const Eigen::Vector2d px = projectPixel(K, Rego, ego_cam, gt(t_now));
        Eigen::Vector3d Xc((px.x() - K(0, 2)) / K(0, 0), (px.y() - K(1, 2)) / K(1, 1), 1.0);
        ego_dir = (Rego * W2C.transpose() * Xc).normalized();
    }
    const double t_peer_last = peer_t.back();
    const Eigen::Vector3d peer_dir_stale = (gt(t_peer_last) - peer_pos).normalized();
    const Eigen::Vector3d snap = midpoint(ego_o, ego_dir, peer_pos, peer_dir_stale);
    const double snap_err = (snap - gt_now).norm();
    std::cout << "    snapshot err=" << snap_err << " m,  smoother err=" << perr
              << " m,  ratio=" << (snap_err / std::max(perr, 1e-9)) << "x" << std::endl;
    check(snap_err > 0.05, "snapshot carries a real v*tau staleness bias (scenario exercises it)",
          "snap_err=" + std::to_string(snap_err) + " m");
    check(perr < 0.4 * snap_err, "smoother beats the snapshot by a decisive margin",
          "smoother=" + std::to_string(perr) + " m vs snapshot=" + std::to_string(snap_err) + " m");

    // --- Check 3: covariance finite + positive-definite ----------------------------------------
    std::cout << "[3] forward-predicted covariance is finite + PD" << std::endl;
    check(q.cov.allFinite(), "covariance finite");
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> es(q.cov);
    const double lam = es.eigenvalues().minCoeff();
    check(lam > 0.0, "covariance positive-definite", "lambda_min=" + std::to_string(lam));
    check(q.cov.trace() < 1e5, "covariance is a real estimate (not the failure sentinel)",
          "trace=" + std::to_string(q.cov.trace()));

    // --- Check 4: Q8 pose interpolation --------------------------------------------------------
    std::cout << "[4] pose interpolation to t_det (Q8)" << std::endl;
    std::deque<PoseSample> buf;
    for (int i = 0; i <= 10; ++i) {
        double t = 0.30 + 0.02 * i;
        PoseSample s; s.t = t;
        s.p = Eigen::Vector3d(1.0 * t, -2.0 * t, 0.5);          // smooth self-motion
        double ang = 0.3 * t;                                    // smooth yaw
        s.q = Eigen::Quaterniond(Eigen::AngleAxisd(ang, Eigen::Vector3d::UnitZ()));
        buf.push_back(s);
    }
    const double t_det = 0.373;  // between samples
    InterpPose ip = interpolatePose(buf, t_det);
    Eigen::Vector3d p_true(1.0 * t_det, -2.0 * t_det, 0.5);
    double ang_true = 0.3 * t_det;
    Eigen::Quaterniond q_true(Eigen::AngleAxisd(ang_true, Eigen::Vector3d::UnitZ()));
    const double pinterp_err = (ip.p - p_true).norm();
    const double qinterp_err = ip.q.angularDistance(q_true);
    check(ip.valid && !ip.extrapolated, "interpolation is bracketed (not extrapolated)");
    check(pinterp_err < 1e-3, "interpolated position matches", "err=" + std::to_string(pinterp_err));
    check(qinterp_err < 1e-3, "interpolated attitude (SLERP) matches",
          "err=" + std::to_string(qinterp_err) + " rad");
    InterpPose ip_ex = interpolatePose(buf, 0.9);
    check(ip_ex.extrapolated, "out-of-span query is flagged extrapolated (guard)");

    // --- Check 5: Q9 hybrid noise builder finite + PD ------------------------------------------
    std::cout << "[5] Q9 hybrid peer-bearing covariance (with transmitted EKF2 cov) is PD" << std::endl;
    {
        const Eigen::Vector3d dir = (gt_now - peer_pos).normalized();
        PeerBearingFactor tmp(0, peer_pos, dir, gtsam::noiseModel::Isotropic::Sigma(2, 1.0));
        PeerNoiseParams p2; p2.sigma_static_rad = 0.5 * M_PI / 180.0; p2.sigma_psi_rad = 1.0 * M_PI / 180.0;
        Eigen::Matrix3d Satt = Eigen::Matrix3d::Identity() * std::pow(0.3 * M_PI / 180.0, 2);
        Eigen::Matrix3d Spos = Eigen::Matrix3d::Identity() * 0.25;  // 0.5 m std GPS-INS
        Eigen::Matrix2d R = buildPeerBearingCov(p2, tmp.u1(), tmp.u2(), peer_pos, dir, gt_now, Satt, Spos);
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> es2(R);
        check(R.allFinite() && es2.eigenvalues().minCoeff() > 0.0,
              "R_theta_eff (R_static + azimuth infl + R_att + Sigma_o proj) is PD",
              "lambda_min=" + std::to_string(es2.eigenvalues().minCoeff()));
    }

    std::cout << "=== " << (g_fail == 0 ? "ALL CHECKS PASSED" : "FAILURES: " + std::to_string(g_fail))
              << " ===" << std::endl;
    return g_fail == 0 ? 0 : 1;
}
