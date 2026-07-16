/**
 * @file test_transmitted_ray.cpp
 * @brief Offline unit test for the transmitted-ray fusion residual (ticket 020, slice S0).
 *
 * A cooperative peer transmits a bearing RAY (origin + unit direction) with no local image
 * model (no K_). Before this fix, such a ray only seeded the initial midpoint and then dropped
 * out of the Ceres refinement, the results gate, and the covariance — so a {raw ego + peer ray}
 * pair produced NO fused point. This test proves the ray is now a first-class constraint.
 *
 * Geometry (all in the z = 0 plane, world/ENU-like frame):
 *   - GT target at (10, 0, 0).
 *   - Ego camera 0 (raw) at the origin, looking along +X. Its LOS is the entire X-axis, so the
 *     ego pixel constrains the two DOF perpendicular to +X but gives ZERO range information.
 *     Therefore a correct fused RANGE can only come from the transmitted peer ray.
 *   - Peer camera 1 (precomputed ray) at (10, 10, 0), transmitting the LOS to GT ((0,-1,0)),
 *     ~90 deg parallax.
 * When the peer ray direction is rotated by delta about +Z, it still intersects the ego LOS at
 * x = 10 + 10*tan(delta) (planar geometry), giving an analytic ground truth for the recovered
 * range as a function of the ray. The fused point must track it — the decisive "live constraint,
 * not just an initial guess" check.
 *
 * Checks (all must pass; process returns non-zero on any failure):
 *   1. Existence + accuracy: {raw + ray} yields exactly one fused point within tol of GT.
 *   2. Live constraint: sweeping the peer ray direction moves the fused point monotonically and
 *      quantitatively along the ego LOS, matching x = 10 + 10*tan(delta).
 *   3. Covariance finite + positive-definite for good geometry.
 *   4. Covariance degrades (parallax -> inf) as the two rays approach parallel.
 */

#include "multiview_triangulation.h"

#include <cmath>
#include <iostream>
#include <vector>

using MultiView::Camera;
using MultiView::CovarianceConfig;
using MultiView::MultiViewTriangulation;

namespace {

int g_failures = 0;

void check(bool cond, const std::string& name, const std::string& detail = "")
{
    std::cout << (cond ? "  [PASS] " : "  [FAIL] ") << name;
    if (!detail.empty()) std::cout << "  (" << detail << ")";
    std::cout << std::endl;
    if (!cond) ++g_failures;
}

double deg2rad(double d) { return d * M_PI / 180.0; }

Eigen::Matrix3d intrinsics()
{
    Eigen::Matrix3d K;
    K << 600.0,   0.0, 320.0,
           0.0, 600.0, 240.0,
           0.0,   0.0,   1.0;
    return K;
}

// Replicate ReprojectionError::project so the ego pixel is exactly consistent with the model.
Eigen::Vector2d projectPixel(const Eigen::Matrix3d& K,
                             const Eigen::Matrix3d& R,
                             const Eigen::Vector3d& t,
                             const Eigen::Vector3d& X)
{
    const Eigen::Matrix3d W2C = (Eigen::Matrix3d() << 0, -1, 0,
                                                       0,  0, -1,
                                                       1,  0,  0).finished();
    Eigen::Vector3d pc = R.transpose() * (X - t);
    Eigen::Vector3d rot = W2C * pc;
    double x = rot.x() / rot.z();
    double y = rot.y() / rot.z();
    return Eigen::Vector2d(K(0, 0) * x + K(0, 2), K(1, 1) * y + K(1, 2));
}

// Build a {raw ego + precomputed peer} triangulator and solve for the given peer ray direction.
// Returns true and fills `out` if a single fused point is produced.
bool triangulateEgoPlusRay(const Eigen::Vector3d& gt,
                           const Eigen::Vector3d& peer_origin,
                           const Eigen::Vector3d& peer_dir,
                           double bearing_sigma_rad,
                           MultiView::Detection::Detection3D& out)
{
    const Eigen::Matrix3d K = intrinsics();
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const Eigen::Vector3d ego_t(0.0, 0.0, 0.0);

    MultiViewTriangulation tri;
    tri.addCamera(0);
    tri.addCamera(1);

    // Ego camera 0 (raw): intrinsics/extrinsics MUST be set before addDetection (it builds the ray).
    tri.setCameraIntrinsics(0, K);
    tri.setCameraExtrinsics(0, I, ego_t);
    tri.setCameraWidthHeight(0, 640, 480);
    Eigen::Vector2d pix = projectPixel(K, I, ego_t, gt);
    MultiView::Detection::Detection2D det;
    det.center = pix;
    det.width = 20.0;
    det.height = 20.0;
    det.class_id = "ego0";
    det.confidence = 1.0f;
    tri.addDetection(0, det);

    // Peer camera 1 (precomputed transmitted ray): extrinsics set the origin (for covariance),
    // then the ray is added, then the fusion-side bearing sigma.
    tri.setCameraExtrinsics(1, I, peer_origin);
    tri.addPrecomputedRay(1, peer_origin, peer_dir, "peer1");
    tri.setCameraBearingSigma(1, bearing_sigma_rad);

    // Enable full pose-uncertainty covariance path with modest input stds.
    CovarianceConfig cfg;
    cfg.use_pose_covariance = false;   // no EKF cov here -> use pos_std/ori_std fallbacks
    tri.setCovarianceConfig(cfg);

    // max_reprojection_error mixes pixels (ego) and metres (peer ray distance); at the true
    // intersection both are ~0, so a generous common gate admits the point (documented v1 limit).
    tri.Triangulate(0.1, 50.0);

    if (tri.results_.size() != 1) return false;
    out = tri.results_[0];
    return true;
}

}  // namespace

int main()
{
    const Eigen::Vector3d gt(10.0, 0.0, 0.0);
    const Eigen::Vector3d peer_origin(10.0, 10.0, 0.0);
    const Eigen::Vector3d peer_dir = (gt - peer_origin).normalized();  // (0,-1,0)
    const double sigma = deg2rad(0.5);

    std::cout << "=== Ticket 020 S0: transmitted-ray fusion residual ===" << std::endl;

    // ---- Check 1: existence + accuracy (old code: NO point) ----------------------------------
    std::cout << "[1] {raw ego + transmitted peer ray} produces a fused point near GT" << std::endl;
    MultiView::Detection::Detection3D fused;
    bool produced = triangulateEgoPlusRay(gt, peer_origin, peer_dir, sigma, fused);
    check(produced, "fused point exists (a raw+ray pair triangulates)");
    if (produced) {
        double err = (fused.position - gt).norm();
        check(err < 0.20, "fused point within 0.20 m of GT",
              "err = " + std::to_string(err) + " m, P = ("
              + std::to_string(fused.position.x()) + ", "
              + std::to_string(fused.position.y()) + ", "
              + std::to_string(fused.position.z()) + ")");
    }

    // ---- Check 2: live constraint — peer ray drives the (otherwise free) range ---------------
    // Ego LOS is the X-axis => ego gives no range. Rotating the peer ray about +Z by delta moves
    // the true intersection to x = 10 + 10*tan(delta). The fused range must track it monotonically.
    std::cout << "[2] peer-ray perturbation moves the solution monotonically (x = 10 + 10*tan d)"
              << std::endl;
    const double deltas_deg[] = {-2.0, -1.0, 0.0, 1.0, 2.0};
    std::vector<double> xs;
    bool all_produced = true, all_accurate = true;
    for (double dd : deltas_deg) {
        double d = deg2rad(dd);
        // Rz(d) * (0,-1,0) = (sin d, -cos d, 0)
        Eigen::Vector3d dir(std::sin(d), -std::cos(d), 0.0);
        MultiView::Detection::Detection3D f;
        bool ok = triangulateEgoPlusRay(gt, peer_origin, dir, sigma, f);
        if (!ok) { all_produced = false; xs.push_back(std::nan("")); continue; }
        double x_expected = 10.0 + 10.0 * std::tan(d);
        double xerr = std::abs(f.position.x() - x_expected);
        if (xerr > 0.05) all_accurate = false;
        xs.push_back(f.position.x());
        std::cout << "    delta = " << dd << " deg: x_fused = " << f.position.x()
                  << ", x_expected = " << x_expected << ", |err| = " << xerr << " m" << std::endl;
    }
    check(all_produced, "a fused point is produced for every perturbation");
    check(all_accurate, "recovered range matches the peer-ray/ego-LOS intersection (<0.05 m)");
    bool monotonic = true;
    for (size_t i = 1; i < xs.size(); ++i) {
        if (!(xs[i] > xs[i - 1] + 1e-4)) monotonic = false;
    }
    check(monotonic, "recovered range is strictly monotonic in the peer-ray angle");
    // Sensitivity magnitude: +-2 deg must move the point > 0.05 m (a live constraint, not noise).
    if (all_produced) {
        double span = xs.back() - xs.front();
        check(span > 0.05, "range span across +-2 deg is significant",
              "span = " + std::to_string(span) + " m");
    }

    // ---- Check 3: covariance finite + positive-definite (good geometry) -----------------------
    std::cout << "[3] fused covariance is finite and positive-definite" << std::endl;
    if (produced) {
        const Eigen::Matrix3d& C = fused.covariance;
        bool finite = C.allFinite();
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C);
        double lam_min = es.eigenvalues().minCoeff();
        bool pd = (lam_min > 0.0);
        bool not_sentinel = (C.trace() < 1e5);  // failure path returns 1e6 * I
        check(finite, "covariance entries all finite");
        check(pd, "covariance positive-definite", "lambda_min = " + std::to_string(lam_min));
        check(not_sentinel, "covariance is a real estimate, not the 1e6 failure sentinel",
              "trace = " + std::to_string(C.trace()));
    }

    // ---- Check 4: covariance degrades as the rays approach parallel ---------------------------
    // Test propagateCovariance directly (bypasses the 2 m detection filter) with a raw ego + a
    // precomputed peer, comparing 90 deg parallax vs a near-collinear peer.
    std::cout << "[4] covariance grows as parallax -> 0 (peer ray near-parallel to ego LOS)"
              << std::endl;
    {
        const Eigen::Matrix3d K = intrinsics();
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        CovarianceConfig cfg;
        cfg.use_pose_covariance = false;

        auto covWithPeerAt = [&](const Eigen::Vector3d& peer_pos) -> Eigen::Matrix3d {
            Camera ego;
            ego.id_ = 0; ego.K_ = K; ego.R_ = I; ego.t_ = Eigen::Vector3d::Zero();
            Camera peer;
            peer.id_ = 1; peer.is_precomputed_ = true; peer.t_ = peer_pos; peer.bearing_sigma_ = sigma;
            std::vector<Camera> cams = {ego, peer};
            std::vector<int> idx = {0, 1};
            return MultiView::propagateCovariance(gt, cams, idx, cfg);
        };

        Eigen::Matrix3d C_good = covWithPeerAt(Eigen::Vector3d(10.0, 10.0, 0.0));   // ~90 deg
        Eigen::Matrix3d C_par  = covWithPeerAt(Eigen::Vector3d(100.0, 3.0, 0.0));   // ~1.9 deg
        double t_good = C_good.trace();
        double t_par = C_par.trace();
        std::cout << "    trace(cov) good-parallax = " << t_good
                  << ",  near-parallel = " << t_par << std::endl;
        check(std::isfinite(t_good) && t_good > 0.0 && t_good < 1e5,
              "good-parallax covariance is finite and small", "trace = " + std::to_string(t_good));
        check(t_par > 5.0 * t_good,
              "near-parallel covariance is much larger (parallax degradation)",
              "ratio = " + std::to_string(t_par / t_good));
    }

    std::cout << "=== " << (g_failures == 0 ? "ALL CHECKS PASSED" : "FAILURES: " + std::to_string(g_failures))
              << " ===" << std::endl;
    return g_failures == 0 ? 0 : 1;
}
