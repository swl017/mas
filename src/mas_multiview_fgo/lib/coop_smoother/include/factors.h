/**
 * @file factors.h
 * @brief GTSAM custom factors for the cooperative target-trajectory smoother (ticket 024).
 *
 * Target state at keyframe k = (position p_k : Point3, velocity v_k : Vector3). Observer/peer
 * poses are held FIXED (018 §3.1 + ticket 024 Q9), so both measurement factors are UNARY on the
 * target position; their pose/geometry is baked in at construction (the ego pose interpolated to
 * the detection time by pose_interp.h, ticket 024 Q8).
 *
 *  - CVMotionFactor   : constant-velocity motion between consecutive keyframes (Q: CV first).
 *  - EgoPixelFactor   : pixel reprojection for the local ego camera (Q7 — keep pixels locally;
 *                        same projection math as ticket 020 reprojection.h).
 *  - PeerBearingFactor: 2-DOF tangent angular residual for a transmitted peer ray (the 2-DOF
 *                        generalization of ticket 020 PointToRayError; carries the anisotropic
 *                        R_theta from ticket 024 Q9).
 *
 * GTSAM 4.2 API: evaluateError uses boost::optional<Matrix&> Jacobians. Analytic Jacobians.
 */
#ifndef MAS_MULTIVIEW_FGO_FACTORS_H
#define MAS_MULTIVIEW_FGO_FACTORS_H

#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>

#include <Eigen/Eigen>

namespace mas_fgo {

// ---------------------------------------------------------------------------------------------
// Constant-velocity motion factor between (p0,v0) at k and (p1,v1) at k+1.
//   r = [ p1 - p0 - v0*dt ; v1 - v0 ]   (6-vector)
// White-noise-acceleration process noise Q(dt) is supplied via the factor's noise model.
// ---------------------------------------------------------------------------------------------
class CVMotionFactor
    : public gtsam::NoiseModelFactor4<gtsam::Point3, gtsam::Vector3, gtsam::Point3, gtsam::Vector3>
{
public:
    CVMotionFactor(gtsam::Key p0, gtsam::Key v0, gtsam::Key p1, gtsam::Key v1,
                   double dt, const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactor4<gtsam::Point3, gtsam::Vector3, gtsam::Point3, gtsam::Vector3>(
              noise, p0, v0, p1, v1),
          dt_(dt) {}

    gtsam::Vector evaluateError(
        const gtsam::Point3& p0, const gtsam::Vector3& v0,
        const gtsam::Point3& p1, const gtsam::Vector3& v1,
        boost::optional<gtsam::Matrix&> H0 = boost::none,
        boost::optional<gtsam::Matrix&> H1 = boost::none,
        boost::optional<gtsam::Matrix&> H2 = boost::none,
        boost::optional<gtsam::Matrix&> H3 = boost::none) const override
    {
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        gtsam::Vector6 r;
        r.head<3>() = p1 - p0 - v0 * dt_;
        r.tail<3>() = v1 - v0;
        if (H0) { Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero(); J.block<3, 3>(0, 0) = -I;          *H0 = J; }
        if (H1) { Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero(); J.block<3, 3>(0, 0) = -dt_ * I; J.block<3, 3>(3, 0) = -I; *H1 = J; }
        if (H2) { Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero(); J.block<3, 3>(0, 0) =  I;          *H2 = J; }
        if (H3) { Eigen::Matrix<double, 6, 3> J = Eigen::Matrix<double, 6, 3>::Zero(); J.block<3, 3>(3, 0) =  I;          *H3 = J; }
        return r;
    }

    double dt() const { return dt_; }

private:
    double dt_;
};

// ---------------------------------------------------------------------------------------------
// Ego pixel-reprojection factor (unary on the target position). Camera fixed at construction:
//   Xc = M (X - t_cam),  M = W2C * R_cam^T ;  pixel = [fx*Xc.x/Xc.z + cx ; fy*Xc.y/Xc.z + cy]
//   r = observed_px - pixel(X)   (matches ticket 020 reprojection.h; residual = observed - proj)
// ---------------------------------------------------------------------------------------------
class EgoPixelFactor : public gtsam::NoiseModelFactor1<gtsam::Point3>
{
public:
    EgoPixelFactor(gtsam::Key x, const Eigen::Vector2d& observed,
                   const Eigen::Matrix3d& K, const Eigen::Matrix3d& R_cam,
                   const Eigen::Vector3d& t_cam, const gtsam::SharedNoiseModel& noise,
                   const Eigen::Matrix3d& world_to_camera =
                       (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished())
        : gtsam::NoiseModelFactor1<gtsam::Point3>(noise, x),
          observed_(observed), t_cam_(t_cam),
          fx_(K(0, 0)), fy_(K(1, 1)), cx_(K(0, 2)), cy_(K(1, 2)),
          M_(world_to_camera * R_cam.transpose()) {}

    gtsam::Vector evaluateError(
        const gtsam::Point3& X,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        const Eigen::Vector3d Xc = M_ * (X - t_cam_);
        const double Z = Xc.z();
        if (Z <= kZFloor_) {  // behind camera: large, gradient-free residual
            if (H) *H = Eigen::Matrix<double, 2, 3>::Zero();
            return (gtsam::Vector2() << 1e3, 1e3).finished();
        }
        const double u = fx_ * Xc.x() / Z + cx_;
        const double v = fy_ * Xc.y() / Z + cy_;
        if (H) {
            Eigen::Matrix<double, 2, 3> dpix_dXc;
            dpix_dXc << fx_ / Z, 0.0, -fx_ * Xc.x() / (Z * Z),
                        0.0, fy_ / Z, -fy_ * Xc.y() / (Z * Z);
            *H = -dpix_dXc * M_;  // r = observed - proj  ->  dr/dX = -dproj/dX
        }
        return (gtsam::Vector2() << observed_.x() - u, observed_.y() - v).finished();
    }

private:
    Eigen::Vector2d observed_;
    Eigen::Vector3d t_cam_;
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3d M_;
    static constexpr double kZFloor_ = 1e-3;
};

// ---------------------------------------------------------------------------------------------
// Peer transmitted-ray bearing factor (unary on the target position). 2-DOF tangent angular
// residual about the ray (origin o, unit dir d), with u1,u2 an orthonormal basis of d^perp:
//   v = X - o,  rho = v.d,  r = [ v.u1 ; v.u2 ] / rho     (dimensionless bearing residual)
// Jacobian rows: u_i^T/rho - (v.u_i)/rho^2 * d^T. Anisotropic R_theta enters via the noise model.
// ---------------------------------------------------------------------------------------------
class PeerBearingFactor : public gtsam::NoiseModelFactor1<gtsam::Point3>
{
public:
    PeerBearingFactor(gtsam::Key x, const Eigen::Vector3d& origin,
                      const Eigen::Vector3d& direction, const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactor1<gtsam::Point3>(noise, x),
          o_(origin), d_(direction.normalized())
    {
        // Orthonormal basis of the plane perpendicular to d.
        const Eigen::Vector3d seed =
            (std::abs(d_.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        u1_ = (seed - seed.dot(d_) * d_).normalized();
        u2_ = d_.cross(u1_);
    }

    gtsam::Vector evaluateError(
        const gtsam::Point3& X,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        const Eigen::Vector3d v = X - o_;
        double rho = v.dot(d_);
        const bool behind = (rho <= kRhoFloor_);
        if (behind) rho = kRhoFloor_;  // floor to keep the division finite
        const double a1 = v.dot(u1_), a2 = v.dot(u2_);
        if (H) {
            if (behind) {
                *H = Eigen::Matrix<double, 2, 3>::Zero();
            } else {
                Eigen::Matrix<double, 2, 3> J;
                J.row(0) = u1_.transpose() / rho - (a1 / (rho * rho)) * d_.transpose();
                J.row(1) = u2_.transpose() / rho - (a2 / (rho * rho)) * d_.transpose();
                *H = J;
            }
        }
        return (gtsam::Vector2() << a1 / rho, a2 / rho).finished();
    }

    const Eigen::Vector3d& u1() const { return u1_; }
    const Eigen::Vector3d& u2() const { return u2_; }

private:
    Eigen::Vector3d o_, d_, u1_, u2_;
    static constexpr double kRhoFloor_ = 1e-3;
};

// ---------------------------------------------------------------------------------------------
// RAL ticket 028 S2c — ego pixel factor with a shared 2-DOF pixel-bias state b:
//   r = observed - (proj(X) + b)
// Models the episode-varying DC detection/transduction offset (024 S4) explicitly instead of
// pretending the per-ray noise is white. One bias key is shared by all ego factors of a solve.
// ---------------------------------------------------------------------------------------------
class EgoPixelBiasFactor : public gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Vector2>
{
public:
    EgoPixelBiasFactor(gtsam::Key x, gtsam::Key b, const Eigen::Vector2d& observed,
                       const Eigen::Matrix3d& K, const Eigen::Matrix3d& R_cam,
                       const Eigen::Vector3d& t_cam, const gtsam::SharedNoiseModel& noise,
                       const Eigen::Matrix3d& world_to_camera =
                           (Eigen::Matrix3d() << 0, -1, 0, 0, 0, -1, 1, 0, 0).finished())
        : gtsam::NoiseModelFactor2<gtsam::Point3, gtsam::Vector2>(noise, x, b),
          observed_(observed), t_cam_(t_cam),
          fx_(K(0, 0)), fy_(K(1, 1)), cx_(K(0, 2)), cy_(K(1, 2)),
          M_(world_to_camera * R_cam.transpose()) {}

    gtsam::Vector evaluateError(
        const gtsam::Point3& X, const gtsam::Vector2& b,
        boost::optional<gtsam::Matrix&> HX = boost::none,
        boost::optional<gtsam::Matrix&> Hb = boost::none) const override
    {
        const Eigen::Vector3d Xc = M_ * (X - t_cam_);
        const double Z = Xc.z();
        if (Z <= kZFloor_) {
            if (HX) *HX = Eigen::Matrix<double, 2, 3>::Zero();
            if (Hb) *Hb = Eigen::Matrix<double, 2, 2>::Zero();
            return (gtsam::Vector2() << 1e3, 1e3).finished();
        }
        const double u = fx_ * Xc.x() / Z + cx_;
        const double v = fy_ * Xc.y() / Z + cy_;
        if (HX) {
            Eigen::Matrix<double, 2, 3> dpix_dXc;
            dpix_dXc << fx_ / Z, 0.0, -fx_ * Xc.x() / (Z * Z),
                        0.0, fy_ / Z, -fy_ * Xc.y() / (Z * Z);
            *HX = -dpix_dXc * M_;
        }
        if (Hb) *Hb = -Eigen::Matrix2d::Identity();
        return (gtsam::Vector2() << observed_.x() - (u + b.x()),
                                    observed_.y() - (v + b.y())).finished();
    }

private:
    Eigen::Vector2d observed_;
    Eigen::Vector3d t_cam_;
    double fx_, fy_, cx_, cy_;
    Eigen::Matrix3d M_;
    static constexpr double kZFloor_ = 1e-3;
};

// ---------------------------------------------------------------------------------------------
// RAL ticket 028 S2c — depth-memory range prior (IDP-inspired, ADAPTED — not a full
// inverse-depth parameterization): 1-DOF prior on the range from a fixed ego position,
//   r = |X - o| - r_mem
// Carries filter-like depth memory across windows along the only weakly-observed direction.
// Self-confirming-feedback risk is a DESIGN property (i_design_s2c E4) — probe use only.
// ---------------------------------------------------------------------------------------------
class RangePriorFactor : public gtsam::NoiseModelFactor1<gtsam::Point3>
{
public:
    RangePriorFactor(gtsam::Key x, const Eigen::Vector3d& o_ego, double r_mem,
                     const gtsam::SharedNoiseModel& noise)
        : gtsam::NoiseModelFactor1<gtsam::Point3>(noise, x), o_(o_ego), r_(r_mem) {}

    gtsam::Vector evaluateError(
        const gtsam::Point3& X,
        boost::optional<gtsam::Matrix&> H = boost::none) const override
    {
        const Eigen::Vector3d v = X - o_;
        const double n = std::max(v.norm(), 1e-6);
        if (H) *H = (v / n).transpose();
        return (gtsam::Vector1() << n - r_).finished();
    }

private:
    Eigen::Vector3d o_;
    double r_;
};

}  // namespace mas_fgo

#endif  // MAS_MULTIVIEW_FGO_FACTORS_H
