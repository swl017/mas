/**
 * @file meas_noise.h
 * @brief Ticket 024 Q9 (estimated-vs-characterized hybrid) + Q10 Tier-1 peer-bearing noise.
 *
 * The peer bearing factor's 2x2 tangent covariance is built per measurement as
 *   R_theta_eff = R_static  +  R_att  +  J_o Sigma_o J_o^T
 * where
 *   R_static : gimbal + calibration + pixel angular variance -- an OFFLINE-characterized parameter
 *              (fusion-side), plus the Q10 Tier-1 azimuthal inflation sigma_psi^2 for the
 *              un-modeled inter-vehicle relative-heading bias (a BOUND, not a fix);
 *   R_att    : from the TRANSMITTED EKF2 attitude covariance (online-estimated), approximated as an
 *              isotropic tangent angular variance;
 *   J_o Sigma_o J_o^T : the peer origin (EKF2 position) covariance projected to the tangent at the
 *              ego-known range (J_o = -[u1;u2]/rho). Peer pose stays FIXED (Q9); its uncertainty
 *              enters the measurement noise (the ticket 020 S0 sandwich, reused).
 *
 * The origin term is evaluated at the current target estimate X_est (a linearization; a
 * per-iteration reweight is the documented refinement).
 */
#ifndef MAS_MULTIVIEW_FGO_MEAS_NOISE_H
#define MAS_MULTIVIEW_FGO_MEAS_NOISE_H

#include <Eigen/Eigen>

namespace mas_fgo {

struct PeerNoiseParams {
    double sigma_static_rad = 0.008726646;  // 0.5 deg: gimbal+calib+pixel angular sigma (offline)
    double sigma_psi_rad = 0.0;             // Q10 Tier-1 azimuthal inflation (relative-heading bias)
    bool include_attitude = true;           // fold transmitted EKF2 attitude cov
    bool include_origin = true;             // fold transmitted EKF2 position cov (range-projected)
};

/**
 * @brief Build the 2x2 peer-bearing tangent covariance R_theta_eff (Q9 hybrid + Q10 Tier-1).
 * @param u1,u2       tangent basis (perpendicular to the ray direction) from PeerBearingFactor.
 * @param origin,dir  peer ray origin + unit direction.
 * @param X_est       current target position estimate (for the range projection of Sigma_o).
 * @param Sigma_att   3x3 transmitted attitude covariance [rad^2] (zero => skip).
 * @param Sigma_pos   3x3 transmitted position covariance [m^2] (zero => skip).
 */
inline Eigen::Matrix2d buildPeerBearingCov(
    const PeerNoiseParams& prm,
    const Eigen::Vector3d& u1, const Eigen::Vector3d& u2,
    const Eigen::Vector3d& origin, const Eigen::Vector3d& dir,
    const Eigen::Vector3d& X_est,
    const Eigen::Matrix3d& Sigma_att = Eigen::Matrix3d::Zero(),
    const Eigen::Matrix3d& Sigma_pos = Eigen::Matrix3d::Zero())
{
    // R_static (isotropic) + Q10 azimuthal inflation.
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * (prm.sigma_static_rad * prm.sigma_static_rad);
    if (prm.sigma_psi_rad > 0.0) {
        // Add sigma_psi^2 on the more-horizontal tangent axis (the azimuth/yaw-sensitive one).
        const double h1 = std::hypot(u1.x(), u1.y());  // horizontal content of u1
        const double h2 = std::hypot(u2.x(), u2.y());
        const double s2 = prm.sigma_psi_rad * prm.sigma_psi_rad;
        R(0, 0) += s2 * h1 * h1;
        R(1, 1) += s2 * h2 * h2;
    }

    // R_att: transmitted attitude cov -> isotropic tangent angular variance (first-order).
    if (prm.include_attitude && Sigma_att.trace() > 0.0) {
        R += Eigen::Matrix2d::Identity() * (Sigma_att.trace() / 3.0);
    }

    // Origin (position) uncertainty projected to the tangent at the current range.
    if (prm.include_origin && Sigma_pos.trace() > 0.0) {
        const Eigen::Vector3d v = X_est - origin;
        double rho = v.dot(dir.normalized());
        if (rho < 1e-3) rho = 1e-3;
        Eigen::Matrix<double, 2, 3> J_o;      // J_X = [u1;u2]/rho ; J_o = -J_X
        J_o.row(0) = -u1.transpose() / rho;
        J_o.row(1) = -u2.transpose() / rho;
        R += J_o * Sigma_pos * J_o.transpose();
    }
    return R;
}

}  // namespace mas_fgo

#endif  // MAS_MULTIVIEW_FGO_MEAS_NOISE_H
