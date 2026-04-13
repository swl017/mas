/**
 * @file covariance_propagation.cpp
 * @brief First-order Jacobian-based covariance propagation implementation.
 *
 * The projection pipeline (matching reprojection.h) is:
 *   1. point_camera = R_cam^T * (X_world - t)     where R_cam = camera.R_, t = camera.t_
 *   2. rotated      = W2C * point_camera           where W2C = world_to_camera_rotation (FLU->RDF)
 *   3. u = fx * rotated[0]/rotated[2] + cx
 *      v = fy * rotated[1]/rotated[2] + cy
 *
 * Combined rotation from world to optical frame: R_wo = W2C * R_cam^T
 * So: Xc = R_wo * (X_world - t)  and  du/dXc is the standard pinhole Jacobian.
 *
 * Jacobians derived:
 *   J_X   = (du/dXc) * R_wo             (w.r.t. world position of target)
 *   J_t   = -(du/dXc) * R_wo = -J_X     (w.r.t. camera position)
 *   J_phi = -(du/dXc) * [Xc]_x * R_wo   (w.r.t. small rotation perturbation of camera orientation)
 *   J_g   = -(du/dXc) * [Xc]_x * e_axis (w.r.t. each gimbal angle, axis in camera frame)
 */

#include "covariance_propagation.h"
#include <iostream>

namespace MultiView {

Eigen::Matrix<double, 2, 3> projectionJacobian(
    const Eigen::Vector3d& Xc, double fx, double fy)
{
    double Z = Xc.z();
    if (std::abs(Z) < 1e-6) {
        Z = (Z >= 0) ? 1e-6 : -1e-6;
    }
    double Z2 = Z * Z;

    Eigen::Matrix<double, 2, 3> J;
    J << fx / Z,    0.0,     -fx * Xc.x() / Z2,
         0.0,       fy / Z,  -fy * Xc.y() / Z2;
    return J;
}

Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d S;
    S <<  0.0,  -v.z(),  v.y(),
          v.z(),  0.0,  -v.x(),
         -v.y(),  v.x(),  0.0;
    return S;
}

Eigen::Matrix3d propagateCovariance(
    const Eigen::Vector3d& X_world,
    const std::vector<Camera>& cameras,
    const std::vector<int>& camera_indices,
    const CovarianceConfig& config,
    const Eigen::Matrix3d& world_to_camera_rotation)
{
    const int C = static_cast<int>(camera_indices.size());
    if (C < 2) {
        return Eigen::Matrix3d::Identity() * 1e6;
    }

    // Pixel noise covariance (same for all cameras)
    Eigen::Matrix2d Sigma_pix = Eigen::Matrix2d::Identity() * (config.pix_std * config.pix_std);
    Eigen::Matrix2d W = Eigen::Matrix2d::Identity() / (config.pix_std * config.pix_std);

    // Accumulate normal matrix A and sandwich middle term
    Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d sandwich = Eigen::Matrix3d::Zero();

    for (int ci = 0; ci < C; ++ci) {
        const Camera& cam = cameras[camera_indices[ci]];

        // Combined rotation: world → optical frame
        Eigen::Matrix3d R_wo = world_to_camera_rotation * cam.R_.transpose();

        // Point in camera (optical) frame
        Eigen::Vector3d Xc = R_wo * (X_world - cam.t_);

        // Skip if point is behind camera
        if (Xc.z() <= 0.0) {
            continue;
        }

        double fx = cam.K_(0, 0);
        double fy = cam.K_(1, 1);

        // Projection Jacobian du/dXc (2x3)
        Eigen::Matrix<double, 2, 3> dU_dXc = projectionJacobian(Xc, fx, fy);

        // World position Jacobian: J_X = (du/dXc) * R_wo  (2x3)
        Eigen::Matrix<double, 2, 3> J_X = dU_dXc * R_wo;

        // Normal matrix contribution
        A += J_X.transpose() * W * J_X;

        // Build residual covariance S_c = Sigma_pix + J_theta * Sigma_theta * J_theta^T
        // Start with pixel noise
        Eigen::Matrix2d S_c = Sigma_pix;

        // Camera position uncertainty
        if (config.include_position_uncertainty) {
            // J_t = -J_X  (2x3)
            Eigen::Matrix<double, 2, 3> J_t = -J_X;

            Eigen::Matrix3d Sigma_t;
            if (config.use_pose_covariance && cam.pose_covariance_.block<3, 3>(0, 0).trace() > 0.0) {
                // Use EKF position covariance (top-left 3x3 of 6x6 pose covariance)
                Sigma_t = cam.pose_covariance_.block<3, 3>(0, 0);
            } else {
                Sigma_t = Eigen::Matrix3d::Identity() * (config.pos_std * config.pos_std);
            }

            S_c += J_t * Sigma_t * J_t.transpose();
        }

        // Camera orientation uncertainty
        if (config.include_orientation_uncertainty) {
            // J_phi = -(du/dXc) * [Xc]_x * R_wo  (2x3, for 3-axis rotation perturbation)
            Eigen::Matrix3d Xc_skew = skewSymmetric(Xc);
            Eigen::Matrix<double, 2, 3> J_phi = -dU_dXc * Xc_skew * R_wo;

            Eigen::Matrix3d Sigma_phi;
            if (config.use_pose_covariance && cam.pose_covariance_.block<3, 3>(3, 3).trace() > 0.0) {
                // Use EKF orientation covariance (bottom-right 3x3 of 6x6 pose covariance)
                Sigma_phi = cam.pose_covariance_.block<3, 3>(3, 3);
            } else {
                Sigma_phi = Eigen::Matrix3d::Identity() * (config.ori_std * config.ori_std);
            }

            S_c += J_phi * Sigma_phi * J_phi.transpose();
        }

        // Gimbal angle uncertainty (3 axes: roll, pitch, yaw)
        if (config.include_gimbal_uncertainty) {
            Eigen::Matrix3d Xc_skew = skewSymmetric(Xc);
            double gimbal_var = config.gimbal_std * config.gimbal_std;

            // Gimbal axes in the camera optical frame.
            // The gimbal rotation is applied before W2C, so its axes in the
            // optical frame are: e_optical = W2C * e_body_frame
            // For a standard gimbal: roll=X, pitch=Y, yaw=Z in body frame
            Eigen::Vector3d e_roll  = world_to_camera_rotation * Eigen::Vector3d::UnitX();
            Eigen::Vector3d e_pitch = world_to_camera_rotation * Eigen::Vector3d::UnitY();
            Eigen::Vector3d e_yaw   = world_to_camera_rotation * Eigen::Vector3d::UnitZ();

            // Each gimbal axis contributes a 2x1 Jacobian
            Eigen::Vector2d J_groll  = -dU_dXc * Xc_skew * e_roll;
            Eigen::Vector2d J_gpitch = -dU_dXc * Xc_skew * e_pitch;
            Eigen::Vector2d J_gyaw   = -dU_dXc * Xc_skew * e_yaw;

            S_c += gimbal_var * (J_groll * J_groll.transpose()
                               + J_gpitch * J_gpitch.transpose()
                               + J_gyaw * J_gyaw.transpose());
        }

        // Accumulate sandwich: J_X^T * W * S_c * W * J_X
        sandwich += J_X.transpose() * W * S_c * W * J_X;
    }

    // Regularize A
    A += Eigen::Matrix3d::Identity() * config.regularization_eps;

    // Check condition number via SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(A);
    double cond = svd.singularValues()(0) / svd.singularValues()(2);
    if (cond > config.condition_threshold || !std::isfinite(cond)) {
        std::cerr << "[CovProp] Poor geometry: condition number = " << cond << std::endl;
        return Eigen::Matrix3d::Identity() * 1e6;
    }

    // Final covariance: A^{-1} * sandwich * A^{-T}
    Eigen::Matrix3d A_inv = A.inverse();
    Eigen::Matrix3d Sigma_X = A_inv * sandwich * A_inv.transpose();

    // Validate: covariance diagonal must be positive
    if (Sigma_X(0, 0) <= 0.0 || Sigma_X(1, 1) <= 0.0 || Sigma_X(2, 2) <= 0.0) {
        std::cerr << "[CovProp] Non-positive diagonal in covariance" << std::endl;
        return Eigen::Matrix3d::Identity() * 1e6;
    }

    return Sigma_X;
}

}  // namespace MultiView
