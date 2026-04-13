/**
 * @file covariance_propagation.h
 * @brief First-order Jacobian-based covariance propagation for triangulated positions.
 *
 * Propagates uncertainty from pixel noise, camera pose (position + orientation),
 * and gimbal angles through the triangulation to produce a 3x3 output covariance.
 * Uses the sandwich formula: Σ_X = A⁻¹ (Σ_c J_X^T W S W J_X) A⁻ᵀ
 */

#ifndef MULTIVIEW_COVARIANCE_PROPAGATION_H
#define MULTIVIEW_COVARIANCE_PROPAGATION_H

#include "camera.h"
#include <Eigen/Eigen>
#include <vector>

namespace MultiView {

struct CovarianceConfig {
    double pix_std = 7.0;           // Pixel detection noise [pixels]
    double pos_std = 0.1;           // Camera position uncertainty [meters]
    double ori_std = 0.001;         // Camera orientation uncertainty [radians]
    double gimbal_std = 0.001;      // Gimbal angle uncertainty [radians]
    bool use_pose_covariance = true;    // Use EKF pose covariance if available
    bool include_position_uncertainty = true;
    bool include_orientation_uncertainty = true;
    bool include_gimbal_uncertainty = true;
    double regularization_eps = 1e-6;
    double condition_threshold = 1e6;
};

/**
 * @brief Compute the 2x3 projection Jacobian du/dXc for a point in camera frame.
 * @param Xc Point in camera frame [X, Y, Z]
 * @param fx Focal length x
 * @param fy Focal length y
 * @return 2x3 Jacobian matrix
 */
Eigen::Matrix<double, 2, 3> projectionJacobian(
    const Eigen::Vector3d& Xc, double fx, double fy);

/**
 * @brief Compute the 3x3 skew-symmetric matrix [v]_x for cross-product.
 */
Eigen::Matrix3d skewSymmetric(const Eigen::Vector3d& v);

/**
 * @brief Propagate input uncertainties to triangulated position covariance.
 *
 * @param X_world Triangulated position in world frame
 * @param cameras Vector of cameras that observed this target
 * @param camera_indices Indices into cameras vector for this association
 * @param config Covariance configuration parameters
 * @param world_to_camera_rotation The FLU->RDF rotation applied in projection
 * @return 3x3 covariance matrix of the triangulated position, or identity*1e6 on failure
 */
Eigen::Matrix3d propagateCovariance(
    const Eigen::Vector3d& X_world,
    const std::vector<Camera>& cameras,
    const std::vector<int>& camera_indices,
    const CovarianceConfig& config,
    const Eigen::Matrix3d& world_to_camera_rotation =
        (Eigen::Matrix3d() << 0, -1, 0,
                               0,  0, -1,
                               1,  0,  0).finished());

}  // namespace MultiView
#endif  // MULTIVIEW_COVARIANCE_PROPAGATION_H
