/**
 * @file reprojection.h
 * @brief Reprojection class definitions for multiple cameras
 */

#ifndef MULTIVIEW_REPROJECTION_H
#define MULTIVIEW_REPROJECTION_H

#include "camera.h"
#include "detection.h"
#include "ray.h"

#include <cmath>
#include <Eigen/Eigen>
#include <ceres/jet.h>

namespace MultiView {

struct Reprojection
{
    Reprojection(const Camera& camera,
                 const Eigen::Matrix3d& world_to_camera_rotation = 
                        (Eigen::Matrix3d() << 0, -1, 0, 
                                                0, 0, -1, 
                                                1, 0, 0).finished())
    : camera_(camera), world_to_camera_rotation_(world_to_camera_rotation) 
    {};
    virtual ~Reprojection() = default;

    template <typename T>
    bool project(const T* const position, T* projected) const {
        // Transform point from world to camera coordinates
        T point_camera[3] = {T(0), T(0), T(0)};

        // Apply camera extrinsics (R, t)
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                point_camera[i] += T(camera_.R_.transpose()(i, j)) * (position[j] - T(camera_.t_(j)));
            }
        }

        // Apply world-to-camera rotation
        T rotated_point[3] = {T(0), T(0), T(0)};
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                rotated_point[i] += T(world_to_camera_rotation_(i, j)) * point_camera[j];
            }
        }
        
        // Check if point is in front of camera
        if (rotated_point[2] <= T(0)) {
            return false; // Point is behind camera
        }
        
        // Project to normalized image plane
        T x = rotated_point[0] / rotated_point[2];
        T y = rotated_point[1] / rotated_point[2];
        
        // Apply camera intrinsics
        projected[0] = T(camera_.K_(0, 0)) * x + T(camera_.K_(0, 2));
        projected[1] = T(camera_.K_(1, 1)) * y + T(camera_.K_(1, 2));
        

        return true;
    }

    Camera camera_;
    Eigen::Matrix3d world_to_camera_rotation_;
};


struct ReprojectionError : Reprojection {
    ReprojectionError(const Camera& camera,
                      const Eigen::Vector2d& observed,
                      const Eigen::Matrix3d& world_to_camera_rotation = 
                        (Eigen::Matrix3d() << 0, -1, 0, 
                                              0, 0, -1, 
                                              1, 0, 0).finished())
    : Reprojection(camera, world_to_camera_rotation), observed_(observed) {}
    
    template <typename T>
    bool operator()(const T* const position, T* residuals) const {
        // Projected point
        T projected[2];
        
        // Try to project the 3D point
        if (!project(position, projected)) {
            // Point is behind camera, set large residuals
            residuals[0] = T(1000.0);
            residuals[1] = T(1000.0);
            return false;
        }
        
        // Compute residuals (observed - projected)
        residuals[0] = T(observed_[0]) - projected[0];
        residuals[1] = T(observed_[1]) - projected[1];
        
        return true;
    }

    // Fix the template issue - remove template parameter and use double explicitly
    double getReprojectionError(const Eigen::Vector3d& point) const {
        Eigen::Vector2d diff;
        double projected[2];
        project<double>(point.data(), projected);
        diff[0] = observed_[0] - projected[0];
        diff[1] = observed_[1] - projected[1];
        return diff.norm();
    }

    Eigen::Vector2d observed_;
};

/**
 * @brief Point-to-ray angular residual for a transmitted (precomputed) bearing ray.
 *
 * A cooperative peer transmits a bearing RAY (origin o + unit direction d) computed at the
 * source — it has no local image model (no K_), so it cannot contribute a pixel-reprojection
 * residual. Its natural residual is the target point's angular deviation from the transmitted
 * ray. With v = X - o and along-ray range rho = v.d, the perpendicular error is
 * e = || v - rho * d ||, and the residual is the *angular* quantity
 *
 *     r = e / (rho * sigma_theta)                                         (ticket 020, Q3=b)
 *
 * dimensionless and directly comparable to a raw camera's pixel residual. Like the pixel
 * residual (reprojection.h divides the rotated point by its depth Z(X)), rho = rho(X) is a
 * function of the point being solved; Ceres autodiff re-linearizes it each iteration, and the
 * recovered range is exactly what the ego/peer parallax supplies. Guards: reject rho <= 0
 * (target behind the peer, mirroring the "behind camera -> false" pixel path) and floor rho at
 * kRhoFloor for the division.  1 residual, 3 params.
 */
struct PointToRayError {
    PointToRayError(const Eigen::Vector3d& ray_origin,
                    const Eigen::Vector3d& ray_direction,
                    double sigma_theta)
    : origin_(ray_origin),
      dir_(ray_direction.normalized()),
      sigma_theta_(sigma_theta > 1e-9 ? sigma_theta : kDefaultBearingSigmaRad) {}

    template <typename T>
    bool operator()(const T* const position, T* residuals) const {
        // v = X - o
        const T vx = position[0] - T(origin_[0]);
        const T vy = position[1] - T(origin_[1]);
        const T vz = position[2] - T(origin_[2]);

        // Along-ray range rho = v . d
        const T rho = vx * T(dir_[0]) + vy * T(dir_[1]) + vz * T(dir_[2]);
        if (rho <= T(0)) {
            residuals[0] = T(1000.0);  // target behind the peer
            return false;
        }

        // Perpendicular component v_perp = v - rho * d
        const T px = vx - rho * T(dir_[0]);
        const T py = vy - rho * T(dir_[1]);
        const T pz = vz - rho * T(dir_[2]);
        const T e2 = px * px + py * py + pz * pz;
        using std::sqrt;  // ADL: ceres::sqrt for Jet, std::sqrt for double
        const T e = sqrt(e2 + T(1e-18));  // epsilon keeps the derivative finite at e = 0

        const T rho_floored = rho > T(kRhoFloor) ? rho : T(kRhoFloor);
        residuals[0] = e / (rho_floored * T(sigma_theta_));
        return true;
    }

    // Post-solve metric: perpendicular distance from the point to the transmitted ray [metres].
    double getRayDistance(const Eigen::Vector3d& point) const {
        const Eigen::Vector3d v = point - origin_;
        const double rho = v.dot(dir_);
        const Eigen::Vector3d v_perp = v - rho * dir_;
        return v_perp.norm();
    }

    Eigen::Vector3d origin_;
    Eigen::Vector3d dir_;
    double sigma_theta_;
    static constexpr double kRhoFloor = 1e-3;  // metres; avoids blow-up near the peer origin
};

struct RegularizationError {
    RegularizationError(const Eigen::Vector3d& initial_guess)
    : initial_guess_(initial_guess) {}

    template <typename T>
    bool operator()(const T* const position, T* residuals) const {
        residuals[0] = position[0] - T(initial_guess_[0]);
        residuals[1] = position[1] - T(initial_guess_[1]);
        residuals[2] = position[2] - T(initial_guess_[2]);
        return true;
    }

    Eigen::Vector3d initial_guess_;

};
}  // namespace MultiView
#endif  // MULTIVIEW_REPROJECTION_H