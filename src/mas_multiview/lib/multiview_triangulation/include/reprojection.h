/**
 * @file reprojection.h
 * @brief Reprojection class definitions for multiple cameras
 */

#ifndef MULTIVIEW_REPROJECTION_H
#define MULTIVIEW_REPROJECTION_H

#include "camera.h"
#include "detection.h"
#include "ray.h"

#include <Eigen/Eigen>

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