/**
 * @file camera.h
 * @brief Camera class definitions with the detections related to the camera
 */

#ifndef MULTIVIEW_CAMERA_H
#define MULTIVIEW_CAMERA_H

#include "detection.h"
#include "ray.h"
#include <Eigen/Eigen>

namespace MultiView {

class Camera
{
public:
    Camera(/* args */) : pose_covariance_(Eigen::Matrix<double, 6, 6>::Zero()),
                         gimbal_angles_(Eigen::Vector3d::Zero()) {};
    ~Camera() {};
    int id_;
    double height_;
    double width_;
    Eigen::Matrix3d K_;
    Eigen::Matrix3d R_;                     // Combined world-to-gimbal rotation
    Eigen::Vector3d t_;                     // Camera position in world frame
    Eigen::Matrix<double, 6, 6> pose_covariance_;  // [position(3), orientation(3)] from EKF
    Eigen::Vector3d gimbal_angles_;         // [roll, pitch, yaw] in radians
    std::vector<Eigen::Vector3d> frustum_corners_; // Frustum corners in world coordinates
    std::vector<Detection> detections_;
    void addDetection2D(const Detection::Detection2D& detection2d)
    {
        Detection detection;
        // Create a new ray to avoid const issues
        Ray ray;
        ray.getRayFromPixels(K_, R_, t_, detection2d.center);
        detection.detection2d = detection2d;
        detection.detection2d.ray = ray; // Use the new ray with updated values
        detections_.push_back(detection);
    }
    void addPrecomputedRay(const Eigen::Vector3d& ray_origin,
                           const Eigen::Vector3d& ray_direction,
                           const std::string& class_id)
    {
        Detection detection;
        detection.detection2d.ray.ray_origin = ray_origin;
        detection.detection2d.ray.ray_direction = ray_direction.normalized();
        detection.detection2d.class_id = class_id;
        detection.detection2d.confidence = 1.0f;
        detections_.push_back(detection);
    }
    std::vector<Eigen::Vector3d> getFrustumCorners(const double& vertice_length)
    {
        std::vector<Eigen::Vector3d> corners;
        std::vector<Eigen::Vector2d> pixels = {
            Eigen::Vector2d(0, 0),
            Eigen::Vector2d(width_, 0),
            Eigen::Vector2d(width_, height_),
            Eigen::Vector2d(0, height_)
        };
        for (int i = 0; i < 4; ++i)
        {
            Ray ray;
            ray.getRayFromPixels(K_, R_, t_, pixels[i]);
            corners.push_back(ray.ray_direction * vertice_length + ray.ray_origin);
            // std::cout << "Pixel " << i << ": " << pixels[i].transpose() << std::endl;
            // std::cout << "Ray origin: " << ray.ray_origin.transpose() << std::endl;
            // std::cout << "Ray direction: " << ray.ray_direction.transpose() << std::endl;
            // std::cout << "Frustum corner " << i << ": " << corners.back().transpose() << std::endl;
        }
        frustum_corners_ = corners;
        return frustum_corners_;
    }
};

}  // namespace MultiView
#endif  // MULTIVIEW_CAMERA_H