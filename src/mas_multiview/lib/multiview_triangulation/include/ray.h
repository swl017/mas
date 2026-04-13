/**
 * @file ray.h
 * @brief Ray class definitions with the ray point distance
 */

#ifndef MULTIVIEW_RAY_H
#define MULTIVIEW_RAY_H

#include <iostream>
#include <Eigen/Eigen>

namespace MultiView {

class Ray
{
public:
    Ray(/* args */) {};
    ~Ray() {};
    Eigen::Vector3d ray_origin;
    Eigen::Vector3d ray_direction;

    void getRayFromPixels(const Eigen::Matrix3d& K, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector2d& pixel)
    {
        Eigen::Vector3d ray_camera;
        double u = (pixel.x() - K(0, 2)) / K(0, 0);
        double v = (pixel.y() - K(1, 2)) / K(1, 1);
        ray_camera << u, v, 1.0;  // Ray in camera coordinates
        // ray_camera << K.inverse() * Eigen::Vector3d(pixel.x(), pixel.y(), 1.0);

        // world frame
        Eigen::Vector3d ray_world;
        ray_world << ray_camera.z(),
                     -ray_camera.x(),
                     -ray_camera.y();
        // std::cout << "Pixel: " << pixel.transpose() << std::endl;
        // std::cout << "K: " << std::endl << K << std::endl;
        // std::cout << "Ray in camera coordinates: " << ray_camera.transpose() << std::endl;
        ray_direction = R * ray_world.normalized();
        // std::cout << "Ray direction in ENU coordinates: " << ray_direction.transpose() << std::endl;
        ray_origin = t;
    }

    // Add const version for when called on const Ray
    // void getRayFromPixels(const Eigen::Matrix3d& K, const Eigen::Matrix3d& R, const Eigen::Vector3d& t, const Eigen::Vector2d& pixel) const
    // {
    //     // Cannot modify ray_direction and ray_origin in const method
    //     // This is needed just for compilation - the caller needs to handle this appropriately
    // }

    double distanceToPoint(const Eigen::Vector3d& point)
    {
        Eigen::Vector3d& rayOrigin = ray_origin;
        Eigen::Vector3d& rayDirection = ray_direction;
        // Normalize the ray direction
        Eigen::Vector3d normalizedDirection = rayDirection.normalized();

        // Vector from ray origin to the point
        Eigen::Vector3d v = point - rayOrigin;

        // Project v onto the ray direction
        double t = v.dot(normalizedDirection);

        // If t < 0, the closest point is the ray origin
        if (t < 0) {
            return (point - rayOrigin).norm();
        }

        // Calculate the closest point on the ray
        Eigen::Vector3d closestPoint = rayOrigin + t * normalizedDirection;

        // Return the distance between the closest point and the given point
        return (point - closestPoint).norm();
    }

    double angleBetweenVectors(const Eigen::Vector3d& v1, const Eigen::Vector3d& v2)
    {
        // Normalize the vectors
        Eigen::Vector3d v1_normalized = v1.normalized();
        Eigen::Vector3d v2_normalized = v2.normalized();

        // Calculate the dot product
        double dot_product = v1_normalized.dot(v2_normalized);

        // Clamp the dot product to [-1, 1] to avoid domain errors with acos
        dot_product = std::max(-1.0, std::min(1.0, dot_product));

        // Calculate the angle in radians
        double angle_rad = std::acos(dot_product);

        // Convert to degrees if needed
        // double angle_deg = angle_rad * 180.0 / M_PI;

        return angle_rad;
    }
private:
};
}  // namespace MultiView
#endif  // MULTIVIEW_RAY_H