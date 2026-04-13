#include <Eigen/Dense>
#include <iostream>
#include <cmath>

class CameraRayToENU {
public:
    struct CameraIntrinsics {
        double fx, fy;  // Focal lengths in pixels
        double cx, cy;  // Principal point
        double k1, k2, p1, p2, k3;  // Distortion coefficients
    };

    struct CameraExtrinsics {
        Eigen::Matrix3d R_enu_to_cam;  // Rotation from ENU to camera
        Eigen::Vector3d t_cam_in_enu;  // Camera position in ENU
    };

    struct Ray3D {
        Eigen::Vector3d origin;
        Eigen::Vector3d direction;
    };

private:
    CameraIntrinsics intrinsics_;
    CameraExtrinsics extrinsics_;
    Eigen::Matrix3d R_cam_to_enu_;  // Inverse rotation (camera to ENU)

public:
    CameraRayToENU(const CameraIntrinsics& intrinsics, const CameraExtrinsics& extrinsics)
        : intrinsics_(intrinsics), extrinsics_(extrinsics) {
        // Precompute the inverse rotation matrix (camera to ENU)
        R_cam_to_enu_ = extrinsics_.R_enu_to_cam.transpose();
    }

    // Convert pixel to ray in ENU frame
    Ray3D pixelToRayENU(double u, double v, bool apply_distortion = true) {
        // Step 1: Convert pixel to normalized camera coordinates
        Eigen::Vector2d normalized_coords;
        if (apply_distortion) {
            normalized_coords = undistortPixel(u, v);
        } else {
            normalized_coords = pixelToNormalized(u, v);
        }

        // Step 2: Create ray in camera frame
        // In camera frame: z-forward, x-right, y-down
        Eigen::Vector3d ray_direction_cam(normalized_coords.x(),
                                         normalized_coords.y(),
                                         1.0);
        ray_direction_cam.normalize();  // Normalize to unit vector

        // Ray origin in camera frame is [0, 0, 0]
        Eigen::Vector3d ray_origin_cam = Eigen::Vector3d::Zero();

        // Step 3: Transform ray to ENU frame
        Ray3D ray_enu;
        ray_enu.origin = extrinsics_.t_cam_in_enu;  // Camera position in ENU
        ray_enu.direction = R_cam_to_enu_ * ray_direction_cam;  // Rotate direction to ENU

        return ray_enu;
    }

    // Alternative method that returns the ray without applying distortion correction
    Ray3D pixelToRayENUFast(double u, double v) {
        return pixelToRayENU(u, v, false);
    }

private:
    // Convert pixel coordinates to normalized camera coordinates (without distortion)
    Eigen::Vector2d pixelToNormalized(double u, double v) {
        double x_norm = (u - intrinsics_.cx) / intrinsics_.fx;
        double y_norm = (v - intrinsics_.cy) / intrinsics_.fy;
        return Eigen::Vector2d(x_norm, y_norm);
    }

    // Undistort pixel coordinates using iterative method
    Eigen::Vector2d undistortPixel(double u, double v) {
        // First get normalized coordinates
        Eigen::Vector2d distorted = pixelToNormalized(u, v);

        // Iteratively solve for undistorted coordinates
        Eigen::Vector2d undistorted = distorted;  // Initial guess

        const int max_iterations = 10;
        const double tolerance = 1e-10;

        for (int i = 0; i < max_iterations; ++i) {
            double x = undistorted.x();
            double y = undistorted.y();
            double r2 = x*x + y*y;
            double r4 = r2 * r2;
            double r6 = r4 * r2;

            // Radial distortion
            double radial_factor = 1 + intrinsics_.k1*r2 + intrinsics_.k2*r4 + intrinsics_.k3*r6;

            // Tangential distortion
            double dx = 2*intrinsics_.p1*x*y + intrinsics_.p2*(r2 + 2*x*x);
            double dy = intrinsics_.p1*(r2 + 2*y*y) + 2*intrinsics_.p2*x*y;

            // Apply distortion model
            Eigen::Vector2d distorted_estimate;
            distorted_estimate.x() = x * radial_factor + dx;
            distorted_estimate.y() = y * radial_factor + dy;

            // Compute error
            Eigen::Vector2d error = distorted - distorted_estimate;

            if (error.norm() < tolerance) {
                break;
            }

            // Update estimate (simple fixed-point iteration)
            undistorted = distorted - (distorted_estimate - undistorted);
        }

        return undistorted;
    }

public:
    // Helper function to print ray information
    static void printRay(const Ray3D& ray, const std::string& name = "Ray") {
        std::cout << name << ":\n";
        std::cout << "  Origin (ENU): [" << ray.origin.transpose() << "]\n";
        std::cout << "  Direction (ENU): [" << ray.direction.transpose() << "]\n";
        std::cout << "  Direction magnitude: " << ray.direction.norm() << "\n\n";
    }
};

// // Example usage
// int main() {
//     // Example camera intrinsics (typical values)
//     CameraRayToENU::CameraIntrinsics intrinsics;
//     intrinsics.fx = 500.0;  // focal length x in pixels
//     intrinsics.fy = 500.0;  // focal length y in pixels
//     intrinsics.cx = 320.0;  // principal point x
//     intrinsics.cy = 240.0;  // principal point y
//     intrinsics.k1 = 0.1;    // radial distortion
//     intrinsics.k2 = -0.05;
//     intrinsics.k3 = 0.01;
//     intrinsics.p1 = 0.001;  // tangential distortion
//     intrinsics.p2 = -0.001;

//     // Example camera extrinsics
//     CameraRayToENU::CameraExtrinsics extrinsics;

//     // Camera looking north, slightly down
//     // This rotation would rotate ENU frame to camera frame where:
//     // - camera Z points north and slightly down
//     // - camera X points east
//     // - camera Y points down-ish
//     double angle = -0.1;  // Small downward tilt
//     extrinsics.R_enu_to_cam <<
//         1,  0,           0,
//         0,  0,           1,
//         0, -1,           0;

//     // Apply small rotation around X (east) axis for downward tilt
//     Eigen::Matrix3d tilt;
//     tilt << 1,         0,          0,
//             0, cos(angle), -sin(angle),
//             0, sin(angle),  cos(angle);
//     extrinsics.R_enu_to_cam = tilt * extrinsics.R_enu_to_cam;

//     // Camera position: 10m east, 20m north, 5m up
//     extrinsics.t_cam_in_enu << 10.0, 20.0, 5.0;

//     // Create converter
//     CameraRayToENU converter(intrinsics, extrinsics);

//     // Test various pixels
//     std::cout << "=== Ray Generation Examples ===\n\n";

//     // Center pixel
//     auto ray1 = converter.pixelToRayENU(320, 240);
//     CameraRayToENU::printRay(ray1, "Center pixel (320, 240)");

//     // Top-left corner
//     auto ray2 = converter.pixelToRayENU(0, 0);
//     CameraRayToENU::printRay(ray2, "Top-left pixel (0, 0)");

//     // Bottom-right corner
//     auto ray3 = converter.pixelToRayENU(639, 479);
//     CameraRayToENU::printRay(ray3, "Bottom-right pixel (639, 479)");

//     // Compare with and without distortion
//     std::cout << "=== Distortion Comparison ===\n\n";
//     auto ray_distorted = converter.pixelToRayENU(100, 100, true);
//     auto ray_undistorted = converter.pixelToRayENU(100, 100, false);

//     CameraRayToENU::printRay(ray_distorted, "Pixel (100, 100) with distortion correction");
//     CameraRayToENU::printRay(ray_undistorted, "Pixel (100, 100) without distortion correction");

//     // Angle between rays
//     double angle_diff = acos(ray_distorted.direction.dot(ray_undistorted.direction)) * 180.0 / M_PI;
//     std::cout << "Angular difference due to distortion: " << angle_diff << " degrees\n";

//     return 0;
// }