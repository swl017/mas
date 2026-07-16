#include "camera.h"
#include "covariance_propagation.h"
#include "detection.h"
#include "ray.h"
#include "reprojection.h"
#include "cartesian_product.h"

#include <mutex>
#include <Eigen/Eigen>
#include <ceres/ceres.h>

namespace MultiView {

class MultiViewTriangulation
{
public:
    MultiViewTriangulation(/* args */);
    ~MultiViewTriangulation();

    void Triangulate();
    void Triangulate(const double& max_solve_time, const double& max_reprojection_error)
    {
        max_solve_time_ = max_solve_time;
        max_reprojection_error_ = max_reprojection_error;
        Triangulate();
    }

    /**
     * @brief Permute over one detection from each camera, for one target entity.
     */
    struct Association {
        std::vector<int> camera_idx;
        std::vector<int> detection_idx;
        Eigen::Vector3d initial_guess;
        Eigen::Vector3d refined_position;
        Eigen::Matrix3d covariance;
        std::vector<double> reprojection_errors;
    };

    // Camera functions
    void addCamera(int id);
    void setCameraWidthHeight(int id, double width, double height);
    void setCameraIntrinsics(int id, const Eigen::Matrix3d& K);
    void setCameraExtrinsics(int id, const Eigen::Matrix3d& R, const Eigen::Vector3d& t);
    void setCameraPoseCovariance(int id, const Eigen::Matrix<double, 6, 6>& cov);
    void setCameraGimbalAngles(int id, const Eigen::Vector3d& angles);
    // Per-camera angular uncertainty [rad] for a transmitted (precomputed) ray (ticket 020).
    void setCameraBearingSigma(int id, double sigma_rad);
    void setCovarianceConfig(const CovarianceConfig& config) { covariance_config_ = config; }
    void addDetection(int id, const Detection::Detection2D& detection2d);
    void addPrecomputedRay(int id, const Eigen::Vector3d& ray_origin,
                           const Eigen::Vector3d& ray_direction,
                           const std::string& class_id);
    void resetCameraDetections(int id);
    int getNumCameras() { return cameras_.size(); }
    std::vector<Detection::Detection3D> get3DDetectionsFromCamera(int id);

    std::vector<Detection::Detection3D> results_;
    std::vector<Camera> cameras_;
    CovarianceConfig covariance_config_;
    void printDetections(const std::vector<Camera>& cameras);

private:


    /**
     * @brief Filtering detections (i.e. exclude 1. other cameras, 2. common FOV region, 3. manually specified regions)
     */
    void filterDetections(std::vector<Camera>& cameras);

    /**
     * @brief Associate detections from different cameras. All possible combinations implemented now.
     * @todo How to implement associations effectively?
     */
    void associateDetections(std::vector<Camera>& cameras, std::vector<Association>& associations);

    /**
     * @brief Estimate initial position of each associated detections using midpoint method
     */
    void estimateInitialPosition(std::vector<Camera>& cameras, std::vector<Association>& associations);

    /**
     * @brief Optimize the position estimate
     * @return
     */
    void optimizePositionEstimate(std::vector<Camera>& cameras, Association& association);

    bool real_time_mode_;
    double max_solve_time_;
    double max_reprojection_error_;
};
}  // namespace MultiView