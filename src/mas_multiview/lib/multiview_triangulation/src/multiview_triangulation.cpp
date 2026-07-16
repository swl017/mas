/**
 * @file multiview_triangulation.cpp
 * @brief Multi-view triangulation class definitions with the triangulation from multiple views
 */

#include "multiview_triangulation.h"

namespace MultiView {

namespace {
// Angular uncertainty [rad] to use for a precomputed camera's ray residual.
inline double bearingSigmaOf(const Camera& cam)
{
    return cam.bearing_sigma_ > 0.0 ? cam.bearing_sigma_ : kDefaultBearingSigmaRad;
}
}  // namespace

MultiViewTriangulation::MultiViewTriangulation(/* args */)
: max_solve_time_(0.1)
{
}

MultiViewTriangulation::~MultiViewTriangulation()
{
}

void MultiViewTriangulation::addCamera(int id)
{
    Camera camera;
    camera.id_ = id;
    cameras_.push_back(camera);
}

void MultiViewTriangulation::setCameraWidthHeight(int id, double width, double height)
{
    cameras_[id].width_ = width;
    cameras_[id].height_ = height;
}

void MultiViewTriangulation::setCameraIntrinsics(int id, const Eigen::Matrix3d& K)
{
    cameras_[id].K_ = K;
}

void MultiViewTriangulation::setCameraExtrinsics(int id, const Eigen::Matrix3d& R, const Eigen::Vector3d& t)
{
    cameras_[id].R_ = R;
    cameras_[id].t_ = t;
}

void MultiViewTriangulation::setCameraPoseCovariance(int id, const Eigen::Matrix<double, 6, 6>& cov)
{
    cameras_[id].pose_covariance_ = cov;
}

void MultiViewTriangulation::setCameraGimbalAngles(int id, const Eigen::Vector3d& angles)
{
    cameras_[id].gimbal_angles_ = angles;
}

void MultiViewTriangulation::setCameraBearingSigma(int id, double sigma_rad)
{
    cameras_[id].bearing_sigma_ = sigma_rad;
}

void MultiViewTriangulation::addDetection(int id, const Detection::Detection2D& detection2d)
{
    cameras_[id].addDetection2D(detection2d);
}

void MultiViewTriangulation::addPrecomputedRay(int id, const Eigen::Vector3d& ray_origin,
                                                const Eigen::Vector3d& ray_direction,
                                                const std::string& class_id)
{
    cameras_[id].addPrecomputedRay(ray_origin, ray_direction, class_id);
}

void MultiViewTriangulation::resetCameraDetections(int id)
{
    cameras_[id].detections_.clear();
}

std::vector<Detection::Detection3D> MultiViewTriangulation::get3DDetectionsFromCamera(int id)
{
    std::vector<Detection::Detection3D> det_3ds;
    for (const Detection& detection : cameras_[id].detections_) {
        det_3ds.push_back(detection.detection3d);
    }
    return det_3ds;
}

void MultiViewTriangulation::Triangulate()
{
    results_.clear();
    std::vector<Camera>& cameras = cameras_;
    filterDetections(cameras);
    std::vector<Association> associations;
    associateDetections(cameras, associations);
    estimateInitialPosition(cameras, associations);
    for (Association& association : associations) {
        Eigen::Vector3d refined_position;
        Eigen::Matrix3d covariance;
        std::vector<double> reprojection_errors;
        optimizePositionEstimate(cameras, association);
    }

    // Get results with reprojection error < max_reprojection_error_ (pixels)
    for (const Association& association : associations) {
        if (!association.reprojection_errors.empty() &&
            std::all_of(association.reprojection_errors.begin(),
                        association.reprojection_errors.end(),
                        [this](double error) { return error < max_reprojection_error_; })) {
            Detection::Detection3D detection3d;
            detection3d.position = association.refined_position;
            detection3d.covariance = association.covariance;
            // Collect detection_ids from contributing cameras
            for (size_t k = 0; k < association.camera_idx.size(); ++k) {
                int cam = association.camera_idx[k];
                int det = association.detection_idx[k];
                detection3d.detection_ids.push_back(
                    cameras_[cam].detections_[det].detection2d.class_id);
            }
            results_.push_back(detection3d);
        }
    }

    // Get results for each camera
    for (Camera& camera : cameras) {
        for (size_t det_idx = 0; det_idx < camera.detections_.size(); det_idx++) {
        // for (Detection& detection : camera.detections_) {
            double min_reprojection_error = std::numeric_limits<double>::max();
            for (Association& association : associations) {
                for (size_t i = 0; i < association.camera_idx.size(); i++) {
                    if (camera.id_ == association.camera_idx[i] && det_idx == association.detection_idx[i]) {
                        if (association.reprojection_errors[i] < min_reprojection_error) {
                            min_reprojection_error = association.reprojection_errors[i];
                            camera.detections_[det_idx].detection3d.position = association.refined_position;
                            camera.detections_[det_idx].detection3d.covariance = association.covariance;
                        }
                    }
                }
            }
        }
    }
}

void MultiViewTriangulation::printDetections(const std::vector<Camera>& cameras)
{
    for (const Camera& camera : cameras) {
        std::cout << "Camera " << camera.id_ << std::endl;
        if (camera.is_precomputed_) {
            // Transmitted-ray (cooperative peer) camera: no K_/width_/height_ to print
            // (ticket 020 — do not read uninitialized intrinsics). Print the ray instead.
            std::cout << "precomputed transmitted ray (no local image model)" << std::endl;
            std::cout << "origin: " << camera.t_.transpose() << std::endl;
            std::cout << "bearing_sigma [rad]: " << camera.bearing_sigma_ << std::endl;
        } else {
            std::cout << "K: " << std::endl << camera.K_ << std::endl;
            std::cout << "R: " << std::endl << camera.R_ << std::endl;
            std::cout << "t: " << camera.t_.transpose() << std::endl;
            std::cout << "width: " << camera.width_ << std::endl;
            std::cout << "height: " << camera.height_ << std::endl;
        }

        int i = 0;
        for (const Detection& detection : camera.detections_) {
            std::cout << "Detection " << i << std::endl;
            if (camera.is_precomputed_) {
                std::cout << "- ray_dir: "
                          << detection.detection2d.ray.ray_direction.transpose() << std::endl;
            } else {
                std::cout << "- center: " << detection.detection2d.center.transpose() << std::endl;
                std::cout << "- width: " << detection.detection2d.width << std::endl;
                std::cout << "- height: " << detection.detection2d.height << std::endl;
            }
            i++;
        }
        std::cout << "----------------" << std::endl;
    }
}


void MultiViewTriangulation::filterDetections(std::vector<Camera>& cameras)
{
    for (int i = 0; i < cameras.size(); ++i) {
        for (int j = 0; j < cameras.size(); ++j) {
            // Filter detections
            if (i == j) {
                continue;
            }
            // 1. Other cameras: Calculate the distance between the ray and the point
            std::vector<int> detections_to_remove;
            for (int k = 0; k < cameras[i].detections_.size(); ++k) {
                if (cameras[i].detections_[k].detection2d.ray.distanceToPoint(cameras[j].t_) < 2.0) {
                    detections_to_remove.push_back(k);
                }
            }
            for (int k = detections_to_remove.size() - 1; k >= 0; --k) {
                cameras[i].detections_.erase(cameras[i].detections_.begin() + detections_to_remove[k]);
            }
            // 2. Common FOV region @todo
            // 3. Manually specified regions @todo
        }
    }
}

void MultiViewTriangulation::associateDetections(std::vector<Camera>& cameras, std::vector<Association>& associations)
{
    std::vector<std::vector<int>> sets;
    std::vector<int> nonempty_camera_idx;
    for (int i = 0; i < cameras.size(); i++) {
        std::vector<int> camera_detections;
        for (int j = 0; j < cameras[i].detections_.size(); j++) {
            camera_detections.push_back(j);
        }
        if (camera_detections.size() > 0) {
            sets.push_back(camera_detections);
            nonempty_camera_idx.push_back(i);
        }
    }
    std::vector<std::vector<int>> all_combinations = generateCombinations(sets);
    std::cout << "Total combinations: " << all_combinations.size() << std::endl;
    int ii = 0;
    for (const std::vector<int>& combination : all_combinations) {
        Association association;
        std::cout << "Combination " << ii++ << ": ";
        for (int i = 0; i < combination.size(); i++) {
            association.camera_idx.push_back(nonempty_camera_idx[i]);
            association.detection_idx.push_back(combination[i]);
            std::cout << "Cam" << nonempty_camera_idx[i]
                      << "Det" << combination[i] << "-";
        }
        std::cout << std::endl;
        if (association.camera_idx.size() < 2) {
            std::cout << "Skipping association with less than 2 cameras." << std::endl;
            continue;
        }
        associations.push_back(association);
    }
}

void MultiViewTriangulation::estimateInitialPosition(std::vector<Camera>& cameras, std::vector<Association>& associations)
{

    std::vector<int> associations_to_remove;
    // Mid point method
    for (size_t k = 0; k < associations.size(); k++) {
        Association& association = associations[k];
        Eigen::Vector3d weighted_mid_point = Eigen::Vector3d::Zero();
        double total_weight = 0.0;
        int num_mid_points = 0;
        for (size_t i = 0; i < association.camera_idx.size() - 1; i++) {
            const Camera& camera1 = cameras[association.camera_idx[i]];
            const Detection& detection1 = camera1.detections_[association.detection_idx[i]];
            Eigen::Vector3d v1 = detection1.detection2d.ray.ray_direction;
            Eigen::Vector3d p1 = detection1.detection2d.ray.ray_origin;
            double a = v1.dot(v1);

            for (size_t j = i+1; j < association.camera_idx.size(); j++) {
                const Camera& camera2 = cameras[association.camera_idx[j]];
                const Detection& detection2 = camera2.detections_[association.detection_idx[j]];

                // Mid point method
                Eigen::Vector3d v2 = detection2.detection2d.ray.ray_direction;
                Eigen::Vector3d p2 = detection2.detection2d.ray.ray_origin;

                Eigen::Vector3d w0 = p1 - p2;

                double b = v1.dot(v2);
                double c = v2.dot(v2);
                double d = v1.dot(w0);
                double e = v2.dot(w0);

                double sc = (b*e - c*d) / (a*c - b*b);
                double tc = (a*e - b*d) / (a*c - b*b);

                Eigen::Vector3d mid_point = 0.5 * (p1 + sc * v1 + p2 + tc * v2);

                // A precomputed-ray (cooperative peer) camera has no local image model
                // (no K_): use the point-to-ray perpendicular distance [m] as its error
                // metric; a raw camera uses pixel reprojection. Branch on is_precomputed_
                // (never read K_ on a ray-only camera) — ticket 020.
                double reprojection1 = camera1.is_precomputed_
                    ? PointToRayError(detection1.detection2d.ray.ray_origin,
                                      detection1.detection2d.ray.ray_direction,
                                      bearingSigmaOf(camera1)).getRayDistance(mid_point)
                    : ReprojectionError(camera1, detection1.detection2d.center).getReprojectionError(mid_point);
                double reprojection2 = camera2.is_precomputed_
                    ? PointToRayError(detection2.detection2d.ray.ray_origin,
                                      detection2.detection2d.ray.ray_direction,
                                      bearingSigmaOf(camera2)).getRayDistance(mid_point)
                    : ReprojectionError(camera2, detection2.detection2d.center).getReprojectionError(mid_point);

                double weight = 1.0 / (reprojection1 + reprojection2 + 1e-12);  // Add small constant to avoid division by zero

                /**
                 * @todo Add more filtering criteria (e.g. common FOV region, manually specified regions)
                 */
                // A precomputed-ray camera transmitted a RAY validated at the source, so the
                // pixel-reprojection init gate structurally does not apply to it (ticket 019
                // fair ego/peer split, user-authorized 2026-07-15; ticket 020 makes the ray a
                // first-class residual downstream). The pair is still constrained geometrically
                // (the ray intersection above) + the imaged camera's reprojection.
                bool ok1 = camera1.is_precomputed_ || (reprojection1 < camera1.width_ * 0.2);
                bool ok2 = camera2.is_precomputed_ || (reprojection2 < camera2.width_ * 0.2);
                if (ok1 && ok2) {
                    weighted_mid_point += weight * mid_point;
                    total_weight += weight;
                }
                else {
                    // If reprojection error is too high, remove association
                    associations_to_remove.push_back(k);
                    break;  // Break the inner loop to avoid further processing
                }
                // weighted_mid_point += mid_point;
                num_mid_points++;
                std::cout << "Cam" << association.camera_idx[i]
                            << "Det" << association.detection_idx[i]
                            << "-Cam" << association.camera_idx[j]
                            << "Det" << association.detection_idx[j] << std::endl;
                std::cout << "- Mid-point: " << mid_point.transpose() << std::endl;
                std::cout << "- Reprojection1: " << reprojection1 << std::endl;
                std::cout << "- Reprojection2: " << reprojection2 << std::endl;
                std::cout << "- Weight: " << weight << std::endl;

            }
        }
        // Normalize the weighted mid-point
        if (total_weight > 0) {
            weighted_mid_point /= total_weight;
            // Store initial guess
            association.initial_guess = weighted_mid_point;
            std::cout << "- Weighted mid-point: " << weighted_mid_point.transpose() << std::endl;
            std::cout << "- Total weight: " << total_weight << std::endl;
            std::cout << "----------------" << std::endl;

            for (size_t i = 0; i < association.camera_idx.size(); i++) {
                cameras[association.camera_idx[i]].detections_[association.detection_idx[i]].detection3d.position = weighted_mid_point;
            }
        }
        else {
            // No valid mid-point — skip this association to avoid zero initial guess
            associations_to_remove.push_back(k);
            std::cout << "Skipping association " << k << ": total_weight <= 0" << std::endl;
        }
    }

    // Remove associations with high reprojection error
    for (int i = associations_to_remove.size() - 1; i >= 0; --i) {
        int idx = associations_to_remove[i];
        if (idx >= 0 && idx < associations.size()) {
            associations.erase(associations.begin() + idx);
        }
    }
}

void MultiViewTriangulation::optimizePositionEstimate(std::vector<Camera>& cameras, Association& association)

{
    ceres::Problem problem;
    double position[3] = {association.initial_guess[0], association.initial_guess[1], association.initial_guess[2]};

    for (size_t i = 0; i < association.camera_idx.size(); i++) {
        Camera& cam = cameras[association.camera_idx[i]];
        const Detection& det = cam.detections_[association.detection_idx[i]];
        if (cam.is_precomputed_) {
            // Transmitted-ray peer: 1-DOF angular (point-to-ray) residual, no K_ (ticket 020).
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<PointToRayError, 1, 3>(
                    new PointToRayError(det.detection2d.ray.ray_origin,
                                        det.detection2d.ray.ray_direction,
                                        bearingSigmaOf(cam)));
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), position);
        } else {
            // Raw camera: 2-DOF pixel-reprojection residual.
            ceres::CostFunction* cost_function =
                new ceres::AutoDiffCostFunction<ReprojectionError, 2, 3>(
                    new ReprojectionError(cam, det.detection2d.center));
            problem.AddResidualBlock(cost_function, new ceres::HuberLoss(1.0), position);
        }
        // Use TrivialLoss for very accurate measurements
        // problem.AddResidualBlock(cost_function, new ceres::TrivialLoss(), position);
    }

    ceres::CostFunction* regularization =
        new ceres::AutoDiffCostFunction<RegularizationError, 3, 3>(
            new RegularizationError(association.initial_guess));
    problem.AddResidualBlock(regularization, nullptr, position);

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.minimizer_progress_to_stdout = false;

    if (real_time_mode_) {
        options.max_solver_time_in_seconds = max_solve_time_;
    }

    options.function_tolerance = 1e-6;
    options.gradient_tolerance = 1e-10;
    options.parameter_tolerance = 1e-8;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    if (!summary.IsSolutionUsable()) {
        std::cout << "Ceres solve failed: " << summary.message << std::endl;
        // Mark association as invalid by setting extreme reprojection errors
        association.reprojection_errors.assign(association.camera_idx.size(),
                                                std::numeric_limits<double>::max());
        association.refined_position = Eigen::Vector3d::Zero();
        association.covariance = Eigen::Matrix3d::Identity() * std::numeric_limits<double>::max();
        return;
    }

    Eigen::Vector3d refined_position;
    Eigen::Matrix3d covariance;
    std::vector<double> reprojection_errors;

    refined_position << position[0], position[1], position[2];

    for (size_t i = 0; i < association.camera_idx.size(); i++) {
        Camera& cam = cameras[association.camera_idx[i]];
        const Detection& det = cam.detections_[association.detection_idx[i]];
        if (cam.is_precomputed_) {
            // Post-solve metric for a transmitted ray = perpendicular ray distance [m]
            // (NOTE: this entry is in metres, whereas raw cameras report pixels — the results
            // gate max_reprojection_error_ mixes the two; a dedicated ray-distance threshold is
            // a documented v1 limitation, ticket 020).
            reprojection_errors.push_back(
                PointToRayError(det.detection2d.ray.ray_origin,
                                det.detection2d.ray.ray_direction,
                                bearingSigmaOf(cam)).getRayDistance(refined_position));
        } else {
            reprojection_errors.push_back(
                ReprojectionError(cam, det.detection2d.center).getReprojectionError(refined_position));
        }
    }

    // Compute covariance via first-order Jacobian propagation (sandwich formula)
    covariance = propagateCovariance(
        refined_position, cameras, association.camera_idx, covariance_config_);

    association.refined_position = refined_position;
    association.covariance = covariance;
    association.reprojection_errors = reprojection_errors;
}
}  // namespace MultiView
