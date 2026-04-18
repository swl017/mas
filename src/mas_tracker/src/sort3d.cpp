#include <mas_tracker/sort3d.h>
#include <algorithm> // For std::max, std::min, std::set_difference
#include <cmath>     // For std::sqrt, std::pow, std::fabs
#include <cfloat>    // For DBL_EPSILON, or use <limits>
#include <iterator>  // For std::inserter
#include <set>       // For std::set

namespace mas_tracker{

// Initialize static member of Tracker3D
int Tracker3D::kf_count_ = 0;
const int Tracker3D::history_size_; // Definition for static const int if not in C++17 inline

Tracker3D::Tracker3D(const vision_msgs::msg::BoundingBox3D& initial_bbox, int min_hits_threshold)
    : m_time_since_update_(0),
      m_hits_(0),
      m_hit_streak_(0),
      min_hits_threshold_(min_hits_threshold),
      last_associated_bbox_(initial_bbox)
{
    kf_count_++;
    m_id_ = kf_count_;

    // Initialize Kalman Filter (12 states: x,y,z, sx,sy,sz, vx,vy,vz, vsx,vsy,vsz; 6 measurements: x,y,z, sx,sy,sz)
    kf_ = std::make_shared<KF>(12, 6);

    // Define State Transition Matrix (F) for constant velocity model
    //  x = x + vx*dt
    // sx = sx + vsx*dt
    // dt is implicitly 1 frame here
    kf_->F_.setIdentity();
    for (int i = 0; i < 6; ++i) { // Link positions/sizes to their velocities
        kf_->F_(i, i + 6) = 1.0; // e.g., x(k) = x(k-1) + vx(k-1)*1
    }

    // Define Measurement Matrix (H)
    // We directly observe position and size (first 6 state variables)
    kf_->H_.setZero();
    for (int i = 0; i < 6; ++i) {
        kf_->H_(i, i) = 1.0;
    }

    // Define Covariance Matrices
    // Initial state covariance (P0) - higher uncertainty for velocities
    kf_->init_state_cov_.setIdentity();
    kf_->init_state_cov_.diagonal() << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, // Position/size variance
                                      1.0, 1.0, 1.0, 1.0, 1.0, 1.0;  // Velocity variance
    kf_->state_cov_ = kf_->init_state_cov_;

    // Process noise covariance (Q) - accounts for model inaccuracies
    kf_->process_noise_cov_.setIdentity();
    kf_->process_noise_cov_.diagonal() << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, // Pos/size process noise
                                         0.05, 0.05, 0.05, 0.05, 0.05, 0.05; // Vel process noise

    // Measurement noise covariance (R) - accounts for sensor noise
    kf_->measurement_cov_.setIdentity();
    kf_->measurement_cov_.diagonal() << 0.04, 0.04, 0.04,  // Position measurement variance
                                       0.01, 0.01, 0.01;   // Size measurement variance

    // Initialize Kalman Filter state vector
    kf_->state_.head(6) = toEigen(initial_bbox); // x,y,z, sx,sy,sz
    kf_->state_.tail(6).setZero();               // vx,vy,vz, vsx,vsy,vsz initialized to 0

    // Initialize Tracker3D's own state variables from KF state
    fromEigen(kf_->state_.head(6));
}

void Tracker3D::updateHistory(){
    x_deq_.push_back(cx_);
    y_deq_.push_back(cy_);
    z_deq_.push_back(cz_);

    // Limit history size
    while (x_deq_.size() > static_cast<size_t>(history_size_)) x_deq_.pop_front();
    while (y_deq_.size() > static_cast<size_t>(history_size_)) y_deq_.pop_front();
    while (z_deq_.size() > static_cast<size_t>(history_size_)) z_deq_.pop_front();
}

void Tracker3D::predict(){
    kf_->predict(); // Perform Kalman prediction
    fromEigen(kf_->state_.head(6)); // Update tracker's representation from KF's predicted state

    m_time_since_update_++; // Increment age
}

void Tracker3D::update(const vision_msgs::msg::BoundingBox3D& bbox){
    last_associated_bbox_ = bbox; // Store the measurement
    Eigen::VectorXd measurement = toEigen(bbox);

    kf_->update(measurement); // Perform Kalman update
    fromEigen(kf_->state_.head(6)); // Update tracker's representation from KF's corrected state

    m_time_since_update_ = 0; // Reset age
    m_hits_++;
    m_hit_streak_++;

    // Alternative validation logic: Use hit count instead of consecutive hits
    // This is more forgiving for detectors with inconsistent output
    if (!is_valid_ && m_hits_ >= min_hits_threshold_){
        is_valid_ = true;
    }
    updateHistory();
}

Eigen::VectorXd Tracker3D::getStateVector() const {
    return kf_->state_;
}

vision_msgs::msg::Detection3D Tracker3D::getAsDetection3D() const {
    vision_msgs::msg::Detection3D det3d;

    // Populate BoundingBox3D from current KF state (more accurate than last_associated_bbox_ for prediction)
    det3d.bbox.center.position.x = kf_->state_(0);
    det3d.bbox.center.position.y = kf_->state_(1);
    det3d.bbox.center.position.z = kf_->state_(2);
    det3d.bbox.size.x = std::max(0.01, kf_->state_(3)); // Ensure non-zero positive size
    det3d.bbox.size.y = std::max(0.01, kf_->state_(4));
    det3d.bbox.size.z = std::max(0.01, kf_->state_(5));
    det3d.bbox.center.orientation = last_associated_bbox_.center.orientation; // Keep orientation from last measurement

    // Create hypothesis
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = std::to_string(m_id_); // Use tracker ID as class_id
    hyp.hypothesis.score = 1.0; // Confidence score (e.g., 1.0 for tracked objects, or derive from KF covariance)

    // Set pose in hypothesis (can be same as bbox center)
    hyp.pose.pose.position = det3d.bbox.center.position;
    hyp.pose.pose.orientation = det3d.bbox.center.orientation;

    // Copy the 3x3 position block of the Kalman posterior covariance into the
    // 6x6 pose covariance (position is the first 3 state dims). Downstream
    // consumers (mas_policy triangulation tail) read the diagonal at [0,7,14].
    const Eigen::MatrixXd& P = kf_->state_cov_;
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            hyp.pose.covariance[r * 6 + c] = P(r, c);
        }
    }

    det3d.results.push_back(hyp);
    // det3d.header can be set by the caller (Sort3DNode) with current timestamp and frame_id
    return det3d;
}

Sort3D::Sort3D(unsigned int max_age, unsigned int min_hits, double dist_threshold)
    : max_age_(max_age), min_hits_(min_hits), dist_threshold_(dist_threshold){
}

std::vector<vision_msgs::msg::Detection3D> Sort3D::update(const std::vector<vision_msgs::msg::Detection3D>& detections){
    // --- 1. Predict new locations of existing trackers ---
    for(auto& tracker : active_trackers_) {
        tracker.predict();
    }

    // --- 2. Associate detections with existing trackers ---
    std::vector<std::pair<int, int>> matched_pairs; // Stores (tracker_idx, detection_idx)
    std::set<int> unmatched_detection_indices;
    for(size_t i = 0; i < detections.size(); ++i) unmatched_detection_indices.insert(i);

    std::set<int> matched_tracker_indices;

    if (!active_trackers_.empty() && !detections.empty()) {
        // Create cost matrix (distance-based)
        std::vector<std::vector<double>> cost_matrix(active_trackers_.size(), std::vector<double>(detections.size()));
        for (size_t i = 0; i < active_trackers_.size(); ++i) {
            for (size_t j = 0; j < detections.size(); ++j) {
                cost_matrix[i][j] = calculateDistance(active_trackers_[i], detections[j]);
            }
        }

        // Solve assignment problem using Hungarian algorithm
        std::vector<int> assignment(active_trackers_.size(), -1);
        hungarian_solver_.Solve(cost_matrix, assignment);

        // Process assignments
        for (size_t tracker_idx = 0; tracker_idx < assignment.size(); ++tracker_idx) {
            int detection_idx = assignment[tracker_idx];
            if (detection_idx != -1) { // If tracker is assigned to a detection
                // Check if the assignment is valid (below distance threshold)
                if (cost_matrix[tracker_idx][detection_idx] <= dist_threshold_) {
                    matched_pairs.emplace_back(tracker_idx, detection_idx);
                    unmatched_detection_indices.erase(detection_idx);
                    matched_tracker_indices.insert(tracker_idx);
                }
            }
        }
    }

    // --- 3. Update matched trackers ---
    for (const auto& match : matched_pairs) {
        active_trackers_[match.first].update(detections[match.second].bbox);
    }

    // --- 3.5. Reset hit streak for unmatched trackers ---
    for (size_t i = 0; i < active_trackers_.size(); ++i) {
        if (matched_tracker_indices.find(i) == matched_tracker_indices.end()) {
            // This tracker was not matched, reset its hit streak
            active_trackers_[i].m_hit_streak_ = 0;
        }
    }

    // --- 4. Create new trackers for unmatched detections ---
    for (int det_idx : unmatched_detection_indices) {
        active_trackers_.emplace_back(detections[det_idx].bbox, min_hits_);
    }

    // --- 5. Manage track lifecycle and prepare output ---
    std::vector<vision_msgs::msg::Detection3D> output_tracks;
    auto it = active_trackers_.begin();
    while (it != active_trackers_.end()) {
        if (it->m_time_since_update_ > max_age_) { // Remove old tracks
            it = active_trackers_.erase(it);
        } else {
            // Output valid tracks
            // A track is output if:
            // 1. It has become valid (met min_hits requirement)
            // 2. It hasn't been lost for too long (some tolerance for missed detections)
            // This allows outputting predictions for valid tracks
            if (it->is_valid_) {
                // You can add additional conditions here if needed
                // For example, only output if updated within last N frames:
                // if (it->m_time_since_update_ < 3) { ... }
                output_tracks.push_back(it->getAsDetection3D());
            }
            ++it;
        }
    }
    return output_tracks;
}

double Sort3D::calculateDistance(const Tracker3D& tracker, const vision_msgs::msg::Detection3D& detection) const {
    double dx = tracker.cx_ - detection.bbox.center.position.x;
    double dy = tracker.cy_ - detection.bbox.center.position.y;
    double dz = tracker.cz_ - detection.bbox.center.position.z;
    // Adding a small epsilon is good practice if the Hungarian algorithm variant is sensitive to exact zeros,
    // but std::sqrt will handle non-negative results.
    return std::sqrt(dx*dx + dy*dy + dz*dz); // + DBL_EPSILON;
}

bool Sort3D::isTrackAlive(int target_id) const {
    for (const auto& tracker : active_trackers_){
        if (target_id == tracker.m_id_){
            return true;
        }
    }
    return false;
}

// Optional: Implementation for getTrackerById
// bool Sort3D::getTrackerById(int target_id, Tracker3D& out_tracker) const {
//     for (const auto& tracker : active_trackers_) {
//         if (tracker.m_id_ == target_id) {
//             out_tracker = tracker; // Copies the tracker
//             return true;
//         }
//     }
//     return false;
// }

} // namespace mas_tracker