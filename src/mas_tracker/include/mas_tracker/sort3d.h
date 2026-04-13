#pragma once

#include "hungarian.h"
#include "kalman_filter.h"
#include <memory>    // For std::shared_ptr
#include <vector>    // For std::vector
#include <set>       // For std::set
#include <deque>     // For std::deque
#include <string>    // For std::string
#include <algorithm> // For std::max, std::min

#include <Eigen/Dense> // For Eigen::VectorXd

// ROS 2 Message Types
#include <vision_msgs/msg/bounding_box3_d.hpp>
#include <vision_msgs/msg/detection3_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <vision_msgs/msg/object_hypothesis.hpp> // For ObjectHypothesis inside ObjectHypothesisWithPose
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>


namespace mas_tracker{

class Tracker3D{
public:
    /**
     * @brief Updates tracker's internal state (cx_, cy_, etc.) from an Eigen vector.
     * The vector is assumed to be [cx, cy, cz, sx, sy, sz].
     */
    void fromEigen(const Eigen::VectorXd& v){
        if (v.size() >= 6) {
            cx_ = v(0); cy_ = v(1); cz_ = v(2);
            sx_ = v(3); sy_ = v(4); sz_ = v(5);
        }
    }

    /**
     * @brief Converts tracker's internal state (cx_, cy_, etc.) to an Eigen vector.
     * @return Eigen vector [cx, cy, cz, sx, sy, sz].
     */
    Eigen::VectorXd toEigen() const { // Mark as const
        Eigen::VectorXd v = Eigen::VectorXd::Zero(6);
        v << cx_, cy_, cz_, sx_, sy_, sz_;
        return v;
    }

    /**
     * @brief Converts a vision_msgs::msg::BoundingBox3D to an Eigen vector.
     * @param bbox The input bounding box.
     * @return Eigen vector [center.x, center.y, center.z, size.x, size.y, size.z].
     */
    Eigen::VectorXd toEigen(const vision_msgs::msg::BoundingBox3D& bbox) const { // Mark as const
        Eigen::VectorXd v = Eigen::VectorXd::Zero(6);
        v << bbox.center.position.x, bbox.center.position.y, bbox.center.position.z,
             bbox.size.x, bbox.size.y, bbox.size.z;
        return v;
    }

    Tracker3D(const vision_msgs::msg::BoundingBox3D& initial_bbox, int min_hits_threshold);

    // Tracker attributes
    int m_time_since_update_; // Frames since last successful update (age)
    int m_hits_;              // Total number of successful updates
    int m_hit_streak_;        // Number of consecutive successful updates
    int m_id_;                // Unique tracker ID

    // Estimated state
    double cx_, cy_, cz_; // Center coordinates
    double sx_, sy_, sz_; // Size (dimensions)

    // History for smoothing or other purposes (optional)
    std::deque<double> x_deq_, y_deq_, z_deq_;
    static const int history_size_ = 100; // Made static const
    bool is_valid_ = false; // Becomes true after min_hits_

    void updateHistory();
    void predict();
    void update(const vision_msgs::msg::BoundingBox3D& bbox);
    Eigen::VectorXd getStateVector() const; // Get full Kalman state
    vision_msgs::msg::Detection3D getAsDetection3D() const; // Get current state as a Detection3D message

    vision_msgs::msg::BoundingBox3D last_associated_bbox_; // Store the last bbox it was updated with

    static int kf_count_;   // Static counter to assign unique IDs
    int min_hits_threshold_; // Minimum hits to be considered valid

    std::shared_ptr<KF> kf_; // Kalman Filter instance
};

class Sort3D {
public:
    Sort3D(unsigned int max_age = 10, unsigned int min_hits = 3, double dist_threshold = 1.0);

    /**
     * @brief Updates the tracker states with new detections.
     * @param detections A vector of new detections in the current frame.
     * @return A vector of Detection3D messages representing active and valid tracks.
     */
    std::vector<vision_msgs::msg::Detection3D> update(const std::vector<vision_msgs::msg::Detection3D>& detections);

    bool isTrackAlive(int target_id) const; // Check if a track with this ID exists
    // Optional: Get a specific tracker (e.g., for debugging or specific target interaction)
    // bool getTrackerById(int target_id, Tracker3D& out_tracker) const; // Be careful with returning references to vector elements

private:
    HungarianAlgorithm hungarian_solver_; // Hungarian algorithm instance

    unsigned int max_age_;    // Max frames a track can survive without updates
    unsigned int min_hits_;   // Min hits for a track to be considered valid
    double dist_threshold_;   // Max distance for associating a detection to a track

    std::vector<Tracker3D> active_trackers_; // List of currently active trackers

    // Association cost functions (distance is used here)
    double calculateDistance(const Tracker3D& tracker, const vision_msgs::msg::Detection3D& detection) const;

    // Optional: IoU calculation if needed for other purposes or alternative association
    // double calculateIoU(const Tracker3D& tracker, const vision_msgs::msg::Detection3D& detection) const;
};

} // namespace mas_tracker{
