#pragma once

#include <rclcpp/rclcpp.hpp> // Core ROS 2 header
#include <mas_tracker/sort3d.h> // Your SORT algorithm logic

// ROS 2 Standard Message Types
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/int8.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <mas_msgs/msg/triangulated_point_array.hpp>
#include <mas_msgs/msg/target_ray_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp> // If camera info is used

// Eigen (already included via sort3d.h or kalman_filter.h if they use Eigen::*)
// #include <Eigen/Dense>

#include <deque>   // For std::deque
#include <vector>  // For std::vector
#include <string>  // For std::string

// Placeholder for MAV_MSGS ROS 2 equivalent
// If mav_msgs is ported to ROS 2, its headers and types would be included here.
// For now, we'll use a simplified Eigen type for odometry.
namespace mav_msgs_ros2 { // Example placeholder namespace
    // This is a highly simplified placeholder.
    // The actual type would come from a ROS 2 port of mav_msgs.
    using EigenOdometry = Eigen::Isometry3d;

    // Placeholder for the conversion function.
    // You would need to implement this or use one from the mav_msgs ROS 2 port.
    inline bool eigenOdometryFromMsg(const nav_msgs::msg::Odometry& ros_odom_msg, EigenOdometry* eigen_odom) {
        if (!eigen_odom) return false;
        // Example conversion (simplified, assumes orientation is quaternion)
        eigen_odom->translation().x() = ros_odom_msg.pose.pose.position.x;
        eigen_odom->translation().y() = ros_odom_msg.pose.pose.position.y;
        eigen_odom->translation().z() = ros_odom_msg.pose.pose.position.z;
        Eigen::Quaterniond q(
            ros_odom_msg.pose.pose.orientation.w,
            ros_odom_msg.pose.pose.orientation.x,
            ros_odom_msg.pose.pose.orientation.y,
            ros_odom_msg.pose.pose.orientation.z
        );
        *eigen_odom = Eigen::Isometry3d(q); // Sets rotation
        eigen_odom->translation() = Eigen::Vector3d( // Re-set translation after rotation
            ros_odom_msg.pose.pose.position.x,
            ros_odom_msg.pose.pose.position.y,
            ros_odom_msg.pose.pose.position.z
        );
        // Velocities are not handled in this simple placeholder
        RCLCPP_WARN_ONCE(rclcpp::get_logger("Sort3DNode_eigenOdometryFromMsg"),
                      "Using placeholder mav_msgs::eigenOdometryFromMsg. Full functionality may be missing.");
        return true;
    }
}


namespace mas_tracker{

class Sort3DNode : public rclcpp::Node
{
public:
    // Constructor now takes rclcpp::NodeOptions
    explicit Sort3DNode(const rclcpp::NodeOptions & options);
    ~Sort3DNode() override = default;

private:
    // ROS 2 Callbacks - use const SharedPtr& for message arguments
    void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void detCallback(const mas_msgs::msg::TriangulatedPointArray::ConstSharedPtr msg);
    void autoPickCallback(const std_msgs::msg::Int8::ConstSharedPtr msg);
    void setTargetPositionCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg);
    void raysCallback(const mas_msgs::msg::TargetRayArray::ConstSharedPtr msg, int camera_index);

    // Helper functions
    vision_msgs::msg::Detection3DArray triPoints2dets(const mas_msgs::msg::TriangulatedPointArray& tri_points);
    void pubChosenTarget();
    int pickTarget(); // Selects a target based on some criteria

    // ROS 2 Publishers and Subscribers
    rclcpp::Subscription<mas_msgs::msg::TriangulatedPointArray>::SharedPtr det_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr auto_pick_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr set_target_pos_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr target_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr target_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr chosen_ray_pub_;
    std::vector<rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr> track_pubs_;

    // Per-camera ray subscriptions and cached ray arrays (indexed 0..num_cameras-1)
    std::vector<rclcpp::Subscription<mas_msgs::msg::TargetRayArray>::SharedPtr> rays_subs_;
    std::vector<mas_msgs::msg::TargetRayArray::SharedPtr> per_camera_rays_;
    int self_camera_index_ = 1;  // 1-indexed: which camera corresponds to this drone

    // Cached latest triangulated points (for detection_id lookup in ray matching)
    mas_msgs::msg::TriangulatedPointArray::SharedPtr latest_tri_points_;

    // Odometry related
    mav_msgs_ros2::EigenOdometry current_odometry_; // Using placeholder
    bool is_odom_received_ = false;

    // Tracking parameters and state
    int target_id_ = 0;      // ID of the currently chosen target
    int auto_pick_mode_ = 1; // 0: no auto pick, 1: auto pick enabled

    int num_object_classes_ = 1; // Number of object classes to track separately
    std::vector<Sort3D> sort_instances_; // One Sort3D instance per class
    // Input detections and output tracks, per class
    std::vector<std::vector<vision_msgs::msg::Detection3D>> per_class_detections_input_;
    std::vector<std::vector<vision_msgs::msg::Detection3D>> per_class_tracks_output_;

    std::string last_detection_frame_id_; // Frame ID from the last received detections

    // Parameters loaded from ROS 2 parameter server
    double association_dist_threshold_ = 1.0;
    int max_track_age_ = 30;
    int min_tracker_hits_ = 5;

    rclcpp::Clock::SharedPtr clock_; // For throttled logging

    std::vector<int> track_ids_; // IDs of the tracks
};
} // namespace mas_tracker
