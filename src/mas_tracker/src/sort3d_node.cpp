#include <mas_tracker/sort3d_node.h>
#include <tf2_eigen/tf2_eigen.h> // For Eigen conversions if needed (usually via tf2_geometry_msgs)
#include <tf2_geometry_msgs/tf2_geometry_msgs.h> // For geometry_msgs <-> TF2/Eigen conversions

#include <functional> // For std::bind and std::placeholders
#include <stdexcept>  // For std::stoi, std::invalid_argument

namespace mas_tracker{

Sort3DNode::Sort3DNode(const rclcpp::NodeOptions & options)
    : Node("sort_3d_tracking_node", options), // Node name
      clock_(this->get_clock()) // Initialize clock for throttled logging
{
    RCLCPP_INFO(this->get_logger(), "Initializing Sort3DNode...");

    // Declare and get parameters from the parameter server
    this->declare_parameter<double>("association_distance_threshold", 1.0);
    this->declare_parameter<int>("max_track_age", 30);
    this->declare_parameter<int>("min_tracker_hits_for_valid", 5);
    this->declare_parameter<int>("number_of_object_classes", 1);
    this->declare_parameter<int>("num_cameras", 3);
    this->declare_parameter<std::string>("camera_name_prefix", "/px4_");
    this->declare_parameter<int>("self_camera_index", 1);

    association_dist_threshold_ = this->get_parameter("association_distance_threshold").as_double();
    max_track_age_ = this->get_parameter("max_track_age").as_int();
    min_tracker_hits_ = this->get_parameter("min_tracker_hits_for_valid").as_int();
    num_object_classes_ = this->get_parameter("number_of_object_classes").as_int();

    RCLCPP_INFO(this->get_logger(), "Parameters loaded:");
    RCLCPP_INFO(this->get_logger(), "  Association Distance Threshold: %.2f", association_dist_threshold_);
    RCLCPP_INFO(this->get_logger(), "  Max Track Age: %d", max_track_age_);
    RCLCPP_INFO(this->get_logger(), "  Min Tracker Hits for Valid: %d", min_tracker_hits_);
    RCLCPP_INFO(this->get_logger(), "  Number of Object Classes: %d", num_object_classes_);

    // Initialize SORT instances and publishers for each class
    per_class_detections_input_.resize(num_object_classes_);
    per_class_tracks_output_.resize(num_object_classes_);
    for(int i = 0; i < num_object_classes_; ++i){
        track_pubs_.push_back(
            this->create_publisher<vision_msgs::msg::Detection3DArray>("tracked_objects/class_" + std::to_string(i), 10)
        );
        sort_instances_.emplace_back(
            static_cast<unsigned int>(max_track_age_),
            static_cast<unsigned int>(min_tracker_hits_),
            association_dist_threshold_
        );
    }

    // Define QoS profile (can be customized)
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10)); // Default QoS: keep last 10, reliable.
    // For sensor data, rclcpp::SensorDataQoS() might be more appropriate if available/needed.

    // Subscribers
    det_sub_ = this->create_subscription<mas_msgs::msg::TriangulatedPointArray>(
        "input_detections/triangulated_points", // Topic for structured triangulated points
        qos_profile,
        std::bind(&Sort3DNode::detCallback, this, std::placeholders::_1)
    );

    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("chosen_target_pose", qos_profile);
    target_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("target_region", qos_profile);
    chosen_ray_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("chosen_target_ray_w", qos_profile);

    auto_pick_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        "set_auto_pick_mode",
        qos_profile,
        std::bind(&Sort3DNode::autoPickCallback, this, std::placeholders::_1)
    );

    set_target_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "set_target_position",
        qos_profile,
        std::bind(&Sort3DNode::setTargetPositionCallback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom",
        qos_profile,
        std::bind(&Sort3DNode::odomCallback, this, std::placeholders::_1)
    );

    // Per-camera ray subscriptions
    int num_cameras = this->get_parameter("num_cameras").as_int();
    self_camera_index_ = this->get_parameter("self_camera_index").as_int();
    per_camera_rays_.resize(num_cameras);
    rays_subs_.reserve(num_cameras);
    for (int i = 0; i < num_cameras; ++i) {
        std::string topic = "target_rays_w_" + std::to_string(i + 1);  // remapped by launch file
        rays_subs_.push_back(
            this->create_subscription<mas_msgs::msg::TargetRayArray>(
                topic, qos_profile,
                [this, i](const mas_msgs::msg::TargetRayArray::ConstSharedPtr msg) {
                    this->raysCallback(msg, i);
                }
            )
        );
    }
    RCLCPP_INFO(this->get_logger(), "  Num cameras: %d, self camera index: %d", num_cameras, self_camera_index_);

    RCLCPP_INFO(this->get_logger(), "Sort3DNode initialized successfully.");
}

void Sort3DNode::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg){
    if (!is_odom_received_){
        is_odom_received_ = true;
        RCLCPP_INFO(this->get_logger(), "First odometry message received.");
    }
    // Convert ROS 2 Odometry message to Eigen using the placeholder/actual mav_msgs function
    if (!mav_msgs_ros2::eigenOdometryFromMsg(*msg, &current_odometry_)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to convert odometry message to Eigen type.");
    }
    // current_odometry_ can now be used if needed by the tracking logic (e.g., for motion compensation)
}

void Sort3DNode::detCallback(const mas_msgs::msg::TriangulatedPointArray::ConstSharedPtr msg){
    RCLCPP_DEBUG(this->get_logger(), "Detection callback triggered with %zu triangulated points.", msg->points.size());

    // Cache for detection_id lookup in ray matching
    latest_tri_points_ = std::make_shared<mas_msgs::msg::TriangulatedPointArray>(*msg);

    // Convert triangulated points to Detection3DArray
    vision_msgs::msg::Detection3DArray current_detections_msg = triPoints2dets(*msg);
    last_detection_frame_id_ = current_detections_msg.header.frame_id; // Store frame_id

    // Clear input detection buffers for each class
    for (int i = 0; i < num_object_classes_; ++i){
        per_class_detections_input_.at(i).clear();
    }

    // Distribute detections to their respective class buffers
    for (const auto& det : current_detections_msg.detections){
        if (det.results.empty() || det.results.at(0).hypothesis.class_id.empty()) {
            RCLCPP_WARN(this->get_logger(), "Skipping detection with no results or empty class_id.");
            continue;
        }

        int class_idx = 0; // Default to class 0
        try {
            // Attempt to parse class_id string as an integer if your detector provides numeric IDs
            // class_idx = std::stoi(det.results.at(0).hypothesis.class_id);
            // For this example, we'll assume all detections belong to class 0 if num_object_classes_ is 1.
            // If you have multiple classes, you'll need a robust way to map class_id string to an index.
            if (num_object_classes_ > 1) {
                 // Implement your class_id string to integer index mapping here
                 // For example, if class_id is "car", map to 0; "person" to 1, etc.
                 // This is highly dependent on your detector's output.
                 RCLCPP_WARN_ONCE(this->get_logger(), "Multi-class parsing not fully implemented. Defaulting to class 0 for some detections.");
            }

        } catch (const std::invalid_argument& ia) {
            RCLCPP_WARN(this->get_logger(), "Invalid class_id format '%s': %s. Assigning to class 0.",
                        det.results.at(0).hypothesis.class_id.c_str(), ia.what());
            class_idx = 0; // Fallback
        } catch (const std::out_of_range& oor) {
             RCLCPP_WARN(this->get_logger(), "Class_id '%s' out of range for parsing: %s. Assigning to class 0.",
                        det.results.at(0).hypothesis.class_id.c_str(), oor.what());
            class_idx = 0; // Fallback
        }


        if (class_idx >= 0 && class_idx < num_object_classes_){
            per_class_detections_input_.at(class_idx).push_back(det);
        } else {
            RCLCPP_WARN(this->get_logger(), "Detection with class_id %d (parsed from '%s') is out of bounds (0 to %d). Skipping.",
                        class_idx, det.results.at(0).hypothesis.class_id.c_str(), num_object_classes_ - 1);
        }
    }

    // Update SORT for each class and publish results
    for (int i = 0; i < num_object_classes_; ++i){
        per_class_tracks_output_.at(i) = sort_instances_.at(i).update(per_class_detections_input_.at(i));

        vision_msgs::msg::Detection3DArray tracks_to_publish_msg;
        tracks_to_publish_msg.header.stamp = this->now(); // Use current node time
        tracks_to_publish_msg.header.frame_id = last_detection_frame_id_;

        for(const auto& tracked_det : per_class_tracks_output_.at(i)){
            // Ensure each detection in the published array also has its header set
            vision_msgs::msg::Detection3D pub_det = tracked_det;
            pub_det.header = tracks_to_publish_msg.header;
            tracks_to_publish_msg.detections.push_back(pub_det);
        }
        track_pubs_.at(i)->publish(tracks_to_publish_msg);
        RCLCPP_DEBUG(this->get_logger(), "Published %zu tracks for class %d.", tracks_to_publish_msg.detections.size(), i);
    }

    // Logging (throttled)
    RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000, // Log every 1000 ms (1 second)
                         "Auto pick mode: %d. Current target ID: %d.", auto_pick_mode_, target_id_);
    if (num_object_classes_ > 0 && !per_class_tracks_output_.empty()) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 1000,
                             "Active tracks (class 0): %zu. Total potential trackers (kf_count): %d.",
                             per_class_tracks_output_.at(0).size(), Tracker3D::kf_count_);
    }

    pubChosenTarget(); // Publish the chosen target's pose
}

vision_msgs::msg::Detection3DArray Sort3DNode::triPoints2dets(const mas_msgs::msg::TriangulatedPointArray& tri_points){
    vision_msgs::msg::Detection3DArray det_array;
    det_array.header = tri_points.header;

    for(size_t i = 0; i < tri_points.points.size(); ++i){
        const auto& pt = tri_points.points[i];

        // Basic validation
        if (std::isnan(pt.position.x) || std::isnan(pt.position.y) || std::isnan(pt.position.z))
        {
            continue;
        }
        // Check covariance diagonal for valid, positive values
        double var_x = pt.covariance[0], var_y = pt.covariance[4], var_z = pt.covariance[8];
        if (var_x <= 0.0 || var_y <= 0.0 || var_z <= 0.0)
        {
            continue;
        }

        RCLCPP_INFO(this->get_logger(), "Processing triangulated point %zu at (%.2f, %.2f, %.2f).",
                     i, pt.position.x, pt.position.y, pt.position.z);

        vision_msgs::msg::Detection3D det;
        det.header = tri_points.header;
        det.bbox.center.position = pt.position;
        det.bbox.center.orientation.w = 1.0;
        // Encode 3-sigma uncertainty in bbox.size (preserves existing tracker behavior)
        det.bbox.size.x = 3.0 * std::sqrt(var_x);
        det.bbox.size.y = 3.0 * std::sqrt(var_y);
        det.bbox.size.z = 3.0 * std::sqrt(var_z);

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = "0";
        hyp.hypothesis.score = 1.0;
        hyp.pose.pose.position = pt.position;
        hyp.pose.pose.orientation.w = 1.0;
        // Store covariance in hypothesis pose (6x6, only use position diagonal)
        hyp.pose.covariance[0]  = var_x;
        hyp.pose.covariance[7]  = var_y;
        hyp.pose.covariance[14] = var_z;

        det.results.push_back(hyp);
        det_array.detections.push_back(det);
    }
    return det_array;
}

void Sort3DNode::pubChosenTarget(){
    if (num_object_classes_ == 0 || per_class_tracks_output_.empty()) {
        RCLCPP_DEBUG(this->get_logger(), "No tracks available to choose a target from.");
        return;
    }
    // This logic primarily considers class 0 for target picking. Adapt if multi-class targeting is needed.
    const auto& tracks_class0 = per_class_tracks_output_.at(0);

    vision_msgs::msg::Detection3D picked_target_detection;
    bool target_is_found = false;

    // Check if current target_id_ is still valid and alive
    if (target_id_ != 0) {
        if (!sort_instances_.at(0).isTrackAlive(target_id_)) {
            RCLCPP_INFO(this->get_logger(), "Previously chosen target ID %d is no longer alive. Resetting.", target_id_);
            target_id_ = 0; // Reset if lost
        } else {
            // Try to find the current target_id_ in the output tracks
            for (const auto& track : tracks_class0) {
                if (!track.results.empty()) {
                    try {
                        if (std::stoi(track.results[0].hypothesis.class_id) == target_id_) {
                            picked_target_detection = track;
                            target_is_found = true;
                            break;
                        }
                    } catch (const std::exception& e) { /* ignore parse error here */ }
                }
            }
        }
    }

    // If no valid target_id_ or current target not found, and auto-pick is enabled
    if (!target_is_found && auto_pick_mode_ == 1) {
        if (!tracks_class0.empty()) {
            int new_target_id = pickTarget(); // pickTarget operates on tracks_class0
            if (new_target_id != 0) {
                RCLCPP_INFO(this->get_logger(), "Auto-picking new target. ID: %d", new_target_id);
                target_id_ = new_target_id;
                // Find the newly picked target in the tracks list
                for (const auto& track : tracks_class0) {
                     if (!track.results.empty()) {
                        try {
                            if (std::stoi(track.results[0].hypothesis.class_id) == target_id_) {
                                picked_target_detection = track;
                                target_is_found = true;
                                break;
                            }
                        } catch (const std::exception& e) { /* ignore */ }
                    }
                }
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 2000, "Auto-pick: pickTarget() did not select a new target.");
            }
        } else {
             RCLCPP_INFO_THROTTLE(this->get_logger(), *clock_, 2000, "Auto-pick: No tracks available in class 0 to pick from.");
        }
    }

    // Publish if a target was found
    if (target_is_found) {
        geometry_msgs::msg::PoseWithCovarianceStamped target_pose_msg;
        target_pose_msg.header.stamp = this->now();
        target_pose_msg.header.frame_id = last_detection_frame_id_;
        target_pose_msg.pose.pose.position = picked_target_detection.bbox.center.position;
        target_pose_msg.pose.pose.orientation = picked_target_detection.bbox.center.orientation;
        // Populate covariance from detection results if available
        if (!picked_target_detection.results.empty()) {
            const auto& hyp_cov = picked_target_detection.results[0].pose.covariance;
            target_pose_msg.pose.covariance[0]  = hyp_cov[0];   // var_x
            target_pose_msg.pose.covariance[7]  = hyp_cov[7];   // var_y
            target_pose_msg.pose.covariance[14] = hyp_cov[14];  // var_z
        }
        target_pub_->publish(target_pose_msg);

        geometry_msgs::msg::PointStamped target_point_msg;
        target_point_msg.header = target_pose_msg.header;
        target_point_msg.point = picked_target_detection.bbox.center.position;
        target_point_pub_->publish(target_point_msg);

        // Select and publish the bearing ray for this drone's camera.
        // Strategy: find the TriangulatedPoint closest to the target, use its
        // detection_ids to look up the exact ray from the ego camera.
        // Fallback: angular proximity matching across ego camera rays.
        int ego_cam_idx = self_camera_index_ - 1;  // 0-indexed
        bool ray_published = false;

        // --- ID-based matching ---
        if (!ray_published && latest_tri_points_ &&
            ego_cam_idx >= 0 && ego_cam_idx < static_cast<int>(per_camera_rays_.size()) &&
            per_camera_rays_[ego_cam_idx])
        {
            // Find the TriangulatedPoint nearest to the Kalman-filtered target position
            const auto& target_pos = picked_target_detection.bbox.center.position;
            double best_dist_sq = std::numeric_limits<double>::max();
            const mas_msgs::msg::TriangulatedPoint* best_tri = nullptr;
            for (const auto& pt : latest_tri_points_->points) {
                double dx = pt.position.x - target_pos.x;
                double dy = pt.position.y - target_pos.y;
                double dz = pt.position.z - target_pos.z;
                double d2 = dx*dx + dy*dy + dz*dz;
                if (d2 < best_dist_sq) {
                    best_dist_sq = d2;
                    best_tri = &pt;
                }
            }

            // Match detection_ids against rays from the ego camera
            if (best_tri && best_dist_sq < association_dist_threshold_ * association_dist_threshold_) {
                const auto& ego_rays = per_camera_rays_[ego_cam_idx];
                for (const auto& det_id : best_tri->detection_ids) {
                    for (const auto& ray : ego_rays->rays) {
                        if (ray.detection_id == det_id) {
                            geometry_msgs::msg::Vector3Stamped ray_msg;
                            ray_msg.header = target_pose_msg.header;
                            ray_msg.vector = ray.direction;
                            chosen_ray_pub_->publish(ray_msg);
                            ray_published = true;
                            RCLCPP_DEBUG(this->get_logger(),
                                "Ray selected by detection_id '%s' for target %d",
                                det_id.c_str(), target_id_);
                            break;
                        }
                    }
                    if (ray_published) break;
                }
            }
        }

        // --- Angular fallback (ego camera) ---
        if (!ray_published &&
            ego_cam_idx >= 0 && ego_cam_idx < static_cast<int>(per_camera_rays_.size()) &&
            per_camera_rays_[ego_cam_idx] && !per_camera_rays_[ego_cam_idx]->rays.empty())
        {
            const auto& ego_rays = per_camera_rays_[ego_cam_idx];
            auto& origin = ego_rays->origin;
            double dx = picked_target_detection.bbox.center.position.x - origin.x;
            double dy = picked_target_detection.bbox.center.position.y - origin.y;
            double dz = picked_target_detection.bbox.center.position.z - origin.z;
            double dist = std::sqrt(dx*dx + dy*dy + dz*dz);
            if (dist > 1e-3) {
                double ex = dx / dist, ey = dy / dist, ez = dz / dist;
                double best_dot = -1.0;
                const mas_msgs::msg::TargetRay* best_ray = nullptr;
                for (const auto& ray : ego_rays->rays) {
                    double dot = ray.direction.x * ex + ray.direction.y * ey + ray.direction.z * ez;
                    if (dot > best_dot) {
                        best_dot = dot;
                        best_ray = &ray;
                    }
                }
                if (best_dot > 0.5 && best_ray != nullptr) {
                    geometry_msgs::msg::Vector3Stamped ray_msg;
                    ray_msg.header = target_pose_msg.header;
                    ray_msg.vector = best_ray->direction;
                    chosen_ray_pub_->publish(ray_msg);
                    ray_published = true;
                    RCLCPP_DEBUG(this->get_logger(),
                        "Ray selected by angular fallback for target %d", target_id_);
                }
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Published chosen target ID %d.", target_id_);
    } else if (target_id_ != 0) { // If a target_id was set but not found in current tracks
        RCLCPP_WARN_THROTTLE(this->get_logger(), *clock_, 2000, "Target ID %d was set but not found in current valid tracks.", target_id_);
    }
}

void Sort3DNode::autoPickCallback(const std_msgs::msg::Int8::ConstSharedPtr msg){
    if (auto_pick_mode_ != msg->data){
        RCLCPP_INFO(this->get_logger(), "Setting auto pick mode to: %d", msg->data);
        auto_pick_mode_ = msg->data;
        if (auto_pick_mode_ == 0) { // If auto-pick is turned off
            target_id_ = 0; // Optionally reset the current target
            RCLCPP_INFO(this->get_logger(), "Auto-pick turned off. Current target ID reset.");
        }
    }
}

void Sort3DNode::setTargetPositionCallback(const geometry_msgs::msg::PointStamped::ConstSharedPtr msg){
    // Find nearest active track to the given position
    const auto& p = msg->point;
    double best_dist_sq = std::numeric_limits<double>::max();
    int best_id = 0;

    if (num_object_classes_ > 0 && !per_class_tracks_output_.empty()) {
        for (const auto& track : per_class_tracks_output_.at(0)) {
            if (track.results.empty()) continue;
            double dx = track.bbox.center.position.x - p.x;
            double dy = track.bbox.center.position.y - p.y;
            double dz = track.bbox.center.position.z - p.z;
            double dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq < best_dist_sq) {
                best_dist_sq = dist_sq;
                try {
                    best_id = std::stoi(track.results[0].hypothesis.class_id);
                } catch (const std::exception&) {}
            }
        }
    }

    if (best_id > 0 && best_dist_sq < 25.0) {  // within 5m
        target_id_ = best_id;
        auto_pick_mode_ = 0;
        RCLCPP_INFO(this->get_logger(),
            "Manual target selection by position: nearest track ID=%d (dist=%.2fm). Auto-pick disabled.",
            target_id_, std::sqrt(best_dist_sq));
    } else {
        RCLCPP_WARN(this->get_logger(),
            "No track found near position (%.1f, %.1f, %.1f).", p.x, p.y, p.z);
    }
}

void Sort3DNode::raysCallback(const mas_msgs::msg::TargetRayArray::ConstSharedPtr msg, int camera_index){
    if (camera_index >= 0 && camera_index < static_cast<int>(per_camera_rays_.size())) {
        per_camera_rays_[camera_index] = std::make_shared<mas_msgs::msg::TargetRayArray>(*msg);
    }
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received target rays for camera %d (%zu rays).",
                      camera_index + 1, msg->rays.size());
}

// pickTarget selects the track with the smallest ID from class 0 tracks.
int Sort3DNode::pickTarget(){
    if (num_object_classes_ == 0 || per_class_tracks_output_.empty() || per_class_tracks_output_.at(0).empty()) {
        return 0; // No tracks to pick from
    }

    int smallest_id_found = Tracker3D::kf_count_ + 1; // Initialize with a value larger than any possible ID
    bool a_target_was_considered = false;

    for (const auto& track : per_class_tracks_output_.at(0)){ // Consider only class 0
         if (!track.results.empty()){
            try {
                int current_track_id = std::stoi(track.results[0].hypothesis.class_id);
                if (current_track_id < smallest_id_found){
                    smallest_id_found = current_track_id;
                    a_target_was_considered = true;
                }
            } catch (const std::invalid_argument& ia) {
                RCLCPP_WARN(this->get_logger(), "Invalid track ID format '%s' in pickTarget: %s",
                            track.results[0].hypothesis.class_id.c_str(), ia.what());
            } catch (const std::out_of_range& oor) {
                RCLCPP_WARN(this->get_logger(), "Track ID '%s' out of range for parsing in pickTarget: %s",
                            track.results[0].hypothesis.class_id.c_str(), oor.what());
            }
        }
    }
    return a_target_was_considered ? smallest_id_found : 0;
}

} // namespace mas_tracker

// Main function for the ROS 2 node
int main(int argc, char** argv){
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    // Add node options here if needed, e.g. for composable nodes:
    // options.use_intra_process_comms(true);
    auto node = std::make_shared<mas_tracker::Sort3DNode>(options);

    RCLCPP_INFO(node->get_logger(), "Starting Sort3DNode spin.");
    rclcpp::spin(node); // Process callbacks until shutdown

    RCLCPP_INFO(node->get_logger(), "Sort3DNode shutting down.");
    rclcpp::shutdown();
    return 0;
}
