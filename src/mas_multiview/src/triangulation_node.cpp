#include <cstdio>
#include <set>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <mas_msgs/msg/triangulated_point_array.hpp>
#include <mas_msgs/msg/target_ray_array.hpp>
#include <vector>
#include "multiview_triangulation.h"
#include "covariance_propagation.h"

class TriangulationNode : public rclcpp::Node
{
public:
    TriangulationNode();
    ~TriangulationNode();

private:
    void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg, int camera_index);
    void cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, int camera_index);
    void cameraZoomCallback(const std_msgs::msg::Float64::SharedPtr msg, int camera_index);
    void cameraPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int camera_index);
    void cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int camera_index);
    void gimbalCallback(const geometry_msgs::msg::Vector3::SharedPtr msg, int camera_index);
    void precomputedRaysCallback(const mas_msgs::msg::TargetRayArray::SharedPtr msg, int camera_index);
    void timerCallback();

    rclcpp::Publisher<mas_msgs::msg::TriangulatedPointArray>::SharedPtr results_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr visualization_pub_;
    std::vector<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr> results_per_cam_pubs_;
    std::vector<rclcpp::Publisher<mas_msgs::msg::TargetRayArray>::SharedPtr> target_rays_pubs_;
    std::vector<rclcpp::Subscription<mas_msgs::msg::TargetRayArray>::SharedPtr> precomputed_rays_subs_;
    std::vector<std::shared_ptr<mas_msgs::msg::TargetRayArray>> precomputed_rays_;
    std::vector<bool> use_precomputed_rays_;
    std::vector<rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr> detection_subs_;
    std::vector<rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr> camera_info_subs_;
    std::vector<rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr> camera_zoom_subs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr> camera_pose_subs_;
    std::vector<rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr> camera_odom_subs_;
    std::vector<rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr> gimbal_subs_;
    std::vector<std::shared_ptr<vision_msgs::msg::Detection2DArray>> detections_;
    std::vector<std::shared_ptr<sensor_msgs::msg::CameraInfo>> camera_infos_;
    std::vector<std::shared_ptr<std_msgs::msg::Float64>> camera_zooms_;
    std::vector<std::shared_ptr<geometry_msgs::msg::PoseStamped>> camera_poses_;
    std::vector<std::shared_ptr<nav_msgs::msg::Odometry>> camera_odoms_;
    std::vector<std::shared_ptr<geometry_msgs::msg::Vector3>> gimbals_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<Eigen::Vector3d> camera_world_positions_;

    int num_camera_;

    enum ProcessState
    {
        INITIALIZING,
        WAITING,
        PROCESSING,
        FINISHED
    } process_state_;

    std::shared_ptr<MultiView::MultiViewTriangulation> multiview_triangulation_;
    std::string gimbal_angle_order_;
    visualization_msgs::msg::Marker blue_line_marker_;
    visualization_msgs::msg::Marker red_line_marker_;
    visualization_msgs::msg::Marker green_line_marker_;
    visualization_msgs::msg::Marker grey_line_marker_;
    std::string frame_id_;
};

TriangulationNode::TriangulationNode() : Node("triangulation_node")
{
    process_state_ = ProcessState::INITIALIZING;

    // Declare parameters
    this->declare_parameter("frame_id", "common_frame");
    this->declare_parameter("publish_rate", 10.0);
    this->declare_parameter("num_camera", 3);
    this->declare_parameter("camera_name_prefix", "/px4_");
    this->declare_parameter("detection_topic_suffix", "yolo_result_vision");
    this->declare_parameter("camera_info_topic_suffix", "camera/color/camera_info");
    this->declare_parameter("camera_zoom_topic_suffix", "camera/zoom_level");
    this->declare_parameter("camera_pose_topic_suffix", "camera_pose");
    this->declare_parameter("camera_odom_topic_suffix", "common_frame/odom");
    this->declare_parameter("gimbal_topic_suffix", "gimbal_state_rpy_deg");
    this->declare_parameter("gimbal_angle_order", "zyx");
    // Per-camera flag: if true, subscribe to precomputed target_rays_w instead of raw topics
    this->declare_parameter("use_precomputed_rays", std::vector<bool>{});
    this->declare_parameter("max_solve_time", 0.1);
    this->declare_parameter("max_reprojection_error", 100.0);
    // Covariance propagation parameters
    this->declare_parameter("cov.pix_std", 7.0);
    this->declare_parameter("cov.pos_std", 0.1);
    this->declare_parameter("cov.ori_std", 0.02);
    this->declare_parameter("cov.gimbal_std", 0.01);
    this->declare_parameter("cov.use_pose_covariance", true);
    this->declare_parameter("cov.include_position_uncertainty", true);
    this->declare_parameter("cov.include_orientation_uncertainty", true);
    this->declare_parameter("cov.include_gimbal_uncertainty", true);

    // Ticket 020: per-ray angular uncertainty [deg] for a transmitted (precomputed) bearing ray.
    // Q2 = fusion-side sigma_theta parameter (no mas_msgs change); default matches the
    // cooperative mock's detector-grade sigma_deg.
    this->declare_parameter("bearing_sigma_deg", 0.5);

    // Create subscribers based on number of views
    int num_camera = this->get_parameter("num_camera").as_int();
    num_camera_ = num_camera;
    detection_subs_.reserve(num_camera);
    detections_.resize(num_camera);
    camera_info_subs_.reserve(num_camera);
    camera_infos_.resize(num_camera);
    camera_zoom_subs_.reserve(num_camera);
    camera_zooms_.resize(num_camera);
    camera_pose_subs_.reserve(num_camera);
    camera_poses_.resize(num_camera);
    camera_odom_subs_.reserve(num_camera);
    camera_odoms_.resize(num_camera);
    gimbal_subs_.reserve(num_camera);
    gimbals_.resize(num_camera);
    camera_world_positions_.resize(num_camera);
    results_per_cam_pubs_.reserve(num_camera);
    results_per_cam_pubs_.resize(num_camera);
    precomputed_rays_.resize(num_camera);
    precomputed_rays_subs_.reserve(num_camera);

    // Resolve per-camera precomputed rays flag
    auto precomputed_param = this->get_parameter("use_precomputed_rays").as_bool_array();
    use_precomputed_rays_.resize(num_camera, false);
    for (size_t i = 0; i < precomputed_param.size() && i < static_cast<size_t>(num_camera); ++i)
    {
        use_precomputed_rays_[i] = precomputed_param[i];
    }

    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

    std::string camera_name_prefix = this->get_parameter("camera_name_prefix").as_string();
    RCLCPP_INFO(this->get_logger(), "camera_name_prefix: %s", camera_name_prefix.c_str());

    std::string detection_topic_suffix = this->get_parameter("detection_topic_suffix").as_string();
    std::string camera_info_topic_suffix = this->get_parameter("camera_info_topic_suffix").as_string();
    std::string camera_zoom_topic_suffix = this->get_parameter("camera_zoom_topic_suffix").as_string();
    std::string camera_pose_topic_suffix = this->get_parameter("camera_pose_topic_suffix").as_string();
    std::string camera_odom_topic_suffix = this->get_parameter("camera_odom_topic_suffix").as_string();
    std::string gimbal_topic_suffix = this->get_parameter("gimbal_topic_suffix").as_string();

    for (int i = 0; i < num_camera; ++i)
    {
        std::string prefix = camera_name_prefix + std::to_string(i + 1) + "/";

        if (use_precomputed_rays_[i])
        {
            // Precomputed rays mode: subscribe only to target_rays_w
            std::string rays_topic = prefix + "target_rays_w";
            RCLCPP_INFO(this->get_logger(), "Camera %d: precomputed rays from %s", i + 1, rays_topic.c_str());
            auto sub = this->create_subscription<mas_msgs::msg::TargetRayArray>(
                rays_topic, 10, [this, i](const mas_msgs::msg::TargetRayArray::SharedPtr msg)
                { precomputedRaysCallback(msg, i); });
            precomputed_rays_subs_.push_back(sub);
            // Initialize default zoom for precomputed cameras
            camera_zooms_[i] = std::make_shared<std_msgs::msg::Float64>();
            camera_zooms_[i]->data = 1.0;
        }
        else
        {
            // Raw mode: subscribe to detections + camera_info + zoom + pose + odom + gimbal
            RCLCPP_INFO(this->get_logger(), "Camera %d: raw topics from %s", i + 1, prefix.c_str());

            auto det_sub = this->create_subscription<vision_msgs::msg::Detection2DArray>(
                prefix + detection_topic_suffix, qos_profile,
                [this, i](const vision_msgs::msg::Detection2DArray::SharedPtr msg)
                { detectionCallback(msg, i); });
            detection_subs_.push_back(det_sub);

            auto info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
                prefix + camera_info_topic_suffix, 10,
                [this, i](const sensor_msgs::msg::CameraInfo::SharedPtr msg)
                { cameraInfoCallback(msg, i); });
            camera_info_subs_.push_back(info_sub);

            auto zoom_sub = this->create_subscription<std_msgs::msg::Float64>(
                prefix + camera_zoom_topic_suffix, qos_profile,
                [this, i](const std_msgs::msg::Float64::SharedPtr msg)
                { cameraZoomCallback(msg, i); });
            camera_zoom_subs_.push_back(zoom_sub);
            camera_zooms_[i] = std::make_shared<std_msgs::msg::Float64>();
            camera_zooms_[i]->data = 1.0;

            auto pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
                prefix + camera_pose_topic_suffix, 10,
                [this, i](const geometry_msgs::msg::PoseStamped::SharedPtr msg)
                { cameraPoseCallback(msg, i); });
            camera_pose_subs_.push_back(pose_sub);

            auto odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
                prefix + camera_odom_topic_suffix, qos_profile,
                [this, i](const nav_msgs::msg::Odometry::SharedPtr msg)
                { cameraOdomCallback(msg, i); });
            camera_odom_subs_.push_back(odom_sub);

            auto gimbal_sub = this->create_subscription<geometry_msgs::msg::Vector3>(
                prefix + gimbal_topic_suffix, qos_profile,
                [this, i](const geometry_msgs::msg::Vector3::SharedPtr msg)
                { gimbalCallback(msg, i); });
            gimbal_subs_.push_back(gimbal_sub);
        }
    }

    // Create publishers
    results_pub_ = this->create_publisher<mas_msgs::msg::TriangulatedPointArray>("triangulated_points", 10);
    visualization_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("triangulated_points/visualization", 10);
    target_rays_pubs_.resize(num_camera);
    for (int i = 0; i < num_camera; ++i)
    {
        std::string topic = "cam" + std::to_string(i + 1) + "/triangulated_points_viz";
        auto pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 10);
        results_per_cam_pubs_[i] = pub;

        std::string rays_topic = camera_name_prefix + std::to_string(i + 1) + "/target_rays_w";
        target_rays_pubs_[i] = this->create_publisher<mas_msgs::msg::TargetRayArray>(rays_topic, 10);
    }

    // Create timer
    double rate = this->get_parameter("publish_rate").as_double();
    timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / rate)),
                                        std::bind(&TriangulationNode::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Triangulation node initialized with %d cameras", num_camera);

    multiview_triangulation_ = std::make_shared<MultiView::MultiViewTriangulation>();
    for (int i = 0; i < num_camera; ++i)
    {
        multiview_triangulation_->addCamera(i);
    }

    gimbal_angle_order_ = this->get_parameter("gimbal_angle_order").as_string();
    if (gimbal_angle_order_ != "zyx" && gimbal_angle_order_ != "zxy" && gimbal_angle_order_ != "zy")
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid gimbal angle order: %s. Defaulting to zyx.", gimbal_angle_order_.c_str());
        gimbal_angle_order_ = "zyx";
    }
    RCLCPP_INFO(this->get_logger(), "gimbal_angle_order: %s", gimbal_angle_order_.c_str());

    // Misc
    frame_id_ = this->get_parameter("frame_id").as_string();
    RCLCPP_INFO(this->get_logger(), "frame_id: %s", frame_id_.c_str());
    blue_line_marker_.header.frame_id = frame_id_;
    blue_line_marker_.header.stamp = this->now();
    blue_line_marker_.ns = "camera_target_connections";
    blue_line_marker_.id = 0; // Single marker for all lines
    blue_line_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    blue_line_marker_.action = visualization_msgs::msg::Marker::ADD;
    blue_line_marker_.pose.orientation.w = 1.0; // Identity orientation
    blue_line_marker_.scale.x = 0.05; // Line width
    blue_line_marker_.color.a = 0.8; // Slightly transparent
    blue_line_marker_.color.r = 0.0;
    blue_line_marker_.color.g = 0.0;
    blue_line_marker_.color.b = 1.0; // Blue lines

    green_line_marker_.header.frame_id = frame_id_;
    green_line_marker_.header.stamp = this->now();
    green_line_marker_.ns = "camera_ray_raw";
    green_line_marker_.id = 0; // Single marker for all lines
    green_line_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    green_line_marker_.action = visualization_msgs::msg::Marker::ADD;
    green_line_marker_.pose.orientation.w = 1.0; // Identity orientation
    green_line_marker_.scale.x = 0.05; // Line width
    green_line_marker_.color.a = 0.8; // Slightly transparent
    green_line_marker_.color.r = 0.0;
    green_line_marker_.color.g = 1.0;
    green_line_marker_.color.b = 0.0;

    grey_line_marker_.header.frame_id = frame_id_;
    grey_line_marker_.header.stamp = this->now();
    grey_line_marker_.ns = "camera_ray_frustum";
    grey_line_marker_.id = 0; // Single marker for all lines
    grey_line_marker_.type = visualization_msgs::msg::Marker::LINE_LIST;
    grey_line_marker_.action = visualization_msgs::msg::Marker::ADD;
    grey_line_marker_.pose.orientation.w = 1.0; // Identity orientation
    grey_line_marker_.scale.x = 0.05; // Line width
    grey_line_marker_.color.a = 0.8; // Slightly transparent
    grey_line_marker_.color.r = 0.5;
    grey_line_marker_.color.g = 0.5;
    grey_line_marker_.color.b = 0.5;
}

TriangulationNode::~TriangulationNode()
{
}

void TriangulationNode::detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg, int camera_index)
{
    detections_[camera_index] = std::make_shared<vision_msgs::msg::Detection2DArray>(*msg);
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received detection from camera %d", camera_index + 1);
}

void TriangulationNode::cameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg, int camera_index)
{
    sensor_msgs::msg::CameraInfo camera_info = *msg;
    camera_infos_[camera_index] = std::make_shared<sensor_msgs::msg::CameraInfo>(*msg);
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received camera info from camera %d", camera_index + 1);
}

void TriangulationNode::cameraZoomCallback(const std_msgs::msg::Float64::SharedPtr msg, int camera_index)
{
    std_msgs::msg::Float64 camera_zoom = *msg;
    camera_zooms_[camera_index] = std::make_shared<std_msgs::msg::Float64>(*msg);
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received camera zoom from camera %d", camera_index + 1);
}

void TriangulationNode::cameraPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg, int camera_index)
{
    camera_poses_[camera_index] = std::make_shared<geometry_msgs::msg::PoseStamped>(*msg);
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received camera pose from camera %d", camera_index + 1);
}

void TriangulationNode::cameraOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg, int camera_index)
{
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header = msg->header;
    pose_stamped.pose = msg->pose.pose;
    camera_poses_[camera_index] = std::make_shared<geometry_msgs::msg::PoseStamped>(pose_stamped);
    camera_odoms_[camera_index] = std::make_shared<nav_msgs::msg::Odometry>(*msg);
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received camera odometry from camera %d", camera_index + 1);
}

void TriangulationNode::gimbalCallback(const geometry_msgs::msg::Vector3::SharedPtr msg, int camera_index)
{
    gimbals_[camera_index] = std::make_shared<geometry_msgs::msg::Vector3>(*msg);
    gimbals_[camera_index]->x = msg->x * M_PI / 180.0; // Convert degrees to radians
    gimbals_[camera_index]->y = msg->y * M_PI / 180.0; // Convert degrees to radians
    gimbals_[camera_index]->z = msg->z * M_PI / 180.0; // Convert degrees to radians
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received gimbal state from camera %d", camera_index + 1);
}

void TriangulationNode::precomputedRaysCallback(const mas_msgs::msg::TargetRayArray::SharedPtr msg, int camera_index)
{
    precomputed_rays_[camera_index] = std::make_shared<mas_msgs::msg::TargetRayArray>(*msg);
    RCLCPP_DEBUG_ONCE(this->get_logger(), "Received precomputed rays from camera %d (%zu rays)",
                      camera_index + 1, msg->rays.size());
}

void TriangulationNode::timerCallback()
{
    std::vector<int> ready_cameras;      // raw cameras ready for triangulation (have detections)
    std::vector<int> ready_precomputed;   // precomputed cameras ready for triangulation
    std::vector<int> frustum_cameras;     // raw cameras with geometric data (for frustum viz)
    for (size_t i = 0; i < num_camera_; i++)
    {
        if (use_precomputed_rays_[i])
        {
            if (precomputed_rays_[i])
            {
                ready_precomputed.push_back(i);
            }
        }
        else
        {
            if (camera_infos_[i] && camera_poses_[i] && gimbals_[i])
            {
                frustum_cameras.push_back(i);
                if (detections_[i] && !detections_[i]->detections.empty())
                {
                    ready_cameras.push_back(i);
                }
            }
        }
    }
    size_t total_ready = ready_cameras.size() + ready_precomputed.size();

    // Set up extrinsics/intrinsics for all frustum-ready cameras (for frustum visualization)
    for (const int& i : frustum_cameras)
    {
        sensor_msgs::msg::CameraInfo camera_info = *camera_infos_[i];
        std_msgs::msg::Float64 camera_zoom = camera_zooms_[i] ? *camera_zooms_[i] : std_msgs::msg::Float64();
        if (!camera_zooms_[i]) camera_zoom.data = 1.0;
        geometry_msgs::msg::Pose pose = camera_poses_[i]->pose;
        geometry_msgs::msg::Vector3 gimbal = *gimbals_[i];

        Eigen::Vector3d odom_t_world_robot;
        odom_t_world_robot << pose.position.x, pose.position.y, pose.position.z;
        Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
        Eigen::Matrix3d odom_R_world_robot = q.toRotationMatrix();
        Eigen::Quaterniond q_gimbal;
        Eigen::Matrix3d gimbal_R_robot_gimbal;
        Eigen::Matrix3d combined_R_world_gimbal;
        if (gimbal_angle_order_ == "zyx")
        {
            q_gimbal = Eigen::AngleAxisd(gimbal.z, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(gimbal.y, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(gimbal.x, Eigen::Vector3d::UnitX());
            gimbal_R_robot_gimbal = q_gimbal.toRotationMatrix();
            combined_R_world_gimbal = odom_R_world_robot * gimbal_R_robot_gimbal;
        }
        else if (gimbal_angle_order_ == "zxy")
        {
            q_gimbal = Eigen::AngleAxisd(gimbal.z, Eigen::Vector3d::UnitZ())
                        * Eigen::AngleAxisd(gimbal.x, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(gimbal.y, Eigen::Vector3d::UnitY());
            gimbal_R_robot_gimbal = q_gimbal.toRotationMatrix();
            combined_R_world_gimbal = odom_R_world_robot * gimbal_R_robot_gimbal;
        }
        else if (gimbal_angle_order_ == "zy")
        {
            auto robot_euler = odom_R_world_robot.eulerAngles(2,1,0);
            q_gimbal = Eigen::AngleAxisd(gimbal.y, Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(gimbal.z + robot_euler(0), Eigen::Vector3d::UnitZ());
            gimbal_R_robot_gimbal = q_gimbal.toRotationMatrix();
            combined_R_world_gimbal = gimbal_R_robot_gimbal;
        }

        Eigen::Vector3d gimbal_mount_t_robot_gimbal = Eigen::Vector3d(0.1, 0.0, -0.1);
        Eigen::Vector3d camera_t_world_camera = odom_t_world_robot + odom_R_world_robot * gimbal_mount_t_robot_gimbal;
        multiview_triangulation_->setCameraExtrinsics(i, combined_R_world_gimbal, camera_t_world_camera);

        Eigen::Matrix3d k = Eigen::Map<const Eigen::Matrix3d>(camera_info.k.data()).transpose();
        Eigen::Matrix3d scaled_k;
        scaled_k << k(0,0) * camera_zoom.data, k(0,1) * camera_zoom.data, k(0,2),
                    k(1,0) * camera_zoom.data, k(1,1) * camera_zoom.data, k(1,2),
                    k(2,0), k(2,1), k(2,2);
        multiview_triangulation_->setCameraIntrinsics(i, scaled_k);
        multiview_triangulation_->setCameraWidthHeight(i, camera_info.width, camera_info.height);

        camera_world_positions_[i] = camera_t_world_camera;
    }

    // Set up extrinsics + fusion-side bearing sigma for precomputed (transmitted-ray) cameras.
    // The extrinsics origin (cam.t_) doubles as the peer ray origin used by the point-to-ray
    // residual and the bearing covariance (ticket 020).
    const double bearing_sigma_rad =
        this->get_parameter("bearing_sigma_deg").as_double() * M_PI / 180.0;
    for (const int& i : ready_precomputed)
    {
        auto& rays_msg = precomputed_rays_[i];
        Eigen::Vector3d origin(rays_msg->origin.x, rays_msg->origin.y, rays_msg->origin.z);
        camera_world_positions_[i] = origin;
        multiview_triangulation_->setCameraExtrinsics(i, Eigen::Matrix3d::Identity(), origin);
        multiview_triangulation_->setCameraBearingSigma(i, bearing_sigma_rad);
    }

    // Always build and publish frustum visualization
    {
        visualization_msgs::msg::MarkerArray frustum_marker_array;
        std::vector<visualization_msgs::msg::MarkerArray> per_camera_frustum(num_camera_);
        std::set<int> frustum_set(frustum_cameras.begin(), frustum_cameras.end());
        frustum_set.insert(ready_precomputed.begin(), ready_precomputed.end());
        for (size_t i = 0; i < multiview_triangulation_->getNumCameras(); ++i)
        {
            if (frustum_set.find(i) == frustum_set.end())
                continue;
            // A transmitted-ray (precomputed) camera has no local image model (no K_/width_),
            // so it has no view frustum to draw; getFrustumCorners would read uninitialized
            // intrinsics. Skip it (ticket 020 — avoid any uninitialized K_/width_ read).
            if (multiview_triangulation_->cameras_[i].is_precomputed_)
                continue;
            double zoom = (camera_zooms_[i] ? camera_zooms_[i]->data : 1.0);
            std::vector<Eigen::Vector3d> corners = multiview_triangulation_->cameras_[i].getFrustumCorners(30.0 * zoom);
            for (size_t j = 0; j < corners.size(); ++j)
            {
                Eigen::Vector3d corner = corners[j];
                visualization_msgs::msg::Marker frustum_marker = grey_line_marker_;
                frustum_marker.ns = "camera" + std::to_string(i) + "/frustum";
                frustum_marker.id = j;
                frustum_marker.points.clear();

                geometry_msgs::msg::Point p_camera;
                p_camera.x = multiview_triangulation_->cameras_[i].t_(0);
                p_camera.y = multiview_triangulation_->cameras_[i].t_(1);
                p_camera.z = multiview_triangulation_->cameras_[i].t_(2);
                frustum_marker.points.push_back(p_camera);
                geometry_msgs::msg::Point p;
                p.x = corner(0);
                p.y = corner(1);
                p.z = corner(2);
                if (std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                {
                    continue;
                }
                frustum_marker.points.push_back(p);
                frustum_marker_array.markers.push_back(frustum_marker);
                per_camera_frustum[i].markers.push_back(frustum_marker);

                frustum_marker.id = corners.size() + j;
                frustum_marker.points.clear();
                frustum_marker.points.push_back(p);
                geometry_msgs::msg::Point p_next;
                if (j < corners.size() - 1)
                {
                    p_next.x = corners[j + 1](0);
                    p_next.y = corners[j + 1](1);
                    p_next.z = corners[j + 1](2);
                }
                else
                {
                    p_next.x = corners[0](0);
                    p_next.y = corners[0](1);
                    p_next.z = corners[0](2);
                }
                frustum_marker.points.push_back(p_next);
                frustum_marker_array.markers.push_back(frustum_marker);
                per_camera_frustum[i].markers.push_back(frustum_marker);
            }
        }
        // Publish frustum visualization: DELETEALL + new markers in one message
        visualization_msgs::msg::Marker delete_marker;
        delete_marker.header.frame_id = frame_id_;
        delete_marker.header.stamp = this->now();
        delete_marker.ns = "deleteall";
        delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        frustum_marker_array.markers.insert(frustum_marker_array.markers.begin(), delete_marker);
        visualization_pub_->publish(frustum_marker_array);
        for (size_t i = 0; i < multiview_triangulation_->getNumCameras(); ++i)
        {
            if (!per_camera_frustum[i].markers.empty())
            {
                results_per_cam_pubs_[i]->publish(per_camera_frustum[i]);
            }
        }
    }

    if (total_ready < 2)
    {
        process_state_ = ProcessState::WAITING;
        RCLCPP_INFO(this->get_logger(), "Messages:");
        for (int i = 0; i < num_camera_; ++i)
        {
            RCLCPP_INFO(this->get_logger(), "Camera %d %s", i + 1,
                        use_precomputed_rays_[i] ? "(precomputed)" : "(raw)");
            if (use_precomputed_rays_[i])
            {
                RCLCPP_INFO(this->get_logger(), "- precomputed rays: %s",
                            precomputed_rays_[i] ? "OK" : "-");
            }
            else
            {
                RCLCPP_INFO(this->get_logger(), "- detections: %s", detections_[i] ? "OK" : "-");
                RCLCPP_INFO(this->get_logger(), "- camera info: %s", camera_infos_[i] ? "OK" : "-");
                RCLCPP_INFO(this->get_logger(), "- camera pose: %s", camera_poses_[i] ? "OK" : "-");
                RCLCPP_INFO(this->get_logger(), "- gimbal state: %s", gimbals_[i] ? "OK" : "-");
            }
        }
        // Reset frustum-ready cameras that won't be reset by the main path
        for (const int& i : frustum_cameras)
        {
            multiview_triangulation_->resetCameraDetections(i);
        }
        for (const int& i : ready_precomputed)
        {
            multiview_triangulation_->resetCameraDetections(i);
            precomputed_rays_[i].reset();
        }
        return;
    }

    process_state_ = ProcessState::PROCESSING;
    auto start_time = this->now();
    std::vector<Eigen::Vector3d> current_camera_positions(num_camera_);

    // Add detections for ready raw cameras (extrinsics/intrinsics already set above)
    for (const int& i : ready_cameras)
    {
        vision_msgs::msg::Detection2DArray det_2d_array = *detections_[i];
        geometry_msgs::msg::Vector3 gimbal = *gimbals_[i];

        // Pass gimbal angles for covariance propagation
        multiview_triangulation_->setCameraGimbalAngles(i,
            Eigen::Vector3d(gimbal.x, gimbal.y, gimbal.z));

        // Pass pose covariance from EKF if available
        if (camera_odoms_[i]) {
            Eigen::Matrix<double, 6, 6> pose_cov;
            for (int r = 0; r < 6; ++r) {
                for (int c = 0; c < 6; ++c) {
                    pose_cov(r, c) = camera_odoms_[i]->pose.covariance[r * 6 + c];
                }
            }
            multiview_triangulation_->setCameraPoseCovariance(i, pose_cov);
        }

        for (const auto &det : det_2d_array.detections)
        {
            MultiView::Detection::Detection2D det_2d;
            det_2d.center << det.bbox.center.position.x, det.bbox.center.position.y;
            det_2d.width = det.bbox.size_x;
            det_2d.height = det.bbox.size_y;
            det_2d.class_id = det.id;

            multiview_triangulation_->addDetection(i, det_2d);
        }
    }

    // Add precomputed rays (extrinsics already set above)
    for (const int& i : ready_precomputed)
    {
        auto& rays_msg = precomputed_rays_[i];
        for (const auto& ray : rays_msg->rays)
        {
            Eigen::Vector3d dir(ray.direction.x, ray.direction.y, ray.direction.z);
            Eigen::Vector3d origin(rays_msg->origin.x, rays_msg->origin.y, rays_msg->origin.z);
            multiview_triangulation_->addPrecomputedRay(i, origin, dir, ray.detection_id);
        }
    }

    // Publish per-camera target rays for raw cameras (before triangulation — rays are inputs)
    for (const int& i : ready_cameras)
    {
        mas_msgs::msg::TargetRayArray ray_array_msg;
        ray_array_msg.header.stamp = this->now();
        ray_array_msg.header.frame_id = frame_id_;
        ray_array_msg.origin.x = camera_world_positions_[i].x();
        ray_array_msg.origin.y = camera_world_positions_[i].y();
        ray_array_msg.origin.z = camera_world_positions_[i].z();
        for (const auto& det : multiview_triangulation_->cameras_[i].detections_)
        {
            mas_msgs::msg::TargetRay ray_msg;
            ray_msg.direction.x = det.detection2d.ray.ray_direction.x();
            ray_msg.direction.y = det.detection2d.ray.ray_direction.y();
            ray_msg.direction.z = det.detection2d.ray.ray_direction.z();
            ray_msg.detection_id = det.detection2d.class_id;
            ray_array_msg.rays.push_back(ray_msg);
        }
        if (i < static_cast<int>(target_rays_pubs_.size()))
        {
            target_rays_pubs_[i]->publish(ray_array_msg);
        }
    }

    // Ticket 020 (§5b auditability): the fair peer-only AoI model fuses a FRESH ego ray with a
    // STALE peer ray captured earlier, so log each fresh-ego vs stale-peer capture-stamp pair and
    // the peer lag per fusion tick. This makes the v*tau temporal-inconsistency effect auditable
    // (the authoritative stamps also ride on the recorded target-ray topics). DEBUG level to
    // avoid steady-state spam.
    for (const int& e : ready_cameras)
    {
        if (!detections_[e]) continue;
        double ego_t = rclcpp::Time(detections_[e]->header.stamp).seconds();
        for (const int& p : ready_precomputed)
        {
            if (!precomputed_rays_[p]) continue;
            double peer_t = rclcpp::Time(precomputed_rays_[p]->header.stamp).seconds();
            RCLCPP_DEBUG(this->get_logger(),
                "[ray-stamp] ego cam%d t=%.3f  peer cam%d t=%.3f  peer_lag=%.3f s",
                e, ego_t, p, peer_t, ego_t - peer_t);
        }
    }

    auto setup_time = this->now();

    double max_solve_time = this->get_parameter("max_solve_time").as_double();
    double max_reprojection_error = this->get_parameter("max_reprojection_error").as_double();

    // Configure covariance propagation
    MultiView::CovarianceConfig cov_config;
    cov_config.pix_std = this->get_parameter("cov.pix_std").as_double();
    cov_config.pos_std = this->get_parameter("cov.pos_std").as_double();
    cov_config.ori_std = this->get_parameter("cov.ori_std").as_double();
    cov_config.gimbal_std = this->get_parameter("cov.gimbal_std").as_double();
    cov_config.use_pose_covariance = this->get_parameter("cov.use_pose_covariance").as_bool();
    cov_config.include_position_uncertainty = this->get_parameter("cov.include_position_uncertainty").as_bool();
    cov_config.include_orientation_uncertainty = this->get_parameter("cov.include_orientation_uncertainty").as_bool();
    cov_config.include_gimbal_uncertainty = this->get_parameter("cov.include_gimbal_uncertainty").as_bool();
    multiview_triangulation_->setCovarianceConfig(cov_config);

    multiview_triangulation_->Triangulate(max_solve_time, max_reprojection_error);

    auto triangulation_time = this->now();

    // Publish structured results (TriangulatedPointArray)
    mas_msgs::msg::TriangulatedPointArray tri_points_msg;
    tri_points_msg.header.stamp = this->now();
    tri_points_msg.header.frame_id = frame_id_;

    // Also build visualization MarkerArray for RViz
    visualization_msgs::msg::MarkerArray visualization_marker_array;
    for (size_t i = 0; i < multiview_triangulation_->results_.size(); ++i)
    {
        auto result = multiview_triangulation_->results_[i];

        // Fix B: reject invalid results (zero position, NaN, Inf)
        bool is_zero = (result.position.norm() < 1e-6);
        bool has_nan = std::isnan(result.position.x()) || std::isnan(result.position.y()) || std::isnan(result.position.z());
        bool has_inf = std::isinf(result.position.x()) || std::isinf(result.position.y()) || std::isinf(result.position.z());
        if (is_zero || has_nan || has_inf)
        {
            RCLCPP_DEBUG(this->get_logger(), "Rejecting invalid triangulation result %zu: zero=%d nan=%d inf=%d",
                         i, is_zero, has_nan, has_inf);
            continue;
        }

        // Structured message
        mas_msgs::msg::TriangulatedPoint tri_point;
        tri_point.position.x = result.position.x();
        tri_point.position.y = result.position.y();
        tri_point.position.z = result.position.z();
        // 3x3 covariance row-major
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                tri_point.covariance[r * 3 + c] = result.covariance(r, c);
        // Contributing detection IDs (ByteTrack IDs from each camera)
        tri_point.detection_ids = result.detection_ids;
        tri_points_msg.points.push_back(tri_point);

        // Visualization markers
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id_;
        marker.header.stamp = this->now();
        // position
        marker.ns = "triangulation" + std::to_string(i) + "/position";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = result.position.x();
        marker.pose.position.y = result.position.y();
        marker.pose.position.z = result.position.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;
        marker.color.a = 1.0;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        visualization_marker_array.markers.push_back(marker);
        // covariance
        marker.ns = "triangulation" + std::to_string(i) + "/covariance";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = result.position.x();
        marker.pose.position.y = result.position.y();
        marker.pose.position.z = result.position.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 3.0 * std::sqrt(result.covariance(0, 0));
        marker.scale.y = 3.0 * std::sqrt(result.covariance(1, 1));
        marker.scale.z = 3.0 * std::sqrt(result.covariance(2, 2));
        marker.color.a = 0.5; // Semi-transparent
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        visualization_marker_array.markers.push_back(marker);
    }

    std::vector<visualization_msgs::msg::MarkerArray> per_camera_marker_array(num_camera_);
    // Build set of cameras that had their extrinsics set this cycle
    std::set<int> active_cameras(ready_cameras.begin(), ready_cameras.end());
    active_cameras.insert(ready_precomputed.begin(), ready_precomputed.end());
    for (size_t i = 0; i < multiview_triangulation_->getNumCameras(); ++i)
    {
        // Skip cameras whose extrinsics were not set this cycle (uninitialized memory)
        if (active_cameras.find(i) == active_cameras.end())
        {
            continue;
        }
        // Frustum visualization is handled above (always published)

        for (size_t j = 0; j < multiview_triangulation_->cameras_[i].detections_.size(); ++j)
        {
            auto ray = multiview_triangulation_->cameras_[i].detections_[j].detection2d.ray;
            visualization_msgs::msg::Marker ray_marker = green_line_marker_;
            ray_marker.ns = "camera" + std::to_string(i) + "/detection" + std::to_string(j) + "/ray";
            ray_marker.id = 0;
            ray_marker.points.clear();
            geometry_msgs::msg::Point p_camera;
            p_camera.x = ray.ray_origin.x();
            p_camera.y = ray.ray_origin.y();
            p_camera.z = ray.ray_origin.z();
            ray_marker.points.push_back(p_camera);
            geometry_msgs::msg::Point p_target;
            p_target.x = ray.ray_origin.x() + 1000 * ray.ray_direction.x();
            p_target.y = ray.ray_origin.y() + 1000 * ray.ray_direction.y();
            p_target.z = ray.ray_origin.z() + 1000 * ray.ray_direction.z();
            ray_marker.points.push_back(p_target);
            visualization_marker_array.markers.push_back(ray_marker);
            per_camera_marker_array[i].markers.push_back(ray_marker);
        }
        std::vector<MultiView::Detection::Detection3D> det_3d = multiview_triangulation_->get3DDetectionsFromCamera(i);
        for (size_t j = 0; j < det_3d.size(); ++j)
        {
            // Skip detections with uninitialized/zero 3D positions (no association matched)
            if (det_3d[j].position.norm() < 1e-6)
            {
                continue;
            }
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = frame_id_;
            marker.header.stamp = this->now();
            marker.ns = "camera" + std::to_string(i) + "/detection" + std::to_string(j) + "/position";
            marker.id = 0;
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = det_3d[j].position.x();
            marker.pose.position.y = det_3d[j].position.y();
            marker.pose.position.z = det_3d[j].position.z();
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 0.5;
            marker.scale.y = 0.5;
            marker.scale.z = 0.5;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            visualization_marker_array.markers.push_back(marker);
            per_camera_marker_array[i].markers.push_back(marker);

            visualization_msgs::msg::Marker line_marker = blue_line_marker_;
            line_marker.ns = "camera" + std::to_string(i) + "/detection" + std::to_string(j) + "/line";
            line_marker.id = 0;
            geometry_msgs::msg::Point p_camera;
            p_camera.x = multiview_triangulation_->cameras_[i].t_.x();
            p_camera.y = multiview_triangulation_->cameras_[i].t_.y();
            p_camera.z = multiview_triangulation_->cameras_[i].t_.z();
            line_marker.points.push_back(p_camera);
            geometry_msgs::msg::Point p_target;
            p_target.x = det_3d[j].position.x();
            p_target.y = det_3d[j].position.y();
            p_target.z = det_3d[j].position.z();
            line_marker.points.push_back(p_target);
            visualization_marker_array.markers.push_back(line_marker);
            per_camera_marker_array[i].markers.push_back(line_marker);
        }
    }

    if (multiview_triangulation_->results_.empty())
    {
        rclcpp::Clock clock;
        RCLCPP_WARN(this->get_logger(), "No triangulated points found.");
        for (size_t i = 0; i < multiview_triangulation_->getNumCameras(); ++i)
        {
            multiview_triangulation_->resetCameraDetections(i);
        }
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "Found %ld triangulated points.", multiview_triangulation_->results_.size());
        multiview_triangulation_->printDetections(multiview_triangulation_->cameras_);
    }

    results_pub_->publish(tri_points_msg);
    if (!visualization_marker_array.markers.empty())
    {
        visualization_pub_->publish(visualization_marker_array);
    }
    for (size_t i = 0; i < multiview_triangulation_->getNumCameras(); ++i)
    {
        if (!per_camera_marker_array[i].markers.empty())
        {
            results_per_cam_pubs_[i]->publish(per_camera_marker_array[i]);
        }
    }

    auto end_time = this->now();

    process_state_ = ProcessState::FINISHED;
    for (size_t i = 0; i < multiview_triangulation_->getNumCameras(); ++i)
    {
        multiview_triangulation_->resetCameraDetections(i);
        if (use_precomputed_rays_[i])
        {
            precomputed_rays_[i].reset();
        }
        else
        {
            detections_[i].reset();
            camera_infos_[i].reset();
            camera_poses_[i].reset();
        }
    }

    // rclcpp::Clock clock;
    // RCLCPP_INFO_THROTTLE(this->get_logger(), clock, 0.5 * 1e3,
    RCLCPP_INFO(this->get_logger(),
        "Time consumed: \n- Setup: %.3f ms\n- Triangulation: %.3f ms\n- Visualization time: %.3f ms\n- Total time: %.3f ms\n=================",
        (setup_time - start_time).seconds() * 1000.0, (triangulation_time - setup_time).seconds() * 1000.0,
        (end_time - triangulation_time).seconds() * 1000.0, (end_time - start_time).seconds() * 1000.0
    );
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TriangulationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
