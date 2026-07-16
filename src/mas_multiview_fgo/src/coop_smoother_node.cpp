/**
 * @file coop_smoother_node.cpp
 * @brief Ticket 024 S1 — ROS2 node wrapping the cooperative GTSAM smoother (choice A, mixed).
 *
 * DEPLOYMENT ROLE = INTERCEPTOR. Runs the smoother ONLY (no `mas_multiview` for its own ego):
 *   - EGO (local): subscribes its own raw detection + camera_info + camera_pose + gimbal + zoom,
 *     assembles the ego camera model (ego_camera.h) with the pose interpolated to the detection
 *     time t_det (pose_interp.h, Q8 localized here), and adds an EgoPixelFactor (Q7 — keep pixels
 *     locally).
 *   - PEER (remote): subscribes each peer's transmitted `target_rays_w` (a bearing, formed +
 *     pose-synced at the OBSERVER's source), and adds a PeerBearingFactor.
 *
 * Each measurement is placed at its own capture stamp + a CV motion factor (the 019 velocity blocker
 * + the ticket 020 async v*tau bias). Publishes the pn cooperative estimate-source contract (drop-in
 * for the 019 cv_smoother, Q4=a — pn untouched):
 *     {coop_prefix}/target_pose   geometry_msgs/PoseWithCovarianceStamped
 *     {coop_prefix}/target_twist  geometry_msgs/TwistStamped
 */
#include "coop_smoother.h"
#include "ego_camera.h"
#include "meas_noise.h"
#include "pose_interp.h"

#include <rclcpp/rclcpp.hpp>
#include <mas_msgs/msg/target_ray_array.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>

#include <deque>
#include <memory>
#include <string>
#include <vector>

namespace {
rclcpp::QoS be_qos() { return rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(); }
double stamp_s(const builtin_interfaces::msg::Time& t) { return t.sec + t.nanosec * 1e-9; }
}  // namespace

class CoopSmootherNode : public rclcpp::Node {
public:
    CoopSmootherNode() : Node("coop_smoother") {
        // Ego (local raw inputs -> pixel factor). Empty topic disables the ego arm.
        declare_parameter<std::string>("ego_detection_topic", "");
        declare_parameter<std::string>("ego_camera_info_topic", "");
        declare_parameter<std::string>("ego_odom_topic", "");   // vehicle world pose (common_frame/odom)
        declare_parameter<std::string>("ego_gimbal_topic", "");
        declare_parameter<std::string>("ego_zoom_topic", "");
        declare_parameter<std::string>("gimbal_angle_order", "zyx");
        declare_parameter<double>("pixel_sigma_px", 2.0);
        // Peer (transmitted bearing rays).
        declare_parameter<std::vector<std::string>>("peer_ray_topics", std::vector<std::string>{});
        // Output + smoother.
        declare_parameter<std::string>("coop_prefix", "coop_loc");
        declare_parameter<std::string>("frame_id", "common_frame");
        declare_parameter<double>("publish_rate", 50.0);
        declare_parameter<double>("bearing_sigma_deg", 0.5);
        declare_parameter<double>("sigma_psi_deg", 0.0);   // Q10 Tier-1 azimuthal inflation
        declare_parameter<double>("window_s", 0.6);
        declare_parameter<double>("q_c", 4.0);
        declare_parameter<bool>("use_robust", false);

        frame_id_ = get_parameter("frame_id").as_string();
        gimbal_order_ = get_parameter("gimbal_angle_order").as_string();
        pixel_sigma_ = get_parameter("pixel_sigma_px").as_double();
        sigma_static_ = get_parameter("bearing_sigma_deg").as_double() * M_PI / 180.0;
        sigma_psi_ = get_parameter("sigma_psi_deg").as_double() * M_PI / 180.0;
        prm_.window_s = get_parameter("window_s").as_double();
        prm_.q_c = get_parameter("q_c").as_double();
        prm_.use_robust = get_parameter("use_robust").as_bool();

        const std::string prefix = get_parameter("coop_prefix").as_string();
        pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            prefix + "/target_pose", be_qos());
        pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            prefix + "/target_twist", be_qos());

        // --- ego (local raw) subscriptions ---
        const auto det_t = get_parameter("ego_detection_topic").as_string();
        if (!det_t.empty()) {
            sub_det_ = create_subscription<vision_msgs::msg::Detection2DArray>(
                det_t, be_qos(),
                [this](const vision_msgs::msg::Detection2DArray::SharedPtr m) { onDetection(m); });
            sub_info_ = create_subscription<sensor_msgs::msg::CameraInfo>(
                get_parameter("ego_camera_info_topic").as_string(), be_qos(),
                [this](const sensor_msgs::msg::CameraInfo::SharedPtr m) { onCameraInfo(m); });
            sub_odom_ = create_subscription<nav_msgs::msg::Odometry>(
                get_parameter("ego_odom_topic").as_string(), be_qos(),
                [this](const nav_msgs::msg::Odometry::SharedPtr m) { onEgoOdom(m); });
            sub_gimbal_ = create_subscription<geometry_msgs::msg::Vector3>(
                get_parameter("ego_gimbal_topic").as_string(), be_qos(),
                [this](const geometry_msgs::msg::Vector3::SharedPtr m) { onGimbal(m); });
            const auto zoom_t = get_parameter("ego_zoom_topic").as_string();
            if (!zoom_t.empty())
                sub_zoom_ = create_subscription<std_msgs::msg::Float64>(
                    zoom_t, be_qos(),   // sim publishes zoom BEST_EFFORT; must match or K is unscaled
                    [this](const std_msgs::msg::Float64::SharedPtr m) { zoom_ = m->data; });
            RCLCPP_INFO(get_logger(), "coop_smoother: EGO pixel input <- %s (+ camera_info/pose/gimbal)",
                        det_t.c_str());
        }

        // --- peer (transmitted ray) subscriptions ---
        // NB: copy the array into a named var — as_string_array() returns a reference into the
        // temporary Parameter, which would dangle if iterated inline.
        const std::vector<std::string> peer_topics = get_parameter("peer_ray_topics").as_string_array();
        for (const auto& t : peer_topics) {
            peer_subs_.push_back(create_subscription<mas_msgs::msg::TargetRayArray>(
                t, be_qos(),
                [this](const mas_msgs::msg::TargetRayArray::SharedPtr m) { onPeerRays(m); }));
            RCLCPP_INFO(get_logger(), "coop_smoother: PEER bearing input <- %s", t.c_str());
        }

        const double rate = std::max(1.0, get_parameter("publish_rate").as_double());
        timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / rate),
                                   [this]() { onTimer(); });
        RCLCPP_INFO(get_logger(),
            "coop_smoother up (choice A): ego pixel %s + %zu peer bearing(s) -> %s/target_pose+twist "
            "(pixel_sigma=%.1f px, bearing_sigma=%.2f deg, window=%.2f s, q_c=%.1f)",
            sub_det_ ? "ON" : "off", peer_subs_.size(), prefix.c_str(),
            pixel_sigma_, sigma_static_ * 180.0 / M_PI, prm_.window_s, prm_.q_c);
    }

private:
    enum MType { EGO_PIXEL, PEER_BEARING };
    struct Meas {
        double t; MType type;
        Eigen::Vector2d px; Eigen::Matrix3d K, R; Eigen::Vector3d cam_t;   // ego pixel
        Eigen::Vector3d o, d;                                              // peer bearing
    };

    void onCameraInfo(const sensor_msgs::msg::CameraInfo::SharedPtr m) {
        K_raw_ = Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(m->k.data());
        have_K_ = true;
    }
    void onGimbal(const geometry_msgs::msg::Vector3::SharedPtr m) {
        gimbal_rad_ = Eigen::Vector3d(m->x, m->y, m->z) * M_PI / 180.0;  // degrees -> radians
        have_gimbal_ = true;
    }
    void onEgoOdom(const nav_msgs::msg::Odometry::SharedPtr m) {
        // Vehicle world pose (common_frame/odom); the gimbal is applied in assembleEgoCamera.
        // Matches triangulation_node::cameraOdomCallback (camera_pose derived from odom).
        mas_fgo::PoseSample s;
        s.t = stamp_s(m->header.stamp);
        const auto& p = m->pose.pose;
        s.p = Eigen::Vector3d(p.position.x, p.position.y, p.position.z);
        s.q = Eigen::Quaterniond(p.orientation.w, p.orientation.x,
                                 p.orientation.y, p.orientation.z).normalized();
        pose_buf_.push_back(s);
        while (pose_buf_.size() > 200) pose_buf_.pop_front();
    }

    void onDetection(const vision_msgs::msg::Detection2DArray::SharedPtr m) {
        if (!have_K_ || !have_gimbal_ || pose_buf_.empty()) return;
        const double t_det = stamp_s(m->header.stamp);                   // Q8: capture time
        const mas_fgo::InterpPose ip = mas_fgo::interpolatePose(pose_buf_, t_det);
        if (!ip.valid) return;
        const mas_fgo::EgoCamera cam =
            mas_fgo::assembleEgoCamera(ip.p, ip.q, gimbal_rad_, zoom_, K_raw_, gimbal_order_);
        for (const auto& det : m->detections) {
            Meas mm; mm.t = t_det; mm.type = EGO_PIXEL;
            mm.px = Eigen::Vector2d(det.bbox.center.position.x, det.bbox.center.position.y);
            mm.K = cam.K; mm.R = cam.R; mm.cam_t = cam.t;
            buf_.push_back(mm);
        }
    }

    void onPeerRays(const mas_msgs::msg::TargetRayArray::SharedPtr m) {
        const double t = stamp_s(m->header.stamp);
        const Eigen::Vector3d origin(m->origin.x, m->origin.y, m->origin.z);
        for (const auto& r : m->rays) {
            Eigen::Vector3d d(r.direction.x, r.direction.y, r.direction.z);
            if (d.norm() < 1e-9) continue;
            Meas mm; mm.t = t; mm.type = PEER_BEARING; mm.o = origin; mm.d = d.normalized();
            buf_.push_back(mm);
        }
    }

    void onTimer() {
        const double now = this->now().seconds();
        const double t_cut = now - prm_.window_s - 0.2;
        while (!buf_.empty() && buf_.front().t < t_cut) buf_.pop_front();
        if (buf_.size() < 2) return;

        mas_fgo::CoopSmoother sm(prm_);
        mas_fgo::PeerNoiseParams np;
        np.sigma_static_rad = sigma_static_;
        np.sigma_psi_rad = sigma_psi_;
        np.include_attitude = false;   // Q9 transmitted pose-cov = mas_msgs follow-on
        np.include_origin = false;

        for (const auto& mm : buf_) {
            if (mm.type == EGO_PIXEL) {
                sm.addEgoPixel(mm.t, mm.px, mm.K, mm.R, mm.cam_t, pixel_sigma_);
            } else {
                const Eigen::Vector3d seed =
                    (std::abs(mm.d.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
                const Eigen::Vector3d u1 = (seed - seed.dot(mm.d) * mm.d).normalized();
                const Eigen::Vector3d u2 = mm.d.cross(u1);
                const Eigen::Matrix2d R =
                    mas_fgo::buildPeerBearingCov(np, u1, u2, mm.o, mm.d, mm.o + 30.0 * mm.d);
                sm.addPeerBearing(mm.t, mm.o, mm.d, R);
            }
        }
        if (!sm.solve()) return;
        const auto q = sm.query(now);
        if (!q.valid) return;

        const rclcpp::Time stamp = this->now();
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.stamp = stamp; pose.header.frame_id = frame_id_;
        pose.pose.pose.position.x = q.p.x();
        pose.pose.pose.position.y = q.p.y();
        pose.pose.pose.position.z = q.p.z();
        pose.pose.pose.orientation.w = 1.0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                pose.pose.covariance[r * 6 + c] = q.cov(r, c);
        for (int i = 3; i < 6; ++i) pose.pose.covariance[i * 6 + i] = 1e3;
        pub_pose_->publish(pose);

        geometry_msgs::msg::TwistStamped tw;
        tw.header.stamp = stamp; tw.header.frame_id = frame_id_;
        tw.twist.linear.x = q.v.x();
        tw.twist.linear.y = q.v.y();
        tw.twist.linear.z = q.v.z();
        pub_twist_->publish(tw);
    }

    std::string frame_id_, gimbal_order_;
    double pixel_sigma_ = 2.0, sigma_static_ = 0.0, sigma_psi_ = 0.0, zoom_ = 1.0;
    mas_fgo::CoopSmoother::Params prm_;

    Eigen::Matrix3d K_raw_ = Eigen::Matrix3d::Identity();
    Eigen::Vector3d gimbal_rad_ = Eigen::Vector3d::Zero();
    bool have_K_ = false, have_gimbal_ = false;
    std::deque<mas_fgo::PoseSample> pose_buf_;
    std::deque<Meas> buf_;

    rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_det_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr sub_info_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr sub_gimbal_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub_zoom_;
    std::vector<rclcpp::Subscription<mas_msgs::msg::TargetRayArray>::SharedPtr> peer_subs_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pub_pose_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr pub_twist_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoopSmootherNode>());
    rclcpp::shutdown();
    return 0;
}
