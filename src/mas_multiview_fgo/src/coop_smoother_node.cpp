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
#include "coop_smoother_fl.h"
#include "ego_camera.h"
#include "meas_noise.h"
#include "output_gate.h"
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
#include <std_msgs/msg/float64_multi_array.hpp>

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
        declare_parameter<double>("pixel_sigma_px", 120.0);   // RAL 024 S4 characterized
        // Peer (transmitted bearing rays).
        declare_parameter<std::vector<std::string>>("peer_ray_topics", std::vector<std::string>{});
        // Output + smoother.
        declare_parameter<std::string>("coop_prefix", "coop_loc");
        declare_parameter<std::string>("frame_id", "common_frame");
        declare_parameter<double>("publish_rate", 50.0);
        declare_parameter<double>("bearing_sigma_deg", 0.5);
        declare_parameter<double>("sigma_psi_deg", 0.0);   // Q10 Tier-1 azimuthal inflation
        // RAL 024 S4 Q9 fallback: characterized peer attitude/origin sigmas (constants) used
        // until the transmitted EKF2 covariance lands on TargetRay (mas_msgs follow-on).
        // 0 = term off. Real-deployment values come from bench/EKF2 characterization.
        declare_parameter<double>("peer_att_sigma_deg", 0.0);
        declare_parameter<double>("peer_pos_sigma_m", 0.0);
        // RAL 024 S5 — association (single-target selection from the detection array),
        // ego robust kernel, declared velocity-cov inflation, and the output-safety gate.
        declare_parameter<std::string>("target_class", "");   // "" = any class
        declare_parameter<double>("min_det_score", 0.0);
        declare_parameter<bool>("use_robust_ego", false);
        declare_parameter<double>("vel_cov_inflation", 20.0);  // RAL 024 S4/S5 declared
        declare_parameter<bool>("gate_enabled", true);
        // RAL 024 S6: warm-start the solve from the last GATE-ACCEPTED belief (CV-propagated),
        // age-limited; acquisition/reacquisition falls back to the midpoint seed.
        declare_parameter<bool>("use_warm_start", true);
        declare_parameter<double>("warm_max_age_s", 2.0);
        // RAL 024 S7: estimator backend — "batch" (full-window LM re-solve per tick) or
        // "fixedlag" (persistent iSAM2 IncrementalFixedLagSmoother; joint-Marginals
        // covariance path — verified batch-equivalent calibration at ~12x less compute).
        declare_parameter<std::string>("backend", "batch");
        declare_parameter<double>("fl_reset_period_s", 0.0);  // finite-memory guard (0 = off)
        declare_parameter<int>("gate_min_peer", 1);
        declare_parameter<double>("gate_max_pos_cov_tr", 100.0);
        declare_parameter<double>("gate_v_max", 30.0);
        declare_parameter<double>("gate_max_jump_m", 5.0);
        declare_parameter<double>("gate_hold_s", 1.0);
        declare_parameter<double>("window_s", 1.2);           // RAL 024 S4
        declare_parameter<double>("q_c", 4.0);
        declare_parameter<bool>("use_robust", false);

        frame_id_ = get_parameter("frame_id").as_string();
        gimbal_order_ = get_parameter("gimbal_angle_order").as_string();
        pixel_sigma_ = get_parameter("pixel_sigma_px").as_double();
        sigma_static_ = get_parameter("bearing_sigma_deg").as_double() * M_PI / 180.0;
        sigma_psi_ = get_parameter("sigma_psi_deg").as_double() * M_PI / 180.0;
        peer_att_rad_ = get_parameter("peer_att_sigma_deg").as_double() * M_PI / 180.0;
        peer_pos_m_ = get_parameter("peer_pos_sigma_m").as_double();
        target_class_ = get_parameter("target_class").as_string();
        min_det_score_ = get_parameter("min_det_score").as_double();
        prm_.use_robust_ego = get_parameter("use_robust_ego").as_bool();
        prm_.vel_cov_inflation = get_parameter("vel_cov_inflation").as_double();
        gate_enabled_ = get_parameter("gate_enabled").as_bool();
        use_warm_start_ = get_parameter("use_warm_start").as_bool();
        warm_max_age_s_ = get_parameter("warm_max_age_s").as_double();
        gp_.min_peer = static_cast<int>(get_parameter("gate_min_peer").as_int());
        gp_.max_pos_cov_tr = get_parameter("gate_max_pos_cov_tr").as_double();
        gp_.v_max = get_parameter("gate_v_max").as_double();
        gp_.max_jump_m = get_parameter("gate_max_jump_m").as_double();
        gp_.hold_s = get_parameter("gate_hold_s").as_double();
        gp_.q_c = get_parameter("q_c").as_double();
        prm_.window_s = get_parameter("window_s").as_double();
        prm_.q_c = get_parameter("q_c").as_double();
        prm_.use_robust = get_parameter("use_robust").as_bool();
        // Backend construction must come AFTER prm_ is fully populated.
        if (get_parameter("backend").as_string() == "fixedlag") {
            fl_ = std::make_unique<mas_fgo::CoopSmootherFL>(
                prm_, get_parameter("fl_reset_period_s").as_double());
        }

        const std::string prefix = get_parameter("coop_prefix").as_string();
        pub_pose_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            prefix + "/target_pose", be_qos());
        pub_twist_ = create_publisher<geometry_msgs::msg::TwistStamped>(
            prefix + "/target_twist", be_qos());
        // Ticket 024 S3 track B: per-tick solver diagnostics (rev1 §11.1), published
        // every tick incl. failed solves so the capture records the cold-start /
        // local-minimum / no-gate behaviour rev1 §§4-5 flags. Layout documented in
        // publishDiag().
        pub_diag_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            prefix + "/solver_diagnostics", be_qos());

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
        // RAL 024 S5 association: select ONE detection — class + score filter, prefer the
        // last selected track id, else highest score (was: every detection became a factor).
        const vision_msgs::msg::Detection2D* best = nullptr;
        double best_score = -1.0;
        bool best_is_track = false;
        for (const auto& det : m->detections) {
            std::string cls; double score = 0.0;
            for (const auto& res : det.results) {
                if (res.hypothesis.score >= score) {
                    score = res.hypothesis.score; cls = res.hypothesis.class_id;
                }
            }
            if (!target_class_.empty() && cls != target_class_) continue;
            if (score < min_det_score_) continue;
            const bool is_track = !last_track_id_.empty() && det.id == last_track_id_;
            if ((is_track && !best_is_track) ||
                (is_track == best_is_track && score > best_score)) {
                best = &det; best_score = score; best_is_track = is_track;
            }
        }
        if (!best) return;
        last_track_id_ = best->id;
        Meas mm; mm.t = t_det; mm.type = EGO_PIXEL;
        mm.px = Eigen::Vector2d(best->bbox.center.position.x, best->bbox.center.position.y);
        mm.K = cam.K; mm.R = cam.R; mm.cam_t = cam.t;
        if (fl_) fl_->addEgoPixel(mm.t, mm.px, mm.K, mm.R, mm.cam_t, pixel_sigma_);
        else buf_.push_back(mm);
    }

    Eigen::Matrix2d peerCov(const Eigen::Vector3d& o, const Eigen::Vector3d& d) const {
        mas_fgo::PeerNoiseParams np;
        np.sigma_static_rad = sigma_static_;
        np.sigma_psi_rad = sigma_psi_;
        np.include_attitude = peer_att_rad_ > 0.0;
        np.include_origin = peer_pos_m_ > 0.0;
        const Eigen::Matrix3d sig_att =
            Eigen::Matrix3d::Identity() * (peer_att_rad_ * peer_att_rad_);
        const Eigen::Matrix3d sig_pos =
            Eigen::Matrix3d::Identity() * (peer_pos_m_ * peer_pos_m_);
        const Eigen::Vector3d seed =
            (std::abs(d.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
        const Eigen::Vector3d u1 = (seed - seed.dot(d) * d).normalized();
        const Eigen::Vector3d u2 = d.cross(u1);
        return mas_fgo::buildPeerBearingCov(np, u1, u2, o, d, o + 30.0 * d, sig_att, sig_pos);
    }

    void onPeerRays(const mas_msgs::msg::TargetRayArray::SharedPtr m) {
        const double t = stamp_s(m->header.stamp);
        const Eigen::Vector3d origin(m->origin.x, m->origin.y, m->origin.z);
        for (const auto& r : m->rays) {
            Eigen::Vector3d d(r.direction.x, r.direction.y, r.direction.z);
            if (d.norm() < 1e-9) continue;
            if (fl_) {
                const Eigen::Vector3d dn = d.normalized();
                fl_->addPeerBearing(t, origin, dn, peerCov(origin, dn));
                continue;
            }
            Meas mm; mm.t = t; mm.type = PEER_BEARING; mm.o = origin; mm.d = d.normalized();
            buf_.push_back(mm);
        }
    }

    void onTimer() {
        const double now = this->now().seconds();
        if (fl_) {  // S7 fixed-lag fast path: persistent update, shared gate + publish tail
            const bool solved = fl_->update(now);
            publishDiag(fl_->diagnostics(), solved, now);
            const auto q = solved ? fl_->query(now) : mas_fgo::CoopSmoother::Query{};
            Eigen::Vector3d bel_p = q.p, bel_v = q.v;
            Eigen::Matrix<double, 6, 6> bel_cov = q.cov;
            if (gate_enabled_) {
                const Eigen::Vector3d ego =
                    pose_buf_.empty() ? Eigen::Vector3d::Zero() : pose_buf_.back().p;
                const mas_fgo::GateOutput out = mas_fgo::applyOutputGate(
                    fl_->diagnostics(), solved, q, now, ego, !pose_buf_.empty(), gp_, gst_);
                if (!out.publish) return;
                bel_p = out.p; bel_v = out.v; bel_cov = out.cov;
            } else if (!solved || !q.valid) {
                return;
            }
            publishBelief(bel_p, bel_v, bel_cov);
            return;
        }
        const double t_cut = now - prm_.window_s - 0.2;
        while (!buf_.empty() && buf_.front().t < t_cut) buf_.pop_front();
        if (buf_.size() < 2) {
            // S5: total measurement dropout — still serve the held fallback within gate_hold_s
            // (diag cadence unchanged: no solve, no diag, matching the pre-S5 capture format).
            if (gate_enabled_) {
                const Eigen::Vector3d ego =
                    pose_buf_.empty() ? Eigen::Vector3d::Zero() : pose_buf_.back().p;
                const mas_fgo::GateOutput out = mas_fgo::applyOutputGate(
                    mas_fgo::CoopSmoother::Diagnostics{}, false,
                    mas_fgo::CoopSmoother::Query{}, now, ego, !pose_buf_.empty(), gp_, gst_);
                if (out.publish) publishBelief(out.p, out.v, out.cov);
            }
            return;
        }

        mas_fgo::CoopSmoother sm(prm_);
        if (use_warm_start_ && gst_.have_last && now - gst_.t_last >= 0.0 &&
            now - gst_.t_last <= warm_max_age_s_) {
            sm.setInitHint(gst_.t_last, gst_.p_last, gst_.v_last);
        }
        mas_fgo::PeerNoiseParams np;
        np.sigma_static_rad = sigma_static_;
        np.sigma_psi_rad = sigma_psi_;
        // RAL 024 S4: Q9 attitude/origin terms via characterized constants (params above);
        // transmitted EKF2 covariance on TargetRay remains the mas_msgs follow-on.
        np.include_attitude = peer_att_rad_ > 0.0;
        np.include_origin = peer_pos_m_ > 0.0;
        const Eigen::Matrix3d sig_att =
            Eigen::Matrix3d::Identity() * (peer_att_rad_ * peer_att_rad_);
        const Eigen::Matrix3d sig_pos =
            Eigen::Matrix3d::Identity() * (peer_pos_m_ * peer_pos_m_);

        for (const auto& mm : buf_) {
            if (mm.type == EGO_PIXEL) {
                sm.addEgoPixel(mm.t, mm.px, mm.K, mm.R, mm.cam_t, pixel_sigma_);
            } else {
                const Eigen::Vector3d seed =
                    (std::abs(mm.d.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
                const Eigen::Vector3d u1 = (seed - seed.dot(mm.d) * mm.d).normalized();
                const Eigen::Vector3d u2 = mm.d.cross(u1);
                const Eigen::Matrix2d R = mas_fgo::buildPeerBearingCov(
                    np, u1, u2, mm.o, mm.d, mm.o + 30.0 * mm.d, sig_att, sig_pos);
                sm.addPeerBearing(mm.t, mm.o, mm.d, R);
            }
        }
        const bool solved = sm.solve();
        publishDiag(sm.diagnostics(), solved, now);
        const auto q = solved ? sm.query(now) : mas_fgo::CoopSmoother::Query{};

        // RAL 024 S5 output-safety gate (rev1 §5): vet the solve; on failure hold the last
        // accepted belief (CV-predicted, cov grown) for gate_hold_s, then go silent.
        Eigen::Vector3d bel_p = q.p, bel_v = q.v;
        Eigen::Matrix<double, 6, 6> bel_cov = q.cov;
        if (gate_enabled_) {
            const Eigen::Vector3d ego =
                pose_buf_.empty() ? Eigen::Vector3d::Zero() : pose_buf_.back().p;
            const mas_fgo::GateOutput out = mas_fgo::applyOutputGate(
                sm.diagnostics(), solved, q, now, ego, !pose_buf_.empty(), gp_, gst_);
            if (!out.publish) return;
            bel_p = out.p; bel_v = out.v; bel_cov = out.cov;
        } else if (!solved || !q.valid) {
            return;
        }
        publishBelief(bel_p, bel_v, bel_cov);
    }

    void publishBelief(const Eigen::Vector3d& p, const Eigen::Vector3d& v,
                       const Eigen::Matrix<double, 6, 6>& cov) {
        const rclcpp::Time stamp = this->now();
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.stamp = stamp; pose.header.frame_id = frame_id_;
        pose.pose.pose.position.x = p.x();
        pose.pose.pose.position.y = p.y();
        pose.pose.pose.position.z = p.z();
        pose.pose.pose.orientation.w = 1.0;
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                pose.pose.covariance[r * 6 + c] = cov(r, c);
        for (int i = 3; i < 6; ++i) pose.pose.covariance[i * 6 + i] = 1e3;
        pub_pose_->publish(pose);

        geometry_msgs::msg::TwistStamped tw;
        tw.header.stamp = stamp; tw.header.frame_id = frame_id_;
        tw.twist.linear.x = v.x();
        tw.twist.linear.y = v.y();
        tw.twist.linear.z = v.z();
        pub_twist_->publish(tw);
    }

    // Float64MultiArray layout (index): 0 solved 1 iterations 2 max_iterations
    // 3 n_keyframes 4 n_ego 5 n_peer 6 error_before 7 error_after 8 max_factor_error
    // 9 t_oldest 10 t_newest 11 window_span 12-14 seed_xyz 15 buf_size.
    void publishDiag(const mas_fgo::CoopSmoother::Diagnostics& d, bool solved, double /*now*/) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {solved ? 1.0 : 0.0,
                    static_cast<double>(d.iterations), static_cast<double>(d.max_iterations),
                    static_cast<double>(d.n_keyframes), static_cast<double>(d.n_ego),
                    static_cast<double>(d.n_peer), d.error_before, d.error_after,
                    d.max_factor_error, d.t_oldest, d.t_newest, d.t_newest - d.t_oldest,
                    d.seed.x(), d.seed.y(), d.seed.z(), static_cast<double>(buf_.size())};
        pub_diag_->publish(msg);
    }

    std::string frame_id_, gimbal_order_;
    double pixel_sigma_ = 120.0, sigma_static_ = 0.0, sigma_psi_ = 0.0, zoom_ = 1.0;
    double peer_att_rad_ = 0.0, peer_pos_m_ = 0.0;  // S4 Q9 characterized fallbacks
    std::string target_class_, last_track_id_;      // S5 association
    double min_det_score_ = 0.0;
    bool gate_enabled_ = true;                      // S5 output gate
    bool use_warm_start_ = true;                    // S6 warm-start
    double warm_max_age_s_ = 2.0;
    mas_fgo::GateParams gp_;
    mas_fgo::GateState gst_;
    std::unique_ptr<mas_fgo::CoopSmootherFL> fl_;   // S7 backend (null = batch)
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
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_diag_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CoopSmootherNode>());
    rclcpp::shutdown();
    return 0;
}
