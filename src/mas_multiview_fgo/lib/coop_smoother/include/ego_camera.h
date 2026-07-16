/**
 * @file ego_camera.h
 * @brief Assemble the ego camera model (K, R_world_camera, t_world_camera) from the interceptor's
 *        LOCAL topics (camera_pose x gimbal x zoom x camera_info), so `mas_multiview_fgo` can form
 *        the ego PIXEL factor itself (ticket 024 choice A) — the interceptor runs the smoother ONLY,
 *        no `mas_multiview` for its own ego. Ported verbatim from `mas_multiview/triangulation_node`
 *        (odom_R * gimbal_R[zyx], t = pos + odom_R*mount, fx/fy scaled by zoom). Apply the pose at
 *        the detection time `t_det` (pose_interp.h, Q8) before calling this.
 */
#ifndef MAS_MULTIVIEW_FGO_EGO_CAMERA_H
#define MAS_MULTIVIEW_FGO_EGO_CAMERA_H

#include <Eigen/Eigen>

namespace mas_fgo {

struct EgoCamera {
    Eigen::Matrix3d K;   // intrinsics (zoom-scaled)
    Eigen::Matrix3d R;   // combined world->gimbal rotation (as consumed by EgoPixelFactor)
    Eigen::Vector3d t;   // camera position in world
};

/**
 * @param pos,q       vehicle world pose (from camera_pose), interpolated to t_det.
 * @param gimbal_rad  gimbal (roll=x, pitch=y, yaw=z) in radians.
 * @param zoom        zoom level (fx,fy scaled).
 * @param K_raw       raw intrinsics (from camera_info, already row->col mapped).
 * @param order       "zyx" (default), "zxy", or "zy" — matching triangulation_node.
 */
inline EgoCamera assembleEgoCamera(const Eigen::Vector3d& pos, const Eigen::Quaterniond& q,
                                   const Eigen::Vector3d& gimbal_rad, double zoom,
                                   const Eigen::Matrix3d& K_raw,
                                   const std::string& order = "zyx",
                                   const Eigen::Vector3d& gimbal_mount =
                                       Eigen::Vector3d(0.1, 0.0, -0.1))
{
    EgoCamera c;
    const Eigen::Matrix3d odom_R = q.toRotationMatrix();
    Eigen::Matrix3d gimbal_R;
    if (order == "zxy") {
        gimbal_R = (Eigen::AngleAxisd(gimbal_rad.z(), Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(gimbal_rad.x(), Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(gimbal_rad.y(), Eigen::Vector3d::UnitY())).toRotationMatrix();
        c.R = odom_R * gimbal_R;
    } else if (order == "zy") {
        const Eigen::Vector3d re = odom_R.eulerAngles(2, 1, 0);
        gimbal_R = (Eigen::AngleAxisd(gimbal_rad.y(), Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX())
                  * Eigen::AngleAxisd(gimbal_rad.z() + re(0), Eigen::Vector3d::UnitZ())).toRotationMatrix();
        c.R = gimbal_R;
    } else {  // "zyx" (default)
        gimbal_R = (Eigen::AngleAxisd(gimbal_rad.z(), Eigen::Vector3d::UnitZ())
                  * Eigen::AngleAxisd(gimbal_rad.y(), Eigen::Vector3d::UnitY())
                  * Eigen::AngleAxisd(gimbal_rad.x(), Eigen::Vector3d::UnitX())).toRotationMatrix();
        c.R = odom_R * gimbal_R;
    }
    c.t = pos + odom_R * gimbal_mount;
    c.K = K_raw;
    c.K(0, 0) *= zoom;
    c.K(0, 1) *= zoom;
    c.K(1, 0) *= zoom;
    c.K(1, 1) *= zoom;
    return c;
}

}  // namespace mas_fgo

#endif  // MAS_MULTIVIEW_FGO_EGO_CAMERA_H
