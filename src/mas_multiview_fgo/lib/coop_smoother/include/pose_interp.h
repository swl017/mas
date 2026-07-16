/**
 * @file pose_interp.h
 * @brief Q8 (ticket 024): interpolate a vehicle's OWN pose to a detection time `t_det` before the
 *        ray/pixel factor is formed. Self-motion is smooth/predictable, so interpolating it is safe;
 *        the target motion is NOT interpolated at the source (that is the smoother's job). SLERP for
 *        attitude, linear for position; bracketed between two samples (extrapolation guarded/flagged).
 */
#ifndef MAS_MULTIVIEW_FGO_POSE_INTERP_H
#define MAS_MULTIVIEW_FGO_POSE_INTERP_H

#include <deque>
#include <Eigen/Eigen>

namespace mas_fgo {

struct PoseSample {
    double t;
    Eigen::Vector3d p;
    Eigen::Quaterniond q;  // world<-body (normalized)
};

struct InterpPose {
    Eigen::Vector3d p;
    Eigen::Quaterniond q;
    bool extrapolated = false;  // true if `t` fell outside the buffered sample span
    bool valid = false;         // false if the buffer was empty
};

/**
 * @brief Interpolate the buffered pose stream to time `t`.
 * @param buf  time-ascending pose samples (self odometry).
 * @param t    query time (the detection time t_det).
 * Bracketed SLERP+lerp when `t` is within [buf.front().t, buf.back().t]; otherwise clamps to the
 * nearest end and sets `extrapolated = true` (caller should guard on it).
 */
inline InterpPose interpolatePose(const std::deque<PoseSample>& buf, double t)
{
    InterpPose out;
    if (buf.empty()) return out;
    out.valid = true;

    if (t <= buf.front().t) { out.p = buf.front().p; out.q = buf.front().q; out.extrapolated = (t < buf.front().t); return out; }
    if (t >= buf.back().t)  { out.p = buf.back().p;  out.q = buf.back().q;  out.extrapolated = (t > buf.back().t);  return out; }

    // Find the bracketing pair [i, i+1] with buf[i].t <= t < buf[i+1].t.
    size_t i = 0;
    while (i + 1 < buf.size() && buf[i + 1].t <= t) ++i;
    const PoseSample& a = buf[i];
    const PoseSample& b = buf[i + 1];
    const double denom = (b.t - a.t);
    const double s = (denom > 1e-9) ? (t - a.t) / denom : 0.0;

    out.p = (1.0 - s) * a.p + s * b.p;
    out.q = a.q.slerp(s, b.q).normalized();
    return out;
}

}  // namespace mas_fgo

#endif  // MAS_MULTIVIEW_FGO_POSE_INTERP_H
