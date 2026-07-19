# RAL ticket 024 AC3 (amended 2026-07-20) — transmitted-ray array carrying the observer's
# online-estimated (EKF2) pose covariance: the Q9 estimated-vs-characterized hybrid transmits
# ONLY what is estimated online; gimbal/calibration/pixel uncertainty stays a fusion-side
# characterized parameter (R_static).
#
# The covariances ride once per ARRAY, not per ray — they are properties of the transmitting
# vehicle's pose at the capture instant, shared by every ray formed from it (~18 float64 per
# message; ~450 B/s at the 3 Hz ray rate). Naming follows the geometry_msgs *WithCovariance
# convention. NOTE: the live pipeline SWITCHOVER to this type is deferred until real-world
# bandwidth tests (user decision 2026-07-20); consumers fold the covariances via
# mas_multiview_fgo meas_noise.h buildPeerBearingCov (R_att + range-projected origin term);
# an all-zero covariance means "not provided" and selects the characterized fallback.
std_msgs/Header header            # stamp = DETECTION capture time t_det (not publication)
geometry_msgs/Point origin        # camera position in world frame at t_det
float64[9] origin_covariance      # 3x3 row-major position covariance [m^2]; all-zero = none
float64[9] attitude_covariance    # 3x3 row-major attitude covariance [rad^2]; all-zero = none
TargetRay[] rays
