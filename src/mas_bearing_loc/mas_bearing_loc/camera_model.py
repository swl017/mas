"""Gimbal-mounted pinhole projection model.

Frame conventions (matching project CLAUDE.md and the paper):
- Body frame (BF): FLU (X-forward, Y-left, Z-up)
- Earth/world frame (EFF / `common_frame`): ENU (X-east, Y-north, Z-up)
- Camera frame (CF): standard CV convention — X-right, Y-down, Z-forward

The gimbal publishes RPY in *gimbal coordinate* (roll about X-forward,
pitch about Y-left, yaw about Z-up of the SIYI base). With zero gimbal angles,
the camera optical axis (CF +Z) points along body +X (forward). We absorb the
fixed CF↔BF axis swap into `R_c_b_zero` below, then apply the gimbal RPY in BF
as small additional rotation. Override `R_c_b_zero` via node parameters if the
mount differs.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np

from .quaternion import rpy_zxy_to_rot


# Camera-to-body at zero gimbal: rotate CF (X-right, Y-down, Z-forward) to BF
# (X-forward, Y-left, Z-up).  v_body = R_c_b_zero @ v_cam.
#   cam X-right   →  body Y-left = +Y_body * (-1)  ⇒ -Y_body
#   cam Y-down    →  body Z-down = -Z_body
#   cam Z-forward →  body X-forward = +X_body
R_C_B_ZERO = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


@dataclass
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0

    def normalize(self, u_pix: float, v_pix: float, zoom: float = 1.0):
        """Pixel → normalized image coordinates (x/z, y/z) at unit focal length.

        With optical zoom factor `zoom`, the *effective* focal length scales as
        zoom * f_baseline, so the normalized coordinate scales by 1/zoom.
        """
        fx_eff = self.fx * zoom
        fy_eff = self.fy * zoom
        return ((u_pix - self.cx) / fx_eff,
                (v_pix - self.cy) / fy_eff)


def gimbal_R_c_b(gimbal_rpy_rad: np.ndarray,
                 R_c_b_zero: np.ndarray = R_C_B_ZERO) -> np.ndarray:
    """Camera-to-body rotation given gimbal RPY (radians).

    Convention matches los_rate_controller.py `_publish_state`:
        R_gimbal_in_body = Rz(yaw) Rx(roll) Ry(pitch)    (ZXY intrinsic)
    and v_body = R_gimbal_in_body @ v_gimbal_zero, where the gimbal-zero frame
    is body-FLU (camera optical axis along body +X).  R_c_b_zero rotates a
    CV-convention camera vector (X-right, Y-down, Z-forward) into that
    gimbal-zero frame.
    """
    R_gimbal = rpy_zxy_to_rot(
        gimbal_rpy_rad[0], gimbal_rpy_rad[1], gimbal_rpy_rad[2])
    return R_gimbal @ R_c_b_zero


def world_los_from_pixel(u_pix: float, v_pix: float, zoom: float,
                         intrinsics: CameraIntrinsics,
                         gimbal_rpy_rad: np.ndarray,
                         R_b_e: np.ndarray,
                         R_c_b_zero: np.ndarray = R_C_B_ZERO) -> np.ndarray:
    """Unit world-frame (ENU) line-of-sight from a pixel detection.

    Range-free by construction — composes the LOS *direction* from the pixel ray
    through the gimbal ∘ attitude chain, independent of any target range:

        n̂^e = unit( R_b^e · R_c^b(gimbal) · unit([x̄, ȳ, 1]) )

    where ``(x̄, ȳ) = intrinsics.normalize(u_pix, v_pix, zoom)`` and ``R_b_e`` is
    the body→earth rotation (``quaternion.quat_to_rot`` of the aircraft odom
    attitude). This is exactly the bearing composition already used inside
    ``direct_projection_ekf_node`` / ``simple_ekf_node``, factored out so the
    raw-IBVS LOS publisher and the EKFs share one convention. The camera-in-body
    lever arm is deliberately not applied: it shifts the ray *origin*, not its
    direction, and raw IBVS servos direction only. Returns a unit 3-vector.
    """
    x_bar, y_bar = intrinsics.normalize(u_pix, v_pix, zoom)
    n_cam = np.array([x_bar, y_bar, 1.0])
    n_cam /= np.linalg.norm(n_cam)
    R_c_e = R_b_e @ gimbal_R_c_b(gimbal_rpy_rad, R_c_b_zero)
    n = R_c_e @ n_cam
    norm = float(np.linalg.norm(n))
    return n / norm if norm > 1e-12 else n


def project_point(p_target_world: np.ndarray,
                  p_aircraft_world: np.ndarray,
                  R_b_e: np.ndarray,
                  R_c_b: np.ndarray,
                  t_cam_in_body: Optional[np.ndarray] = None) -> np.ndarray:
    """Project a world point to *normalized* camera coords (x/z, y/z).

    Returns (p_bar_x, p_bar_y, depth).  Depth is in camera frame (Z forward).

    `t_cam_in_body` is the camera optical-center position expressed in the
    body (FLU) frame.  Default = origin (paper's `t_b^c = 0`); for IrisGimbal3
    in this project it is (0.0, -0.10, 0.12).
    """
    R_c_e = R_b_e @ R_c_b
    delta = p_target_world - p_aircraft_world  # body → target in EFF
    if t_cam_in_body is not None:
        # Subtract the body→camera lever arm so the projection is from the
        # camera optical center rather than the body origin.
        delta = delta - R_b_e @ t_cam_in_body
    p_cam = R_c_e.T @ delta
    z = p_cam[2]
    if abs(z) < 1e-6:
        z = 1e-6 if z >= 0 else -1e-6
    return np.array([p_cam[0] / z, p_cam[1] / z, z])


def interaction_matrix(p_bar: np.ndarray, depth: float) -> np.ndarray:
    """Image-Jacobian L_s ∈ R^{2×6} at normalized feature `p_bar = (x, y)`.

    Paper Eq. 7: maps camera twist (c_v, c_ω) in CF to image feature rate p̄_dot.
        p̄_dot = L_s @ [c_v; c_ω]
    Standard IBVS form (depth Z, normalized coords x = X/Z, y = Y/Z):
        L_s = [ -1/Z   0    x/Z    x*y     -(1+x^2)   y
                 0   -1/Z   y/Z   1+y^2    -x*y      -x ]
    """
    x, y = p_bar[0], p_bar[1]
    if abs(depth) < 1e-6:
        depth = 1e-6 if depth >= 0 else -1e-6
    inv_z = 1.0 / depth
    return np.array([
        [-inv_z, 0.0, x * inv_z, x * y, -(1.0 + x * x), y],
        [0.0, -inv_z, y * inv_z, 1.0 + y * y, -x * y, -x],
    ])
