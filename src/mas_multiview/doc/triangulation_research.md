# Triangulation Implementation Comparison

Comparison of four triangulation implementations for multi-drone observation.

## Summary

| Feature | mas_multiview (C++) | mas_multiview_py | mrcal mid2 | iris_ma6 |
|---|---|---|---|---|
| **Multi-target** | Yes | Yes | No | Yes |
| **Multi-camera (>2)** | Yes | Yes | No (stereo) | Yes |
| **Propagates input covariance** | No | Partial | No (gradients only) | Yes, full |
| **Output covariance** | Yes (Ceres Hessian) | Yes (MC + Hessian) | No | Yes (sandwich formula) |
| **Batch/GPU** | No | No | No | Yes (PyTorch) |
| **Algorithm** | Midpoint init + Ceres reprojection opt | Midpoint + BFGS + Monte Carlo | Lee-Civera Mid2 heuristic | Weighted midpoint (linear solve) |

---

## 1. mas_multiview (C++)

**Source:** `mas_multiview/lib/multiview_triangulation/src/multiview_triangulation.cpp`

### Algorithm
1. **Initial estimate** — midpoint method: for each ray pair, find closest points and average. Pairs weighted by reprojection error.
2. **Refinement** — Ceres Solver (DENSE_SCHUR) minimizing:
   - Reprojection error (Huber loss) across all cameras
   - Regularization toward midpoint initial guess
3. Filters associations by max reprojection error (20% image width).

### Multi-target support: YES
- Generates **Cartesian product** of all detections across cameras (one detection per camera per combination).
- Each combination is an independent "association" optimized separately.
- No appearance-based matching — purely exhaustive combinatorial.

### Uncertainty propagation: NO
- **Output covariance:** 3x3 matrix from Ceres covariance estimation (inverse Hessian of reprojection residuals at solution).
- **Does NOT accept or propagate:**
  - EKF pose covariance
  - Camera intrinsics uncertainty
  - Gimbal angle uncertainty
  - Detection bbox uncertainty as an explicit input
- Covariance reflects only reprojection geometry, not upstream noise.

---

## 2. mas_multiview_py (Python)

**Source:** `mas_multiview_py/mas_multiview_py/multiview/multiview_triangulation.py`

### Algorithm
Three methods combined:
1. **Midpoint triangulation** — analytic closest-point averaging across ray pairs.
2. **BFGS optimization** — minimizes reprojection error (scipy), Hessian-based covariance.
3. **Monte Carlo** — perturbs 2D detections (100 samples), triangulates each, computes sample covariance.

Final covariance = average of optimization Hessian and Monte Carlo covariances.

### Multi-target support: YES
- **Greedy correspondence matching** (`find_correspondences()`):
  - All cross-camera detection pairs triangulated and scored by reprojection error.
  - Greedy assignment: each detection used once, targets extended across cameras.
- Each matched target group localized independently via `localize_from_bounding_boxes()`.

### Uncertainty propagation: PARTIAL
- **Propagated:** Detection pixel noise (constant std, scaled by bbox size) via Monte Carlo sampling.
- **NOT propagated:**
  - Odometry covariance (field read from `nav_msgs/Odometry` but ignored)
  - Camera calibration uncertainty
  - Gimbal angle uncertainty

---

## 3. mrcal_triangulate_leecivera_mid2

**Source:** `/home/usrg/source/mrcal/triangulation.cc` (lines 630-679)

### Algorithm
Lee-Civera "Mid2" heuristic from "Triangulation: Why Optimize?" (2019):
```
l0 = sqrt(||v1 × t01||² / ||v0 × v1||²)
l1 = sqrt(||v0 × t01||² / ||v0 × v1||²)
m  = (v0*l0 + t01 + v1*l1) / 2
```
- No explicit objective function minimized — heuristic midpoint variant.
- Chirality check: returns (0,0,0) for parallel/divergent rays.
- Fast, no iteration.

### Multi-target support: NO
- **Binary stereo only:** two direction vectors + baseline translation → one 3D point.
- Signature: `mrcal_triangulate_leecivera_mid2(dm_dv0, dm_dv1, dm_dt01, v0, v1, t01)`
- Caller must loop over targets and camera pairs externally.

### Uncertainty propagation: NO (gradients provided)
- Returns optional gradients `dm/dv0`, `dm/dv1`, `dm/dt01` for external Jacobian-based propagation.
- mrcal library-level docs describe propagating calibration-time noise and observation-time noise at a higher level, but `mid2` itself does not compute covariance.
- **Caveat from docs:** near-parallel geometries produce non-Gaussian error distributions with long tails — first-order propagation breaks down.

### Related mrcal functions
- `mrcal_triangulate_geometric()` — basic closest-approach (highest bias)
- `mrcal_triangulate_lindstrom()` — approximates L2 reprojection minimization
- `mrcal_triangulate_leecivera_wmid2()` — improved near-range variant

---

## 4. iris_ma6/triangulation

**Source:** `/home/usrg/IsaacPX4/IsaacLab/source/isaaclab_tasks/isaaclab_tasks/direct/iris_ma6/triangulation/triangulation.py`

### Algorithm
Weighted midpoint (linear least-squares):
```
A = Σ_c (I - d_c d_c^T)        # sum of projection matrices
b = Σ_c (I - d_c d_c^T) p_c    # sum of projected camera origins
X = A⁻¹ b                       # triangulated point
```
Diagonal regularization for numerical stability. No iterative refinement.

### Multi-target support: YES
- Tensor shape `[N, C, T]`: N environments x C cameras x T targets.
- All targets triangulated simultaneously in vectorized PyTorch operations.
- Per-camera-target validity mask allows partial observations.

### Uncertainty propagation: YES, FULL
Five independent uncertainty sources, all propagated via first-order Jacobians:

| Source | Parameter | Default |
|---|---|---|
| Pixel detection noise | `pix_std` | 7.0 px |
| Camera position | `pos_std` | 0.1 m |
| Camera orientation | `ori_std` | 0.001 rad |
| Gimbal angles | `gimbal_std` | 0.001 rad |
| Camera intrinsics | `intrinsics_std` | 10.0 |

**Covariance formula (sandwich):**
```
A = Σ_c  J_X,c^T  W_c  J_X,c
S_c = Σ_pix + J_θ,c  Σ_θ,c  J_θ,c^T
Σ_X = A⁻¹ (Σ_c  J_X,c^T  W_c  S_c  W_c  J_X,c) A⁻ᵀ
```

Where `J_θ,c` includes Jacobians w.r.t. camera position, orientation, and gimbal angles (nuisance parameters).

### Quality metrics (selectable)
- Trace (A-optimality, default): `tr(Σ_X)`
- Determinant (D-optimality): `det(Σ_X)^(1/3)`
- Max eigenvalue (E-optimality): `λ_max(Σ_X)`

### Validity checks
- Minimum 2 cameras per target
- Condition number of A < threshold
- Positive definite covariance
- Trace below max threshold

---

## Gaps in mas_multiview / mas_multiview_py

1. **No input covariance propagation** — neither package uses EKF pose covariance, calibration uncertainty, or gimbal angle uncertainty. Output covariance only reflects reprojection geometry.
2. **No quality metric selection** — no A/D/E-optimality scores for downstream decision-making.
3. **Combinatorial association** (C++) scales as O(D^C) with D detections per camera and C cameras. Greedy matching (Python) is better but still approximate.
4. **No batch/GPU path** — both are single-threaded, single-frame.

The iris_ma6 implementation demonstrates how full Jacobian-based covariance propagation can be done efficiently in a vectorized framework.

---

## Implementation Research: Adding Covariance Propagation to mas_multiview

### Target hardware
Jetson Orin NX (ARM64, CUDA 12.6 available)

### Language choice: C++ with Eigen

All matrices are tiny (3x3, 2x3). Eigen fixed-size types are stack-allocated with loop unrolling — ~50-100ns per 3x3 inverse. Full pipeline for T=10, C=5: ~25-50 microseconds. cuBLAS/GPU launch overhead dominates for these sizes; CPU is 10-100x faster. No new dependencies needed (Eigen3 and Ceres already in mas_multiview).

Python with NumPy would be 10-100x slower due to interpreter overhead. Numba/JAX not installed on target Jetson.

### Jacobians: analytical (not autodiff)

Pinhole projection + rotation Jacobians are short closed-form expressions. iris_ma6 already derives them — direct port. Avoids new dependencies (autodiff, JAX).

### Available on the system
- Eigen 3.3.7 (`/usr/include/eigen3/`)
- Ceres Solver 2.2.0 (`/usr/local/include/ceres/`)
- CUDA 12.6.68 (`/usr/local/cuda/`) — not needed for this
- NumPy 1.24.4, SciPy 1.10.1 — for MC validation tests only

### Key finding: pose covariance already on the wire
`mas_multiview/src/triangulation_node.cpp` subscribes to `nav_msgs/Odometry` which contains a 6x6 `pose.covariance` field (position + orientation). Currently discarded in `cameraOdomCallback()` — only `pose.pose` is extracted. Extracting this field is the only ROS-side change needed.
