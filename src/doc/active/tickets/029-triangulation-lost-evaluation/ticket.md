## Ticket: Evaluate replacing `mas_multiview` nonlinear triangulation with GTSAM LOST

**What**: Evaluate whether the current `mas_multiview` triangulation backend should be replaced, or optionally augmented, with GTSAM's LOST triangulation method for faster per-association point estimation.

**Why**: The current implementation performs brute-force association across camera detections and then runs a Ceres-based nonlinear reprojection optimization for each surviving association. This likely increases latency and runtime variance as detection count or camera count grows. GTSAM LOST is a non-iterative weighted linear triangulation method that may reduce solve cost while preserving comparable accuracy to optimal triangulation.

**Status**: Backlog
**Priority**: Low

**Scope boundary**:
- DO: Compare the current `mas_multiview` solver path against a GTSAM LOST-based backend
- DO: Quantify expected benefit in runtime, estimator quality, and implementation complexity
- DO: Assess whether existing covariance propagation can be reused with a LOST point estimate
- DO: Propose an incremental migration path with fallback to the current Ceres backend
- DO NOT: Rewrite detection association as part of this ticket
- DO NOT: Convert the full package to factor-graph optimization
- DO NOT: Change ROS message types or downstream interfaces unless evaluation proves it necessary

**Current implementation facts to validate during the ticket**:
- `mas_multiview` currently enumerates Cartesian-product associations across non-empty cameras
- Each association is initialized by midpoint geometry and refined with a Ceres nonlinear solve
- A separate first-order covariance propagation step computes 3x3 output covariance from pixel, pose, orientation, and gimbal uncertainty
- The main potential speedup from LOST is in per-association estimation, not in association generation

**Key questions to answer**:
1. Is per-association Ceres optimization a meaningful runtime bottleneck in realistic operating regimes?
2. Does LOST provide equal or acceptable estimation quality under the project's camera geometry and noise levels?
3. Can the current covariance propagation module be reused unchanged, or does it need re-derivation or recalibration?
4. Does removing nonlinear robust loss create a regression when associations are imperfect?
5. Is a dual-backend design (`ceres` vs `gtsam_lost`) the right migration path?

**Expected work**:
1. Read and document the current triangulation stack in `src/mas_multiview/`
2. Add a design note comparing:
   - Current Ceres reprojection optimization
   - GTSAM DLT
   - GTSAM LOST
3. Identify the minimal interface seam for swapping the triangulation backend
4. Estimate build/dependency impact of adding GTSAM to the ROS/CMake environment
5. Define a benchmark plan using recorded detections or synthetic camera measurements
6. Recommend one of:
   - keep current solver
   - add LOST as optional backend
   - replace Ceres backend fully

**Affected modules**:
- `src/mas_multiview/lib/multiview_triangulation/`
- `src/mas_multiview/src/triangulation_node.cpp`
- `src/mas_multiview/CMakeLists.txt`
- `src/mas_multiview/package.xml`

**Acceptance criteria**:
- A written comparison exists between current triangulation and GTSAM LOST, focused on runtime, robustness, and uncertainty handling
- The runtime bottleneck is identified with evidence, not guesswork
- The migration seam and dependency impact are documented
- A clear recommendation is made: no change, optional LOST backend, or full switch
- If implementation is approved later, the proposed path preserves current ROS I/O and allows fallback to the existing solver during rollout

**Reference**:
- GTSAM LOST article: `https://gtsam.org/2023/02/04/lost-triangulation.html`
- Related local tickets: `024-triangulation-fix`, `017-triangulation-default-zero-bug`

**Flow**: Light
