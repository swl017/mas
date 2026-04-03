## Ticket #016: Frustum visualization roll/pitch swap — DONE

### What
The frustum visualization in RViz appeared rotated in roll when the gimbal pitched. Yaw was correct.

### Root cause
The Euler angle composition in `triangulation_node.cpp` used **intrinsic** order (reversed matrix multiplication) when it should have used **extrinsic** order matching the physical gimbal kinematic chain.

The iris_gimbal3 kinematic chain is: body → yaw(Z) → roll(X) → pitch(Y).
This requires extrinsic ZXY: **R = Rz(yaw) × Rx(roll) × Ry(pitch)**.

The code was building intrinsic ZXY: R = Ry(pitch) × Rx(roll) × Rz(yaw) — the reversed product. At yaw≈90°, this caused the pitch rotation to act around the look direction (appearing as roll twist) instead of tilting the camera up/down.

Additionally, the pitch sign needed negation: the pitch joint's positive direction is opposite to Ry's positive rotation convention in FLU.

### Fix
`triangulation_node.cpp` lines 418-425:
- Changed `zxy` case from `Ry * Rx * Rz` → `Rz * Rx * Ry(-pitch)`
- Changed `zyx` case from `Rx * Ry * Rz` → `Rz * Ry * Rx` (same fix pattern)
- Removed debug logging added during diagnosis

### Affected modules
`mas_multiview` — `triangulation_node.cpp`

### Verified
Frustum and rays now correctly show pitch tilt and yaw rotation in RViz.
