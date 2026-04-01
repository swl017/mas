## Ticket #005: Hardware verification — gimbal encoder wiring

### What
Verify 5 encoder behaviors on real SIYI hardware: sign convention, stream continuity, los_rate_controller compat, point_to_region closed-loop, multiview reprojection error

### Why
Encoder angles (0x26) replaced IMU angles as primary gimbal state; software is done but hardware behavior is unvalidated. Wrong sign convention would cause divergent pointing or triangulation error.

### Scope boundary
Fix sign multipliers or frame conventions if needed. Do not change the architectural decision (encoder as primary). Do not touch sim path.

### Affected modules
`gimbal_controller`, `mas_multiview`, `mas_policy`

### Acceptance criteria
All 5 checklist items in `gap_analysis.md` Priority 1 pass on real hardware

### Flow
Light (known scope, need I -> S -> Y if fixes are required)

### Status
Blocked on hardware access
