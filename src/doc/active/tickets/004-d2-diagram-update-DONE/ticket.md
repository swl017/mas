## Ticket #004: Semantic d2 diagram update

### What
Rename `ray_w` to `target_rays_w` in cross-agent edges, add `chosen_target_ray_w` edge from mas_tracker to mas_policy

### Why
Diagram is the primary architecture reference; stale edges mislead design reviews

### Scope boundary
Only update d2 source and re-export PNG/SVG. Do not redesign layout or add new components.

### Affected modules
`doc/diagrams/mas_architecture_semantic.d2`

### Acceptance criteria
Exported diagram shows `target_rays_w` on cross-agent edges and `chosen_target_ray_w` from tracker to policy

### Flow
Direct fix

### Status
Done (diagram already updated during tracker-ray-selection work, 2026-04-02)
