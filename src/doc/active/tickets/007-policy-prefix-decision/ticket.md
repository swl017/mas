## Ticket #007: Policy topic prefix convention decision

### What
Decide whether `policy/*` prefix (mission gate) should be default or remain opt-in via `use_mission_gate:=true`

### Why
Without mission gate, policy commands go directly to actuators with no safety gate. Current opt-in approach means the unsafe path is the default.

### Scope boundary
Launch config only — no node code changes

### Affected modules
`mas_policy/launch/`, `mas_mission/`

### Acceptance criteria
Decision documented; launch files updated if default changes

### Flow
Direct fix (once decision is made)

### Status
Deferred — depends on operator workflow maturity
