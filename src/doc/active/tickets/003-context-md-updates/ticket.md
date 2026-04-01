## Ticket #003: CONTEXT.md updates for changed interfaces

### What
Four packages have stale CONTEXT.md files that don't reflect current interfaces

### Why
CONTEXT.md is the primary navigation aid for working on a package; stale docs cause wrong assumptions in future sessions

### Scope boundary
Docs only — no code changes. Only update interface descriptions (topics, params, message types). Do not rewrite prose or add new sections.

### Affected modules
`mas_multiview/CONTEXT.md`, `mas_tracker/CONTEXT.md`, `gimbal_controller/CONTEXT.md`, `ultralytics_ros/CONTEXT.md`

### Acceptance criteria
Each CONTEXT.md accurately lists current subscribers, publishers, parameters, and message types (verified against source code)

### Flow
Direct fix

### Status
Pending since 2026-03-27 — blocked on #001 (tracker CONTEXT.md needs final interface)
