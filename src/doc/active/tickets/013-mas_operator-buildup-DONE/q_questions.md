## Open Questions — Resolved

| # | Question | Resolution |
|---|----------|------------|
| 1 | Vehicle list source — match vehicles.yaml or standalone? | Separate `config/operator.yaml` with vehicle list, matching mas_mission's YAML-loading pattern |
| 2 | Which triangulation topic to subscribe to? | `/{veh}/tracked_objects/class_{i}` only (processed data from tracker), not raw multiview output |
| 3 | `triangulated_points` namespace — global or per-vehicle? | Per-drone: `/{veh}/triangulated_points`. Fix ARCHITECTURE.md line 30 |
| 4 | `mavros_msgs` availability — hard or soft depend? | Hard-depend on `mavros_msgs` |
| 5 | Display technology — curses vs rich vs rqt/rviz? | curses for terminal UI (zero deps, tmux-friendly) + RViz for 3D spatial context with labeled markers |
| 6 | Rosbag testing — sim time or wall clock? | Bag has metadata.yaml now (after reindex); use `--clock` for sim time |
| 7 | Number of vehicles for testing? | Default `["px4_1", "px4_2"]`, configurable to arbitrary count. Rest are target drones |
| 8 | Threading model? | Option A — curses in separate daemon thread, rclpy.spin() in main thread, shared FleetState with lock |
| 9 | Alert persistence after clearing? | Linger for 1 second |
| 10 | Auto-pick scope — per-vehicle or global? | Global toggle that publishes to all vehicles |
| 11 | RViz marker publishing — own or reuse multiview? | Operator publishes its own MarkerArray (ground station is a separate machine from agents) |

### Additional clarifications from review

**AoI definition:** AoI is cross-agent data latency, not operator-to-agent. Example: agent 0's AoI = data latency from other agent(s). The operator approximates this by measuring `now - msg.header.stamp` for each agent's `common_frame/odom` (same DDS network, reasonable proxy).

**RViz AoI visualization:** Add 2-way line markers connecting all agent pairs, color-coded by AoI severity (green → yellow → red based on thresholds), with text labels showing ms value. For N agents: N*(N-1)/2 lines.

**Target selection UX:** Operator selects targets by track ID in the curses table, cross-referenced with labeled markers in RViz. No minimap — RViz provides the spatial context.

**`scripts/operator.py` status:** Outdated. The d2 architecture diagram is correct and represents the target interface.
