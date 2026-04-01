# MAS — Multi-Agent Drone Observation System

## Architecture
See [ARCHITECTURE.md](ARCHITECTURE.md) for node graph and data flow.

## Session Workflow Protocol

At the **START** of each session:
- Read `src/doc/active/feature_list.json` and `src/doc/active/progress.txt`
- Read `ARCHITECTURE.md` for node boundaries and topic wiring
- If working based on tickets, name the session "Ticket xxx <ticket name>"

At the **END** of each session:
- Append to `src/doc/active/progress.txt`: what was done, what's next
- Update `src/doc/active/feature_list.json` if any feature status changed
- Update the relevant package's `CONTEXT.md` if its interface (topics, services, parameters) changed
- Update `ARCHITECTURE.md` if inter-package dependencies or topic wiring changed

## Package Navigation
Each package has a `CONTEXT.md` describing its nodes' purpose, subscriptions, publishers, services, parameters, and dependencies. Read the relevant `CONTEXT.md` before working on a package.

## Specs
Authoritative specifications live in `src/doc/*_spec.md`. These define *what* to build. Do not duplicate spec content elsewhere.

## Folder Semantics
```
mas/
├── CLAUDE.md                    # This file (session workflow)
├── src/
│   ├── ARCHITECTURE.md          # System-wide node graph and topic flow
│   ├── qrispy_workflow.md       # Stage-gated QRISPY development process
│   ├── harness_rationale.md     # Why the workflow is designed this way
│   ├── doc/
│   │   ├── active/              # Multi-session tracking (feature_list.json, progress.txt)
│   │   └── *_spec.md            # Authoritative specs
│   ├── gimbal_controller/       # Gimbal hardware interface + pointing
│   │   └── CONTEXT.md
│   ├── mas_common_frame/        # GPS → common frame transforms
│   │   └── CONTEXT.md
│   ├── mas_mission/             # Mission state machine + command routing
│   │   └── CONTEXT.md
│   ├── mas_msgs/                # Shared message/service definitions
│   ├── mas_multiview/           # Multi-view triangulation (C++)
│   │   └── CONTEXT.md
│   ├── mas_offboard/            # Offboard control interface
│   │   └── CONTEXT.md
│   ├── mas_operator/            # Operator UI / ground station
│   ├── mas_policy/              # High-level policy / observation assembly
│   │   └── CONTEXT.md
│   ├── mas_tracker/             # 3D multi-object tracking (SORT)
│   │   └── CONTEXT.md
│   ├── tmux/                    # Launch session scripts
│   │   └── CONTEXT.md
│   ├── ultralytics_ros/         # YOLO detection
│   │   └── CONTEXT.md
│   └── vision_opencv/           # cv_bridge (vendored)
├── template_isaaclab/           # IsaacLab workflow templates
└── template_ros2/               # ROS2 workflow add-on templates
```

## Build
```bash
# Build all
cd ~/mas && colcon build

# Build single package
colcon build --packages-select <package_name>

# Build with debug symbols
cd ~/mas && source colcon_build_debug.sh
```

## AI Workflow
- **Operational workflow**: `src/qrispy_workflow.md` — stage-gated QRISPY process for development tasks
- **Design rationale**: `src/harness_rationale.md` — why the workflow and meta-file system are designed this way
