# MAS — Multi-Agent Drone Observation System

## Architecture
See [ARCHITECTURE.md](ARCHITECTURE.md) for node graph and data flow.

## Session Workflow Protocol

At the **START** of each session:
- Read `doc/active/feature_list.json` and `doc/active/progress.txt`
- Read `ARCHITECTURE.md` for node boundaries and topic wiring

At the **END** of each session:
- Append to `doc/active/progress.txt`: what was done, what's next
- Update `doc/active/feature_list.json` if any feature status changed
- Update the relevant package's `CONTEXT.md` if its interface (topics, services, parameters) changed
- Update `ARCHITECTURE.md` if inter-package dependencies or topic wiring changed

## Package Navigation
Each package has a `CONTEXT.md` describing its nodes' purpose, subscriptions, publishers, services, parameters, and dependencies. Read the relevant `CONTEXT.md` before working on a package.

## Specs
Authoritative specifications live in `doc/*_spec.md`. These define *what* to build. Do not duplicate spec content elsewhere.

## Folder Semantics
```
mas/src/
├── ARCHITECTURE.md          # System-wide node graph and topic flow
├── CLAUDE.md                # This file (session workflow)
├── doc/
│   ├── active/              # Multi-session tracking (feature_list.json, progress.txt)
│   └── *_spec.md            # Authoritative specs
├── gimbal_controller/       # Gimbal hardware interface + pointing
│   └── CONTEXT.md
├── mas_common_frame/        # GPS → common frame transforms
│   └── CONTEXT.md
├── mas_mission/             # Mission state machine + command routing
│   └── CONTEXT.md
├── mas_multiview/           # Multi-view triangulation (C++)
│   └── CONTEXT.md
├── mas_tracker/             # 3D multi-object tracking (SORT)
│   └── CONTEXT.md
├── ultralytics_ros/         # YOLO detection
│   └── CONTEXT.md
└── template_ros2/           # AI workflow add-on templates
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

## Rationale
For background on why this workflow exists, see `ai_workflow.md`.
