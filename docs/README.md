# DiscVisionDeluxe — Documentation

DiscVisionDeluxe is a Unity 6 disc golf flight simulator. A FLIR Spinnaker camera with AprilTag disc markers captures real throws; a Kalman Filter estimates the disc's initial state; the DfisX physics engine (ported from C++) simulates the full flight. The simulator can also accept manual throw parameters from a UI panel.

## Contents

### Project
- [Overview](Project/Overview.md) — High-level diagram, two input paths, key entrypoints, tech stack
- [Development Guidelines](Project/DevelopmentGuidelines.md) — Invariants, conventions, how to wire new inputs
- [Assembly Reference](Project/AssemblyReference.md) — All asmdef files, dependency graph, platform rules
- [Decision Log](Project/DecisionLog.md) — All architecture decisions with rationale

### Setup
- [Unity Project Setup](setup/unity-setup.md) — Unity version, packages, URP configuration, scene list
- [Camera Hardware Setup](setup/camera-hardware.md) — FLIR Spinnaker SDK installation, DLL configuration
- [Camera Calibration](setup/camera-calibration.md) — Lens calibration and ground plane calibration workflows

### Features
| Feature | Status | Entry |
|---|---|---|
| DfisX Physics Engine | ✅ Complete (Burst pending) | [Entry](Features/DfisXPhysicsEngine/Entry.md) |
| Kalman Filter Pipeline | ✅ Complete | [Entry](Features/KalmanFilterPipeline/Entry.md) |
| Camera Pipeline | 🔧 In Progress | [Entry](Features/CameraPipeline/Entry.md) |
| Disc Database | ✅ Complete | [Entry](Features/DiscDatabase/Entry.md) |
| Disc Visualization | 🔧 In Progress | [Entry](Features/DiscVisualization/Entry.md) |
| UI Throw Panel | ✅ Complete | [Entry](Features/UIThrowPanel/Entry.md) |
| UI Results Panel | ✅ Complete | [Entry](Features/UIResultsPanel/Entry.md) |
| Flight Cameras | ✅ Complete | [Entry](Features/FlightCameras/Entry.md) |
| MiniMap | ✅ Complete | [Entry](Features/MiniMap/Entry.md) |
| Scene Environment | 🔧 In Progress | [Entry](Features/SceneEnvironment/Entry.md) |
| Wind Indicator HUD | ⏳ Pending | [Entry](Features/WindIndicatorHUD/Entry.md) |
| Practice Range Modes | ⏳ Pending | [Entry](Features/PracticeRangeModes/Entry.md) |
| Replay System | ⏳ Pending | [Entry](Features/ReplaySystem/Entry.md) |
| Collision Feedback | ⏳ Pending | [Entry](Features/CollisionFeedback/Entry.md) |
| Course Holes | ⏳ Stretch | [Entry](Features/CourseHoles/Entry.md) |
| Burst Migration | ⏳ Pending | [Entry](Features/BurstMigration/Entry.md) |
| Main Menu | ⏳ Pending | [Entry](Features/MainMenu/Entry.md) |

### AI Context
The `ai/` folder contains context files for the AI coding assistant. They are not intended for human reading.
