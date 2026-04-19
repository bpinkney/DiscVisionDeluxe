# DiscVisionDeluxe — Documentation

DiscVisionDeluxe is a Unity 6 disc golf flight simulator. A FLIR Spinnaker camera with AprilTag disc markers captures real throws; a Kalman Filter estimates the disc's initial state; the DfisX physics engine (ported from C++) simulates the full flight. The simulator can also accept manual throw parameters from a UI panel.

## Contents

### Setup
- [Unity Project Setup](setup/unity-setup.md) — Unity version, packages, URP configuration, scene list
- [Camera Hardware Setup](setup/camera-hardware.md) — FLIR Spinnaker SDK installation, DLL configuration
- [Camera Calibration](setup/camera-calibration.md) — Lens calibration and ground plane calibration workflows

### Architecture
- [System Overview](architecture/overview.md) — High-level diagram, two input paths, key entrypoints
- [DfisX Physics Engine](architecture/dfisX-physics.md) — C# port design, blittable structs, coordinate system
- [Kalman Filter Pipeline](architecture/kf-pipeline.md) — KF design, CSV path, live camera path
- [Camera Pipeline](architecture/camera-pipeline.md) — P/Invoke rationale, thread architecture, AprilTag
- [Assembly Reference](architecture/assembly-reference.md) — All asmdef files, dependency graph, platform rules

### Development
- [Adding Features](development/adding-features.md) — Conventions, invariants, how to wire new inputs
- [Disc Database](development/disc-database.md) — PDGA CSV structure, import tool, DiscModel ScriptableObject
- [Open Issues](development/open-issues.md) — Known problems with context and workarounds
- [Decision Log](development/decision-log.md) — All architecture decisions with rationale

### AI Context
The `ai/` folder contains context files for the AI coding assistant. They are not intended for human reading.
