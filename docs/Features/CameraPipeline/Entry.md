---
name: Camera Pipeline Entry
description: Status and key paths for the FLIR Spinnaker + AprilTag live capture pipeline
type: project
---

# Camera Pipeline Entry

- **Status**: 🔧 In Progress
- **Active task**: CAM-ISSUE-1 — Redo camera calibration (RMS 3.4274 → target <1.0)
- **Blocker**: Physical recalibration required (lighting, checkerboard setup)

## Key Scripts
- `Assets/DiscVisionDeluxe/Camera/SpinnakerCameraCapture.cs`
- `Assets/DiscVisionDeluxe/Camera/AprilTagDetector.cs`
- `Assets/DiscVisionDeluxe/Camera/LiveDiscTracker.cs`
- `Assets/DiscVisionDeluxe/Camera/CameraCalibration.cs`
- `Assets/DiscVisionDeluxe/Camera/GroundPlaneCalibration.cs`
- `Assets/DiscVisionDeluxe/Camera/GroundPlaneCalibrator.cs`
- `Assets/DiscVisionDeluxe/Camera/SpinnakerCAPI.cs`

## Load More
- Full context: [Design.md](Design.md)
- Overview: [Overview.md](Overview.md)
- Active tasks: [Tasks.md](Tasks.md)
- Open issues: [Bugs.md](Bugs.md)
- History: [CHANGELOG.md](CHANGELOG.md)
