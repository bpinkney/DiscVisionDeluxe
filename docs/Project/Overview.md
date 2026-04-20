# DiscVisionDeluxe — Project Overview

## Purpose
DiscVisionDeluxe simulates disc golf flight. A real throw can be captured via a FLIR camera and AprilTag disc marker, or throw parameters can be entered manually through a UI panel. Either way, the DfisX physics engine simulates the full flight and visualises it in Unity.

## Two Input Paths, One Entrypoint

```
Path A (Live Camera):
  FLIR Spinnaker Camera
    → SpinnakerCameraCapture (background thread, 290fps)
    → AprilTagDetector (background thread, ~25fps)
    → LiveDiscTracker (main thread)
    → DiscKalmanFilter
    → onThrowComplete(DiscInitState)
    → DiscVisualizer.LaunchDfisX()       ← single entrypoint

Path B (Manual / CSV):
  UI Sliders (ThrowParameterPanelController)
    OR
  csvlog.csv → CsvLogReader → DiscSimulator → DiscKalmanFilter
    → onThrowComplete(DiscInitState)
    → DiscVisualizer.LaunchDfisX()       ← same entrypoint
```

**`DiscVisualizer.LaunchDfisX(DiscInitState)` is the only valid way to start a simulated throw.** All input paths must converge here. Never call `DiscFlightSimulator.NewThrow()` directly from application code.

## Flight Simulation

```
DiscFlightSimulator.NewThrow(DiscInitState)
  → StepForUnityFrame() called each Update()
      → sub-steps at 1kHz internally
      → Daero (aerodynamic forces)
      → Dgyro (gyroscopic precession)
      → Dpropagate (Euler integration)
  → OnThrowFinished (disc z <= 0)
  → Rigidbody handoff (bounce, roll, settle via PhysX)
```

## Coordinate System
- DfisX internal: **Z-up** (X forward, Y right, Z up)
- Gravity: `(0, 0, -9.8 * mass)`
- Unity scene: **Y-up** — conversion happens in `DiscFlightSimulator.ToUnitySpace()` / `ToUnityRotation()`
- Disc flies along Unity **+X axis**; all camera offsets and distance markers are placed along +X

## Key Components
| Component | Location | Role |
|---|---|---|
| DiscVisualizer | Assets/DiscVisionDeluxe/ | Owns disc GameObject; drives DiscFlightSimulator; handles Rigidbody handoff |
| DiscFlightSimulator | Assets/DfisX/Runtime/ | Per-frame sim stepping; ToUnitySpace conversion |
| DiscKalmanFilter | Assets/DiscVisionDeluxe/ | Estimates initial state from noisy measurements |
| LiveDiscTracker | Assets/DiscVisionDeluxe/Camera/ | Bridges camera detections to KF |
| ThrowParameterPanelController | Assets/DiscVisionDeluxe/UI/ | Manual throw input + disc selection |

## Project Tracks
- **Track A** (Priority 1): Hardware — FLIR Spinnaker + AprilTag → live throw detection → sim
- **Track B** (Priority 2/3): Sim experience — practice range, UI panels, disc database, polish

## Tech Stack
- Unity 6000.4.0f1, URP 17.4.0, Cinemachine 3.1.6, Burst 1.8.17
- FLIR Spinnaker SDK (P/Invoke via SpinnakerC_v140.dll)
- Emgu CV 4.12 (AprilTag detection via cvextern.dll)
