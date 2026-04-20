# Kalman Filter Pipeline — Overview

## Purpose
The Kalman Filter estimates the disc's initial flight state (position, velocity, spin, orientation) from a series of noisy pose measurements — either from the live FLIR camera or from a CSV log — then fires a completed `DiscInitState` to launch the DfisX simulation.

## Requirements
- Accept measurements from two sources: FLIR/AprilTag detections and CSV log replays
- Filter noisy measurements to produce a stable initial velocity and orientation estimate
- Fire `onThrowComplete(DiscInitState)` → `DiscVisualizer.LaunchDfisX()` when the throw is confirmed
- Run entirely on the main thread (measurements injected from camera thread via queue)

## Architecture
```
CSV path:    csvlog.csv → CsvLogReader → DiscSimulator → DiscKalmanFilter
Live path:   FLIR frame → AprilTagDetector → LiveDiscTracker → DiscKalmanFilter
Both →       onThrowComplete(DiscInitState) → DiscVisualizer.LaunchDfisX()
```

KF stages: `Idle` → `Priming` (collecting initial measurements) → `Tracking` → `Complete` → fires `onThrowComplete`.

## Place in the Project
Sits between the input sources (camera pipeline / CSV) and the physics engine. All measurements arrive in DfisX Z-up coordinates. The `DiscInitState` output is the only data structure that enters DfisX.
