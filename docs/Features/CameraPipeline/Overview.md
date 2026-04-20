# Camera Pipeline — Overview

## Purpose
Captures real disc golf throws using a FLIR Spinnaker camera with AprilTag markers on the disc. Detects the disc's 6-DOF pose at ~25fps, passes measurements to the Kalman Filter, and fires a throw event when the initial state is confirmed.

## Requirements
- Capture at 290fps (Mono8) from FLIR Blackfly S camera
- Detect AprilTag markers and estimate disc pose using solvePnP
- Run capture and detection on background threads (main thread safety contract)
- Transform camera-space coordinates to DfisX Z-up world frame
- Support lens calibration and ground plane calibration workflows

## Architecture
```
FLIR Camera (290fps Mono8)
  → CaptureLoop thread → _pendingDetectQueue (max 60 frames)
  → DetectionLoop thread (ArucoInvoke.DetectMarkers, ~25fps) → _detectionQueue
  → Main thread Update() → onTagsDetected (UnityEvent)
  → LiveDiscTracker.OnTagsDetected() → BuildMeasurement() → cameraToWorld transform
  → DiscKalmanFilter.AddMeasurement()
  → onThrowComplete(DiscInitState) → DiscVisualizer.LaunchDfisX()
```

**Why P/Invoke:** `SpinnakerNET_v140.dll` is C++/CLI mixed-mode and cannot be loaded as a managed assembly by Unity. All bindings use `DllImport("SpinnakerC_v140")` in `SpinnakerCAPI.cs`.

**Why Emgu CV:** OpenCVForUnity costs ~$95. Emgu CV 4.12 is free (Apache 2.0, NuGet). Both wrap the same OpenCV 4.x Aruco module.

## Place in the Project
Track A (Priority 1). Produces `KFMeasurement` values that feed the Kalman Filter Pipeline. The ground plane calibration defines the world coordinate frame. When camera calibration is poor, all downstream pose estimates are incorrect.
