# Camera Pipeline — Design

## Hardware: BFS-U3-04S2C-CS (FLIR Blackfly S)
| Spec | Value |
|---|---|
| Sensor | Sony IMX287, 1/2.9" CMOS, global shutter |
| Resolution | 720 × 540 |
| Max FPS Mono8 | 290 (ISP on, demosaics Bayer→mono) |
| Max FPS BayerRG8 | 522 (ISP off — USB bandwidth issues, do not use) |
| Interface | USB 3.1 Gen 1 |
| Recommended exposure | AutoExposureMaxUs=1000µs |
| Recommended gain | 0 (auto) |
| TargetFps | 290 |

## Spinnaker DLL Setup (Assets/Plugins/Spinnaker/)
All: Native, x86 UNCHECKED, Editor + Standalone, CPU x64.
`SpinnakerC_v140.dll`, `Spinnaker_v140.dll`, `GCBase_MD_VC140_v3_0.dll`, `GenApi_MD_VC140_v3_0.dll`, `log4cpp_MD_VC140_v3_0.dll`, `Log_MD_VC140_v3_0.dll`, `MathParser_MD_VC140_v3_0.dll`, `NodeMapData_MD_VC140_v3_0.dll`, `XMLParser_MD_VC140_v3_0.dll`, `libiomp5md.dll` — from `C:\Program Files\FLIR Systems\Spinnaker\bin64\vs2015\`.
`vcomp140.dll` — from `C:\Windows\System32\` (NOT from Spinnaker bin64).

## Key Files
| File | Role |
|---|---|
| SpinnakerCAPI.cs | All DllImport("SpinnakerC_v140") P/Invoke bindings. Must be public. |
| SpinnakerCameraCapture.cs | CaptureLoop + DetectionLoop threads; ConcurrentQueue<DetectionFrame> (max 60); fires onTagsDetected on main thread |
| AprilTagDetector.cs | Not MonoBehaviour; Emgu CV 4.12; ArucoInvoke.DetectMarkers; DetectorParameters.GetDefault(); solvePnP with Matrix<double>(3,1) output |
| CameraCalibration.cs | Serializable; fx/fy/cx/cy + distCoeffs[5]; LoadFromJson/SaveToJson |
| GroundPlaneCalibration.cs | Serializable; worldToCamera[16], cameraToWorld[16], cameraHeightM |
| GroundPlaneCalibrator.cs | MonoBehaviour; averages N rvec+tvec frames; computes cameraToWorld; saves ground_plane.json |
| LiveDiscTracker.cs | Subscribes to onTagsDetected; builds KFMeasurements; drives DiscKalmanFilter; fires onThrowComplete |
| Editor/CameraCalibrationEditor.cs | EditorWindow; live FLIR preview; checkerboard capture + CalibrateCamera |

## Thread Architecture
- **CaptureLoop** (SpinnakerCapture thread): grabs frames at 290fps → enqueues to `_pendingDetectQueue` (max 60)
- **DetectionLoop** (AprilTagDetection thread): dequeues → ArucoInvoke.DetectMarkers (~25-30fps effective) → pushes to `_detectionQueue` with original capture timestamp
- **Update()** (main thread): drains `_detectionQueue` → fires `onTagsDetected`

**Rule:** Never call Unity API from background thread. Only cross-thread comms via `ConcurrentQueue`.

## Clock Synchronisation
Single `Stopwatch` (`_captureClock`) started in `StartCapture()`. Every frame tagged with nanosecond capture timestamp. `LiveDiscTracker` reads `cameraCapture.ElapsedCaptureNs` for current time. Never mix with `DateTime.Now` or `Time.realtimeSinceStartup` — causes immediate detection timeouts.

## AprilTag → KF Flow (LiveDiscTracker)
1. `onTagsDetected` fires on main thread → `OnTagsDetected()` → `BuildMeasurement()`
2. `BuildMeasurement()`: applies `cameraToWorld` if ground plane profile loaded; raw fallback: `kfX=camZ, kfY=camX, kfZ=-camY`
3. → `_kf.AddMeasurement()`
4. `Update()` drives `_kf.Step()` using `cameraCapture.ElapsedCaptureNs` for timing
5. DetectionTimeout: if no detection for `detectionTimeoutMs` (2000ms), `_kf.SignalDetectionLost()`
6. `_kf.CurrentStage == Complete` → `FireComplete()` → validates speed → `onThrowComplete(DiscInitState)`

## Speed Guards (FireComplete)
Discards throws: speed <5kph or >400kph, NaN position/velocity, `linearPositionM.z <= 0` (clamped to 1.0m — calibration workaround).

## Ground Plane Math
`worldToCamera = [R | t]`, `cameraToWorld = [R^T | -R^T*t]`. `cameraHeightM = tz`.
World frame: origin = reference tag centre on ground, X = tag X (must point downrange), Y = 90° right of X, Z = up.
Transform: `p_world = cameraToWorld * p_camera` (homogeneous multiply, row-major 4×4).

## Calibration Files
`Assets/StreamingAssets/CameraCalibration/`
- `camera_calibration.json` — current RMS 3.4274 (needs redo — see Bugs.md)
- `ground_plane.json`

## asmdef
`DiscVisionDeluxe.Camera.asmdef`: refs DiscVisionDeluxe, Unity.Cinemachine, DfisX.Unity, DfisX.Runtime, Unity.Mathematics, Unity.Collections. Editor + WindowsStandalone64 platforms only.
