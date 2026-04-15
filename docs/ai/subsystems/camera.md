# Camera Pipeline — Reference

## Hardware: BFS-U3-04S2C-CS (FLIR Blackfly S)
| Spec | Value |
|---|---|
| Sensor | Sony IMX287, 1/2.9" CMOS, global shutter |
| Resolution | 720 × 540 |
| Max FPS Mono8 | 290 (ISP on, demosaics Bayer→mono) |
| Max FPS BayerRG8 | 522 (ISP off, requires CvtColor — USB bandwidth issues at this rate) |
| Interface | USB 3.1 Gen 1 |
| Recommended exposure | AutoExposureMaxUs=1000µs (cap prevents motion blur in dim light) |
| Recommended gain | 0 (auto) |
| TargetFps | 290 (NOT 60 — default was wrong) |

## Spinnaker DLL Setup (Assets/Plugins/Spinnaker/)
All: Native, x86 UNCHECKED, Editor + Standalone, CPU x64.
SpinnakerC_v140.dll, Spinnaker_v140.dll, SpinnakerNET_v140.dll (present but unused), GCBase_MD_VC140_v3_0.dll, GenApi_MD_VC140_v3_0.dll, log4cpp_MD_VC140_v3_0.dll, Log_MD_VC140_v3_0.dll, MathParser_MD_VC140_v3_0.dll, NodeMapData_MD_VC140_v3_0.dll, XMLParser_MD_VC140_v3_0.dll, libiomp5md.dll — all from C:\Program Files\FLIR Systems\Spinnaker\bin64\vs2015\.
vcomp140.dll — from C:\Windows\System32\.

## Key Files (Assets/DiscVisionDeluxe/Camera/)
| File | Role |
|---|---|
| SpinnakerCAPI.cs | public static class; all DllImport("SpinnakerC_v140") P/Invoke bindings |
| SpinnakerCameraCapture.cs | MonoBehaviour; CaptureLoop thread (290fps) + DetectionLoop thread; ConcurrentQueue<DetectionFrame> (60 frames); fires onTagsDetected on main thread |
| AprilTagDetector.cs | Not MonoBehaviour; Emgu CV 4.12; ArucoInvoke.DetectMarkers; DetectorParameters.GetDefault(); solvePnP with Matrix<double>(3,1) output |
| CameraCalibration.cs | Serializable; fx/fy/cx/cy + distCoeffs[5]; LoadFromJson/SaveToJson/LoadProfile |
| GroundPlaneCalibration.cs | Serializable; worldToCamera[16], cameraToWorld[16], cameraHeightM; LoadProfile/SaveToJson |
| GroundPlaneCalibrator.cs | MonoBehaviour; averages N rvec+tvec frames; computes cameraToWorld; saves ground_plane.json |
| LiveDiscTracker.cs | MonoBehaviour; subscribes to onTagsDetected; builds KFMeasurements; drives DiscKalmanFilter; fires onThrowComplete |
| FollowFlightCamera.cs | CM3 API; camera mode switching |
| Editor/CameraCalibrationEditor.cs | EditorWindow; runs in Edit mode; live FLIR preview; checkerboard capture + CalibrateCamera |

## asmdef
DiscVisionDeluxe.Camera.asmdef: refs DiscVisionDeluxe, Unity.Cinemachine, DfisX.Unity, DfisX.Runtime, Unity.Mathematics, Unity.Collections. No precompiledReferences. Editor + WindowsStandalone64 only.

## Thread Pipeline
- CaptureLoop (SpinnakerCapture thread): grabs frames at 290fps → enqueues to _pendingDetectQueue (max 60)
- DetectionLoop (AprilTagDetection thread): dequeues → AprilTag detect (~25-30fps effective) → pushes to _detectionQueue (ConcurrentQueue<DetectionFrame>) with original capture timestamp
- Update() (main thread): drains _detectionQueue → sets LastDetectionTimestampNs → fires onTagsDetected

## AprilTag → KF Flow (LiveDiscTracker)
1. onTagsDetected fires on main thread → OnTagsDetected() → BuildMeasurement()
2. BuildMeasurement(): applies cameraToWorld if ground plane profile loaded; raw fallback: kfX=camZ, kfY=camX, kfZ=-camY
3. → _kf.AddMeasurement()
4. Update() drives _kf.Step() using cameraCapture.ElapsedCaptureNs for timing
5. DetectionTimeout: if no detection for detectionTimeoutMs (2000ms), _kf.SignalDetectionLost()
6. _kf.CurrentStage == Complete → FireComplete() → validates speed → onThrowComplete(DiscInitState)

## Ground Plane Math
worldToCamera = [R | t], cameraToWorld = [R^T | -R^T*t]. cameraHeightM = tz.
World frame: origin = reference tag centre on ground, X = tag X (must point downrange), Y = 90° right of X, Z = up.
Transform: p_world = cameraToWorld * p_camera (homogeneous multiply, row-major 4×4).

## Speed Guards (FireComplete)
Discards throws: speed <5kph or >400kph, NaN position/velocity, linearPositionM.z <=0 (clamped to 1.0m — calibration workaround).

## Calibration Files Location
Assets/StreamingAssets/CameraCalibration/
- camera_calibration.json (current RMS 3.4274 — needs redo)
- ground_plane.json

## Open Issues
### ISSUE-1: Camera Calibration RMS 3.4274 — Needs Redo (High)
Target <1.0 (good), <0.5 (excellent). Current RMS causes inaccurate solvePnP and is root cause of Z-axis inversion.
Fix: redo with even lighting, 20-30 frames, varied board angles, full frame coverage, measure square size with calipers.
Current values: fx=321.0, fy=320.6, cx=353.6, cy=281.8.

### ISSUE-2: World Z Axis Inverted in Ground Plane (Medium)
Disc height shows negative when should be positive. Root cause: poor intrinsics from ISSUE-1 → solvePnP error → wrong cameraToWorld Z row.
Workaround: FireComplete() clamps linearPositionM.z=1.0f when negative. Fix: redo ISSUE-1 → redo ground calibration.

### ISSUE-3: Real Throws Not Detected Without Establishment (Medium)
Disc must be held still ~0.5s before moving for KF to prime. Natural throw from outside frame not reliably caught.
Options: reduce image before detection (halve intrinsics for solvePnP), lower primeMinVar, ROI mode for higher fps.

### ISSUE-4: Detection Timeout vs Queue Drain Time (Low)
At 30ms/frame × 60 frames = 1800ms worst-case drain. Timeout=2000ms (200ms margin). Existing scenes may retain old 500ms value — update in Inspector.
