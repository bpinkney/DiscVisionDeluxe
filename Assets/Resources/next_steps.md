# DiscVisionDeluxe — Next Steps & Development Archive

## !! IMPORTANT — READ BEFORE EVERY SESSION !!
This file is the living archive of all work done after the initial DfisX port.
Before any new session: read this file AND `porting_notes.md`.
Before creating any new `.cs` file: search existing files — extend rather than duplicate.
After every session: append a row to the Session Log at the bottom.

---

## Project Status at Start of This Document
All 5 phases of the DfisX port are complete (see `porting_notes.md` for full details).
The sim can launch disc throws manually via `DiscThrowDebugger` and visualise flight + Rigidbody
landing. There is no live camera input, no proper UI, and no sim environment beyond a flat plane.

**Unity version:** 6000.4.0f1 (Unity 6)
**Render pipeline at start:** Built-in (no URP)

---

## Master Plan Summary

Two parallel tracks:

**Track A — Real-World Hardware (Priority 1)**
Connect a FLIR Spinnaker camera + AprilTag disc markers so real disc throws drive the simulation.
This is the most important track as it unlocks the core product vision.

**Track B — Sim Experience (Priority 2 + 3)**
Build the simulator into a standalone practice tool with proper UI, environment, camera work,
a large disc library, and polish comparable to commercial golf sims.

---

## Phase / Task Index

### Infrastructure (Do First)
| ID | Task | Status |
|----|------|--------|
| INF-1 | Create `DiscVisionDeluxe.asmdef` | ✅ Complete |
| INF-2 | URP migration | ✅ manifest.json updated — **manual steps required in Unity Editor** (see INF-2 note) |
| INF-3 | Create this archive file (`next_steps.md`) | ✅ Complete |
| INF-4 | Burst migration (`System.Random` → `Unity.Mathematics.Random`, `[BurstCompile]`) | 🔲 Pending |
| INF-5 | Main menu scene + `AppSettings.cs` persistence | 🔲 Pending |
| INF-6 | Add packages to manifest (URP, Cinemachine, Burst, Jobs) | ✅ Complete |

### Priority 1 — Camera Pipeline
| ID | Task | Status |
|----|------|--------|
| CAM-1 | FLIR Spinnaker SDK integration (`SpinnakerCameraCapture.cs`) | ✅ Complete |
| CAM-2 | AprilTag detection (`AprilTagDetector.cs`) | ✅ Complete |
| CAM-3 | Camera calibration tool (`CameraCalibrationEditor.cs`) | ✅ Tool built — run calibration to generate profile |
| CAM-4 | Ground plane calibration (`GroundPlaneCalibrator.cs`) | ✅ Tool built — calibration run, accuracy limited by camera calibration quality |
| CAM-5 | Live KF pipeline integration (`LiveDiscTracker.cs`) | ✅ PoC working — throws detected, KF resolves, sim runs. Open issues below. |

### Priority 2 — Sim Experience
| ID | Task | Status |
|----|------|--------|
| SIM-1 | Flight characteristics UI panel (UI Toolkit) | ✅ Complete |
| SIM-2 | Practice range scene with distance markers | ✅ Complete |
| SIM-3 | Follow-flight camera (Cinemachine) | ✅ Complete |
| SIM-4 | Disc database expansion (`DiscParamsImporter.cs` Editor tool) | ✅ Complete |

### Priority 3 — Polish & QoL
| ID | Task | Status |
|----|------|--------|
| POL-0 | Disc trail customization (live style + per-type keep-all colors) | ✅ Complete |
| POL-1 | Shot shape preview line (ghost trajectory) | ✅ Complete |
| POL-2 | Landing distance readout + animated marker | ✅ Complete |
| POL-2.5 | Distance marker polish (smaller text, outline stroke, ground lines) | ✅ Complete |
| POL-3 | Mini-map (orthographic top-down overlay) | ✅ Complete |
| POL-4 | Throw result statistics panel | 🔲 Pending |
| POL-5 | Wind indicator HUD — windsock style + gust indicator | 🔲 Pending |
| POL-6 | Practice range modes + leaderboard/scoring | 🔲 Pending |
| POL-7 | Replay system | 🔲 Pending |
| POL-8 | Multiple camera angles (basket cam, tee cam, auto-cut) | 🔲 Pending |
| POL-9 | Course holes / pin placement ScriptableObjects (stretch) | 🔲 Pending |
| POL-10 | Disc render polish (visual appearance of the disc itself) | 🔶 Mesh + colour + foil stamp done — spin anim pending |
| POL-11 | Practice range visual improvements (environment/scenery polish) | 🔲 Pending |
| POL-12 | Collision feedback — counter-force/torque handoff from scenery into flight model | 🔲 Pending |

---

## Architecture Decisions (Post-Port)

### INF-2: URP Migration Decision
Built-in render pipeline was fine for the port phase. Now that scene and material work begins,
migrating to URP first prevents double-conversion cost. All new materials/shaders use URP from now on.
**After adding URP to manifest.json, manually in the Unity Editor:**
1. `Edit > Project Settings > Graphics > Scriptable Render Pipeline Settings` → create + assign a URP Asset
2. Run `Edit > Rendering > Render Pipeline Converter` → upgrade existing Built-in materials
3. Confirm DiscDebug scene renders with no pink materials before proceeding to SIM-2

### INF-6: Package Versions for Unity 6000.4
- `com.unity.render-pipelines.universal`: `17.4.0` (installed via Package Manager UI, not manifest)
- `com.unity.cinemachine`: `3.1.6`
- `com.unity.burst`: `1.8.17`
- `com.unity.jobs`: removed (version conflict — not needed)

### INF-1: DiscVisionDeluxe.asmdef
Before this existed, all `DiscVisionDeluxe/` scripts compiled into `Assembly-CSharp` by default.
Adding a named asmdef is required before any sub-module (Camera, UI) can reference types like
`KFMeasurement`, `DiscSimulator`, etc. from a sibling asmdef.

### CAM-1: Camera Hardware — BFS-U3-04S2C-CS (Teledyne FLIR Blackfly S)

| Spec | Value |
|------|-------|
| Sensor | Sony IMX287, 1/2.9" CMOS, global shutter |
| Resolution | 720 × 540 (0.4 MP) |
| Max FPS — Mono8 | **290 FPS** (ISP on — demosaics color Bayer → mono) |
| Max FPS — BayerRG8 | **522 FPS** (ISP off — raw Bayer, requires software debayer) |
| Max FPS — at 320×240 ROI | ~997–999 FPS |
| Exposure range | 4 µs – 30 000 000 µs |
| Gain range | 0 – 47.99 dB |
| Interface | USB 3.1 Gen 1 (USB3 Vision v1.0) |
| Pixel formats | BayerRG8/10/12, Mono8/10/12/16, RGB8, BGR8, YCbCr variants |
| Hardware trigger | Yes — Line0 (opto-isolated input), Line1 (opto-isolated output) |
| Lens mount | CS-mount |
| On-board buffer | 240 MB |

**Global shutter** means all pixels expose simultaneously — no rolling shutter distortion on a fast spinning disc. This camera is well suited for disc detection.

**It is a color camera.** The "C" in BFS-U3-04S2**C**-CS means color. The sensor outputs a Bayer pattern. Requesting `PixelFormat = Mono8` causes the on-camera ISP to demosaic and convert — this caps frame rate at 290 FPS. Requesting `BayerRG8` bypasses the ISP (522 FPS) but requires a `CvInvoke.CvtColor(BayerRG → Gray)` step before passing pixels to `AprilTagDetector`.

**Recommended Inspector settings for disc detection:**

| Field | Value | Notes |
|-------|-------|-------|
| `Target Fps` | **290** | Mono8 max. Was incorrectly defaulted to 60. |
| `Exposure Us` | **0** | Leave on auto, controlled by AutoExposureMaxUs |
| `Auto Exposure Max Us` | **1000** | Caps auto-exposure at 1 ms. Auto without a cap can settle on 10–30 ms in dim conditions, causing severe motion blur on a spinning disc. Raise toward 2000–4000 µs if the image is too dark. |
| `Gain Db` | **0** | Auto gain |

**If 522 FPS is needed:** Change `SetEnum(hNodeMap, "PixelFormat", "BayerRG8")` in `InitCamera()`, then add `CvInvoke.CvtColor(mat, grayMat, ColorConversion.BayerRG2Gray)` in `CaptureLoop()` before calling `AprilTagDetector.Detect()`. The byte array stays the same size (720×540×1). For most disc detection cases 290 FPS is sufficient.

### CAM-1: Spinnaker — Use P/Invoke against SpinnakerC_v140, NOT SpinnakerNET

**SpinnakerNET_v140.dll is C++/CLI mixed-mode** — Unity cannot load it as a managed assembly
regardless of how it is configured in the Inspector. It will always show Type: Native and CS0246
errors will persist. Do not attempt to reference SpinnakerNET from any asmdef.

**Correct approach:** P/Invoke against `SpinnakerC_v140.dll` (pure C interface).
All bindings live in `SpinnakerCAPI.cs` (internal static class, `DllImport("SpinnakerC_v140")`).
`SpinnakerCameraCapture.cs` uses only types from that file — no SpinnakerNET usings anywhere.

**DLL Setup — copy ALL of these to `Assets/Plugins/Spinnaker/`:**

All DLLs must be set: Native, x86 UNCHECKED, Editor ✅, Standalone ✅, CPU x64.

| DLL | Source |
|-----|--------|
| `SpinnakerC_v140.dll` | `C:\Program Files\FLIR Systems\Spinnaker\bin64\vs2015\` |
| `Spinnaker_v140.dll` | same |
| `SpinnakerNET_v140.dll` | same — present for completeness, not used in code |
| `GCBase_MD_VC140_v3_0.dll` | same |
| `GenApi_MD_VC140_v3_0.dll` | same |
| `log4cpp_MD_VC140_v3_0.dll` | same |
| `Log_MD_VC140_v3_0.dll` | same — separate from log4cpp, both required |
| `MathParser_MD_VC140_v3_0.dll` | same |
| `NodeMapData_MD_VC140_v3_0.dll` | same |
| `XMLParser_MD_VC140_v3_0.dll` | same |
| `libiomp5md.dll` | same — Intel OpenMP runtime, Spinnaker ships its own copy |
| `vcomp140.dll` | `C:\Windows\System32\` — VC++ 2015 OpenMP; NOT in Spinnaker bin64 |

**How missing DLLs were found:** Used [Dependencies](https://github.com/lucasg/Dependencies) tool.
Drag the DLL in, look at tree — any entry from `C:\Program Files\FLIR Systems\Spinnaker\` or
outside `C:\Windows\` that isn't already in the project needs to be copied over.
Process Monitor (Sysinternals) with filters `Process=Unity.exe`, `Result=NAME NOT FOUND`,
`Path contains Spinnaker` was also used to confirm which DLLs were missing at runtime.

**After adding or changing any native DLL: fully close and reopen Unity.**
Native plugins are loaded at Editor startup — re-entering Play mode is not enough.
A full PC reboot was required once during initial setup to seat the Spinnaker kernel filter driver.

**spinImageConvert does not exist in Spinnaker 3.2 C API.**
Instead, configure the camera to output Mono8 directly via the GenICam node map:
`SetEnum(hNodeMap, "PixelFormat", "Mono8")` in `InitCamera()`. No runtime conversion needed.

**TextureFormat.R8 displays as teal/cyan in Unity 6 URP**, not as grayscale.
Preview texture uses `TextureFormat.RGB24` with manual byte expansion (each Mono8 byte → R=G=B).
The raw `byte[]` passed to the AprilTag detector stays as single-channel Mono8 — do not expand there.

**`DiscVisionDeluxe.Camera.asmdef` refs:** `DiscVisionDeluxe`, `Unity.Cinemachine`, `DfisX.Unity`,
`DfisX.Runtime`, `Unity.Mathematics`, `Unity.Collections`. No precompiledReferences needed.
Include platforms: Editor + WindowsStandalone64.

### CAM-2: Emgu CV instead of OpenCVForUnity

**OpenCVForUnity is paid (~$95 Asset Store) — Emgu CV 4.12.0 is free (NuGet, Apache 2.0).**
Both wrap the same underlying OpenCV 4.x with the Aruco module. Use Emgu CV.

**Installation via NuGetForUnity:**
1. Install NuGetForUnity package into Unity (Window → NuGet → Install)
2. Search for `Emgu.CV` and install — NuGetForUnity automatically resolves the managed dependency chain
   (System.Text.Json, System.IO.Pipelines, etc.) and places DLLs in `Assets/Packages/`
3. NuGet will install the latest available version — **4.12.0 was installed** (not 4.9.0 as originally planned).
   The version is shown in `Assets/Packages/Emgu.CV.X.X.X.XXXX/`

**cvextern.dll — the native OpenCV runtime:**
- NuGetForUnity only installs the managed `Emgu.CV.dll` wrapper — it does NOT include `cvextern.dll`
- `cvextern.dll` must be obtained separately from the Emgu CV GitHub releases (~80MB)
- Copy to `Assets/Plugins/EmguCV/cvextern.dll`
- The `.meta` file Unity auto-generates may be nearly empty (no platform settings) — write it explicitly:
  Native, x86 UNCHECKED, Editor ✅ (CPU x86_64, OS Windows), Win64 ✅ (CPU x86_64), all others off
- After placing the DLL and fixing the meta, **fully restart Unity** — the ArucoInvoke static initialiser
  will throw TypeInitializationException until Unity reloads native plugins from disk

**libusb-1.0.dll — hidden dependency of cvextern.dll:**
- `cvextern.dll` links against `libusb-1.0.dll` which is not shipped with Emgu CV
- Symptom: TypeInitializationException for ArucoInvoke even after cvextern.dll is correctly configured
- Find it via the Dependencies tool (drag cvextern.dll in, look for red entries)
- Obtain from libusb GitHub releases: `VS2019/MS64/dll/libusb-1.0.dll`
- Copy to `Assets/Plugins/EmguCV/` with the same Native plugin settings as cvextern.dll
- Full Unity restart required again

**Emgu CV 4.12 API differences from older versions:**

*Dictionary:*
- `Dictionary.PredefinedDictionaryName.AprilTag36H11` does not exist (enum name varies by version)
- `Dictionary.PredefinedDictionaryName.DictAprilTag36H11` also does not exist
- Use the raw integer cast: `new Dictionary((Dictionary.PredefinedDictionaryName)20)`
- `DICT_APRILTAG_36H11 = 20` is a fixed OpenCV constant that does not change across versions

*ArucoDetector class:*
- `ArucoDetector` class does not exist in this API path
- Use the static method: `ArucoInvoke.DetectMarkers(mat, dictionary, corners, ids, detectorParams, rejected)`

*DetectorParameters:*
- In 4.12, `DetectorParameters` is a **struct with PascalCase public fields** and a static `GetDefault()` factory
- Old camelCase field names (`markerBorderBits`, `adaptiveThreshWinSizeMin`) do NOT exist → CS0117
- `new DetectorParameters()` zero-initialises all fields; OpenCV asserts `markerBorderBits > 0` at runtime
- **Correct usage:** `DetectorParameters.GetDefault()` — returns struct with all OpenCV defaults set (MarkerBorderBits=1 etc.)
- Individual fields can be set after GetDefault() using PascalCase: `p.MarkerBorderBits = 1`
- To discover the actual API, examine `Assets/Packages/Emgu.CV.X.X.X/lib/netstandard2.0/Emgu.CV.xml`
  — it lists all `F:`, `P:`, `M:` members for every class

**SpinnakerCAPI visibility:**
`SpinnakerCAPI` must be `public static class` (not `internal`) so the Editor assembly
(`DiscVisionDeluxe.Camera.Editor.asmdef`) can reference it via `using static`.

**Working detection output:**
```
[AprilTag] ID=0  corners=(387,270) hasPose=False
```
`hasPose=False` is expected until `SetCalibration()` is called on the detector.

**Tag corners → 6-DOF pose:** `solvePnP` → `rvec`, `tvec`. Angular extraction matches `CsvLogReader.ParseLine()`:
`hyzer = asin(R[1,2])`, `pitch = asin(R[0,2])`, `spin = atan2(R[0,1], R[0,0])`

### CAM-3: Camera Calibration Tool

**Files:**
- `Camera/CameraCalibration.cs` — runtime data class; holds fx/fy/cx/cy + distortion coeffs; LoadFromJson/SaveToJson
- `Camera/Editor/CameraCalibrationEditor.cs` — EditorWindow; runs entirely in Edit mode (no Play mode needed)
- `Camera/Editor/DiscVisionDeluxe.Camera.Editor.asmdef` — Editor-only, refs DiscVisionDeluxe.Camera

**Usage:**
1. Open **DiscVisionDeluxe → Camera Calibration Tool** from menu
2. Connect camera (no Play mode required — uses P/Invoke directly)
3. Hold a checkerboard target in front of the camera; green/red dots appear when corners are detected
4. Capture 15-20 frames covering varied board angles, tilts, and positions across the frame
5. Run Calibration — RMS < 1.0 is good, < 0.5 is excellent
6. Enter a profile name, Save → writes to `StreamingAssets/CameraCalibration/{name}.json`
7. Load profile at runtime: `CameraCalibration.LoadProfile("name")` → call `aprilTagDetector.SetCalibration(...)`

**Checkerboard target:**
- Standard OpenCV checkerboard — `FindChessboardCorners` + `CornerSubPix` refinement
- Default: 9×6 **inner corners** = 10×7 grid of squares
- Set Inner Corners (cols/rows) to match your actual board exactly
- Measure printed square size with calipers — printer scaling affects calibration accuracy
- Print flat, mount on rigid backing (foam board etc.) — any warp degrades accuracy

**Calibration files live in:** `Assets/StreamingAssets/CameraCalibration/`
Multiple profiles supported — one per camera / lens / resolution combination.

### CAM-4: Ground Plane Calibration

**Files:**
- `Camera/GroundPlaneCalibration.cs` — serializable data class; holds `worldToCamera[16]`, `cameraToWorld[16]`, `cameraHeightM`; LoadProfile/SaveToJson
- `Camera/GroundPlaneCalibrator.cs` — MonoBehaviour; subscribes to `onTagsDetected`; averages N rvec+tvec frames; computes R, R^T, translation; saves JSON

**World frame convention (tag frame):**
- Origin: centre of reference AprilTag placed flat on the ground at the tee
- X axis: tag's printed X direction — **must point downrange (flight direction)**
- Y axis: 90° right of X on the ground plane
- Z axis: up (perpendicular to ground, toward camera)

**Usage:**
1. Assign `SpinnakerCameraCapture` and set `cameraCalibrationProfile` (e.g. `"camera_calibration"`) so intrinsics load on Start
2. Set `referenceTagId` to match the tag on the ground (check `[AprilTag] ID=X` in Console)
3. Enter Play mode
4. Right-click component → **Start Ground Calibration** — keep the ground tag fully visible until `_framesCollected` reaches `averageFrames` (default 30)
5. Calibration saves automatically to `StreamingAssets/CameraCalibration/ground_plane.json`
6. `[GroundCalib] Done! Height=X.XXXm` confirms success

**Math:** `worldToCamera = [R | t]`, `cameraToWorld = [R^T | -R^T*t]`. `cameraHeightM = tz` (Z component of camera position in world/tag frame). Transform from camera space to world space: `p_world = cameraToWorld * p_camera` (homogeneous multiply, row-major 4×4).

**Ground tag orientation:** The front (printed) face must be visible to the camera. In OpenCV/ArUco convention, the tag's Z axis points out of the printed face. When the tag is flat on the ground face-up, Z points toward the camera = world up. If the face is down, Z points into the ground and the calibration Z axis will be inverted.

**Known issue — Z axis inversion:** With the current camera calibration quality (RMS 3.4274), the computed ground plane rotation matrix has errors that result in the world Z axis being inverted (negative Z = disc above ground). `LiveDiscTracker.FireComplete()` has a workaround: `if (init.linearPositionM.z <= 0f) init.linearPositionM.z = 1.0f`. This will be resolved when camera calibration quality is improved. See Open Issues.

### CAM-4/5: Ground Plane Integration in LiveDiscTracker

`LiveDiscTracker` loads the ground plane profile on `OnEnable()` and applies `cameraToWorld` in `BuildMeasurement()` when available:
- Position: `p_world = cameraToWorld * p_camera` (direct KF measurement — world frame IS the KF frame)
- Rotation: `R_world = R_cam_to_world * R_disc_cam` (3×3 upper-left of cameraToWorld × Rodrigues of rvec)
- Fallback when no profile loaded: raw camera-space mapping (`kfX=camZ, kfY=camX, kfZ=-camY`) — only correct for a horizontally forward-facing camera

Inspector field: `Ground Plane Profile` (string, default `"ground_plane"`). Leave empty to use fallback.

### CAM-5: Live Disc Tracking Pipeline — Architecture

**File:** `Camera/LiveDiscTracker.cs` — MonoBehaviour, wired to `SpinnakerCameraCapture.onTagsDetected`

**Flow:**
1. `onTagsDetected` fires on main thread → `OnTagsDetected()` → `BuildMeasurement()` → `_kf.AddMeasurement()`
2. `Update()` drives `_kf.Step()` at `kfParams.predDtS` intervals using `cameraCapture.ElapsedCaptureNs` for timing
3. Detection timeout: if no detection seen for `detectionTimeoutMs` (default 2000ms), calls `_kf.SignalDetectionLost()`
4. When `_kf.CurrentStage == Complete` → `FireComplete()` → validates speed → invokes `onThrowComplete(DiscInitState)`
5. Auto-resets to `Idle` after each throw (complete or discarded) — ready for next throw immediately

**Clock synchronisation (critical):** ALL timestamps must come from `SpinnakerCameraCapture._captureClock` (a single `Stopwatch` started in `StartCapture()`). `LiveDiscTracker` reads `cameraCapture.ElapsedCaptureNs` for current time and `cameraCapture.LastDetectionTimestampNs` for each detection's capture time. Mixing clocks causes immediate detection timeouts.

**KF params for live camera** (set in Inspector, more relaxed than CSV defaults):
- `primeMaxEntries = 3` — prime queue uses last 3 frames only (CSV uses 8)
- `primeCount = 3` — need 3 frames to prime
- `primeMinVar = 0.5` — lowered to 0.05 in Inspector for slow/establish-then-move testing

**Speed guards in `FireComplete()`:** Discards throws with speed < 5 kph or > 400 kph, or NaN position/velocity. Also clamps `linearPositionM.z` to 1.0m when negative (ground plane calibration workaround — see CAM-4 issue).

**Wiring in scene:**
- `Camera Capture` → SpinnakerCameraCapture component
- `Disc Tag Id` → the AprilTag ID physically attached to the disc
- `Ground Plane Profile` → `"ground_plane"` (or leave empty for raw camera-space fallback)
- `On Throw Complete` → `DiscVisualizer.LaunchDfisX`

### CAM-5: Detection Thread Pipeline

`SpinnakerCameraCapture` runs two background threads:
- **CaptureLoop** (`SpinnakerCapture` thread): grabs frames at full 290fps from camera, enqueues to `_pendingDetectQueue` (max 60 frames)
- **DetectionLoop** (`AprilTagDetection` thread): dequeues frames, runs AprilTag detection at its own pace (~25-30fps effective rate)

The bounded queue (60 frames ≈ 200ms at 290fps) ensures all frames containing the disc are available for detection even if the disc passes through the frame faster than the detection thread processes. Detection results are enqueued to `_detectionQueue` (a separate `ConcurrentQueue<DetectionFrame>`) with the original capture timestamp.

`Update()` on the main thread drains `_detectionQueue`, sets `LastDetectionTimestampNs`, and fires `onTagsDetected`.

**Attempted: BayerRG8 at 522fps** — caused rapid camera disconnect/reconnect (likely USB3 bandwidth saturation at 720×540×522fps ≈ 200MB/s). Reverted to Mono8 at 290fps. To revisit: would also need `CvtColor(BayerRG → Gray)` before detection, and must verify USB host can sustain the bandwidth.

### CAM-5: solvePnP Result Extraction Bug (Fixed)

**Symptom:** All tvec values were (0.000, 0.000, 0.000) despite tags being detected.

**Root cause:** `rvecMat.CopyTo(new Matrix<double>(rvec))` in `AprilTagDetector.Detect()`. `new Matrix<double>(double[])` creates a **1×N row vector**, but `solvePnP` outputs a **3×1 column vector**. The dimension mismatch causes `CopyTo` to silently fail, leaving the array at zero.

**Fix:** Use `Matrix<double>(3, 1)` directly as the solvePnP output, then read with indexer:
```csharp
using var rvecOut = new Matrix<double>(3, 1);
using var tvecOut = new Matrix<double>(3, 1);
CvInvoke.SolvePnP(..., rvecOut, tvecOut, ...);
rvec = new double[] { rvecOut[0,0], rvecOut[1,0], rvecOut[2,0] };
tvec = new double[] { tvecOut[0,0], tvecOut[1,0], tvecOut[2,0] };
```

### CAM-5: Thread Safety Contract
- Background thread: camera capture → AprilTag detect → push to `ConcurrentQueue<DetectionFrame>`
- Main thread `Update()`: drain queue → `onTagsDetected.Invoke()` → `LiveDiscTracker.OnTagsDetected()` → `_kf.AddMeasurement()`
- All UnityEvent callbacks (onThrowComplete, etc.) always fire on the main thread
- NEVER call any Unity API from the background thread

---

## Open Issues — CAM Pipeline (As of Session 12)

### ISSUE-1: Camera Calibration Quality (RMS 3.4274 — Needs Redo)
**Severity:** High — affects all pose estimation accuracy

The camera calibration RMS of 3.4274 pixels is too high. Target is < 1.0 (good), < 0.5 (excellent). This error propagates into every solvePnP call and is the primary cause of the ground plane Z axis inversion (see ISSUE-2).

**Hypothesis (user):** Better lighting will significantly improve calibration quality. The checkerboard detection relies on sharp, high-contrast corner edges. In dim or uneven lighting, `CornerSubPix` refinement produces noisy sub-pixel positions, which corrupts the calibration solve.

**To fix:**
1. Recalibrate with good, even lighting (no harsh shadows on checkerboard)
2. Cover more of the frame with the checkerboard — especially corners and edges (distortion is worst there)
3. Use varied board angles (tilted, rotated, not just flat-on)
4. Aim for 20-30 frames with good spatial coverage
5. Measure printed square size with calipers — printer scaling affects calibration

**Current calibration values** (3.4274 RMS, for reference):
`fx=321.0, fy=320.6, cx=353.6, cy=281.8` — saved at `StreamingAssets/CameraCalibration/camera_calibration.json`

### ISSUE-2: World Z Axis Inversion in Ground Plane Calibration
**Severity:** Medium — sim runs via workaround, but Z velocity is wrong

After applying `cameraToWorld`, disc positions consistently show `kfZ ≈ -1.0 to -1.5m` when the disc is held at ~1m height. Expected: `kfZ ≈ +1.0m`. The Z axis is inverted.

**Root cause (likely):** Poor camera intrinsics (ISSUE-1) cause solvePnP to compute an inaccurate rotation matrix for the ground tag. The resulting `cameraToWorld` has the Z row pointing in the wrong direction. Ground tag face orientation (front face up, Z pointing toward camera) is correct in principle — the issue is calibration quality, not tag placement.

**Workaround in place:** `LiveDiscTracker.FireComplete()` clamps `init.linearPositionM.z = 1.0f` when negative. This allows the sim to run but Z velocity (e.g. `Vel.z = -1.63 m/s`) is still affected.

**To fix:** Redo camera calibration (ISSUE-1) → redo ground plane calibration → Z should be correct.

### ISSUE-3: Real Throws Not Detected — Requires "Establish Then Move"
**Severity:** Medium — PoC works with establishment, not with natural throws

Currently the disc must be held still in the camera frame for ~0.5s before moving for the KF to acquire and track it. A natural throw from outside the camera frame is not reliably caught.

**Root cause:** The KF priming requires `primeCount=3` measurements with sufficient velocity variance (`primeMinVar`). A disc entering the frame from outside at full speed gets 3-6 detections in the bounded queue, but if priming variance is not met (consistent fast motion looks like low variance to the KF), no ideal state is produced.

**Hypothesis (user):** Higher frame rate (up to 522fps BayerRG8) would provide more detection opportunities per throw, increasing the chance of the disc being detected in multiple frames with sufficient variance. **This requires solving the USB bandwidth issue** (BayerRG8 caused disconnect at 522fps — may need USB3 host controller check, or reduced resolution ROI).

**Camera placement note:** Camera is currently mounted overhead at ~45° looking downrange. For a natural throw from behind the camera, the disc enters the frame from the thrower's side and travels across — this is actually reasonable geometry. The bottleneck is detection rate (~25-30fps effective with current ArUco processing time) vs. throw duration in frame.

**Possible approaches to try:**
1. Reduce image resolution before detection (e.g. `Imgproc.Resize` to 360×270 before `DetectMarkers`) — would roughly 4× detection rate. Requires halving intrinsics (fx/2, fy/2, cx/2, cy/2) passed to solvePnP, OR scaling detected corners back up before solvePnP.
2. Lower `primeMinVar` further in Inspector (currently 0.5, try 0.05–0.1)
3. Use ROI mode on the camera to capture a smaller resolution at higher fps without USB bandwidth issues

### ISSUE-4: Detection Timeout vs. Queue Drain Time
**Severity:** Low — currently mitigated by 2000ms timeout

At 30ms detection per frame and 60-frame queue capacity, worst-case queue drain time is 1800ms. Detection timeout is set to 2000ms (200ms margin). If detection is slower (heavy CPU load, larger image), the queue may not drain before timeout fires.

**Inspector note:** `Detection Timeout Ms` value is saved per-scene. Existing scenes will retain old 500ms value — manually update to 2000 in Inspector for the LiveCapture scene.

---

### SIM-1: UI Toolkit (UIElements) for all new panels
`com.unity.modules.uielements` is already in the manifest. UI Toolkit (UXML/USS) is preferred over
uGUI Canvas for new panels — supports data binding, UXML/USS separation, better for theming.
Keep OnGUI only in `DiscThrowDebugger` (existing debug tool, no change required).

### SIM-3: Cinemachine 3 (CM3 API)
Unity 6 uses Cinemachine 3 (`CinemachineCamera`, not `CinemachineVirtualCamera` which is CM2).
`FollowFlightCamera.cs` must use CM3 API. Do not use deprecated CM2 types.

**CM3 Camera Switching:** Use `vcam.Priority = int` for switching. Do NOT remove auto-switch-on-throw
logic — it was removed because it overrode the user's selected camera on every launch.
`VCam_Follow` is manually positioned every frame (no Cinemachine procedural components — Position/Rotation
Control = None in Inspector). `VCam_Side` and `VCam_Overhead` use CinemachineFollow + RotationComposer
with Tracking Target = disc assigned in Inspector. `VCam_Overview` is fully static.

**Disc axis:** Disc flies along Unity **+X**. All camera offsets, marker positions, and scene layout
use +X as the flight direction. VCam_Follow offset: `(-followDistance, followHeight, 0)`.
VCam_Side offset: `(0, 4, 20)` (perpendicular). VCam_Overhead offset: `(0, 35, 0)`.

### SIM-2: Distance Markers
Markers spawn along +X axis at `new Vector3(distM, groundOffsetY, sideOffsetM)`.
Labels face camera (disc flies +X, camera behind looks +X) with `Quaternion.Euler(0, 90, 0)`.
`groundOffsetY` Inspector field (default 0) lets you raise markers above terrain surface.

### DiscVisualizer: Rigidbody Handoff
At landing, `HandoffToRigidbody` syncs `discRigidbody.transform.position` to `discTransform.position`
before going non-kinematic — critical if discTransform is a child of the Rigidbody object.
Also sets `CollisionDetectionMode.Continuous` to prevent disc tunneling through the ground plane.
Disc GameObject must have a Collider component for landing physics to work.

### POL-1: Synchronous SimulateThrow for Preview
`DiscFlightSimulator.SimulateThrow()` is already synchronous and runs in ~1ms.
Use it for the ghost trajectory preview — sample `discStateArray` every 50 sim steps (~200 points).
Debounce the preview update to max 5Hz to avoid per-frame recomputes while sliders are dragged.

### POL-0: Disc Trail Customization
`DiscVisualizer` has two trail customization systems:
- **Live throw style**: `liveThrowTrailColor` / `liveThrowTrailWidth` — applied when throw originates from `LaunchDfisXFromLive()`.
- **Per-disc-type colors** (keepAllThrows mode): `List<DiscTypeTrailStyle>` — each entry maps a `DiscIndex` to a color and width.
- **Material fix**: `dfisxTrajectoryLine` must have a vertex-color-capable material. `Awake()` auto-creates `Universal Render Pipeline/Particles/Unlit` if `sharedMaterial == null`. Without this, Unity renders the line pink regardless of `startColor`.

### POL-1: ShotPreviewLine
`ShotPreviewLine` (Visualization/) runs a full synchronous `SimulateThrow()` on every `RequestUpdate()` call, throttled to `maxUpdatesPerSecond` (default 5Hz) via `Time.unscaledTime`.
- Call `RequestUpdate(ThrowParameters, DiscEnvironment, AeroDebugSettings)` from every slider callback.
- Call `HidePreview()` immediately before `LaunchDfisX` fires so the ghost doesn't linger over the real trail.
- Seed with initial values in `ThrowParameterPanelController.Start()` so the ghost is visible before the first slider drag.
- Uses same `directModel` path as live launch — selected disc is reflected in the preview shape.

### POL-2: LandingMarker
`LandingMarker` (Visualization/) is a self-pooling world-space TMP label. No prefab needed.
- `LandingMarker.Spawn(worldPos, distanceM)` — call from `DiscVisualizer.HandleSimFinished`.
- Floats upward at `FloatSpeedMps` (0.6 m/s default), holds full opacity for first 20% of `FadeDurationS` (5s), then fades. Billboards to `Camera.main` each frame.
- `DiscVisualizer` inspector fields: `showLandingMarker` toggle, `landingMarkerHeightM` (default 1.5m) spawn height above landing point.

### POL-2.5: Distance Marker Polish
`DistanceMarkerSpawner` changes:
- Labels smaller (`labelFontSize` default 1.0, scale 0.008) with TMP outline (`labelOutlineWidth` default 0.25, black) — uses per-instance `fontMaterial` so shared font asset is not modified.
- Ground lines: `LineRenderer` drawn at `groundOffsetY + 0.01` running along Z axis (perpendicular to flight). Width `groundLineWidth` (0.04m), color `groundLineColor` (white 25% alpha). Uses same `URP/Particles/Unlit` vertex-color material as DfisX trail.
- All parameters Inspector-tunable; call `Rebuild()` at runtime to apply changes.

### POL-3: MiniMap
`MiniMap` (UI/) is a fully software-drawn 2D overhead view — no extra camera.
- Draws directly onto a `Texture2D` every frame using Bresenham lines. No 3D camera or RenderTexture.
- **Coordinate mapping**: world X (flight) → pixel Y (tee at bottom, far end at top). World Z (lateral) → pixel X (flipped to match 3D view direction).
- **Grid**: horizontal lines at `gridIntervalFt` intervals (default 50ft), converted to metres internally.
- **Labels**: TMP overlaid on the map RectTransform at `labelIntervalFt` intervals (default 50ft). No unit suffix — self-explanatory.
- **Trail**: reads `dfisxTrajectoryLine` positions directly; draws at `trailThicknessPx` (default 3px). Also draws archived trails from `discVisualizer.GetArchivedTrails()`.
- **Disc dot**: small filled circle at disc's current world position.
- **Auto-find**: if `discVisualizer` not wired in Inspector, `FindAnyObjectByType<DiscVisualizer>()` runs in `Start()`.
- `DiscVisionDeluxe.UI.asmdef` requires `Unity.TextMeshPro` reference — was missing, caused CS0246.

### Camera Dropdown Fix
`CameraMode` enum order (`Overview=0, Follow=1, Side=2, Overhead=3`) does not match the UI choices order (`Follow=0, Side=1, Overhead=2, Overview=3`). Always use `ModeToIndex()`/`IndexToMode()` in `ThrowParameterPanelController` — never cast directly. Also: `dropdown.index = x` does not fire `RegisterValueChangedCallback`, so `SetMode()` must be called explicitly on startup.

### Disc Selection — How Per-Disc Aero Works
All 11 disc models are physically different `DiscModel` assets in `Assets/Resources/DiscModels/`. The aero differences come entirely from their **physical dimensions**: `mass`, `radius`, `rimWidth`, `thickness`, `rimDepth`, `rimCamberHeight`, `domeHeight`, `rimCamberShape`. These feed directly into area and moment calculations in `Daero.cs`.

**The Aero Debug sliders are NOT per-disc parameters.** They are global physics-model tuning constants that apply identically to every disc:

| Slider | Role |
|--------|------|
| `cdEdge` | Edge form drag coefficient (default 0.6) |
| `clCavity` | Bernoulli lift from cavity lip (default 45.0) |
| `clCamber` | Bernoulli lift from dome (default 1.0) |
| `cavityEdgeExposedAreaFactor` | Fraction of cavity lip that is aerodynamically active (default 1.0) |
| `pitchingMomentCavityLiftOffset` | Cavity lift moment arm as fraction of diameter (affects turn/fade balance, default 0.042) |
| `pitchingMomentCamberLiftOffset` | Dome lift moment arm as fraction of diameter (affects turn/fade balance, default 0.15) |

Syncing these sliders to the selected disc would be incorrect — they tune the simulation model, not the disc shape.

**Disc selection was broken (now fixed):** `ThrowParameterPanelController` previously hardcoded `DiscIndex.DRIVER` regardless of dropdown. Fix: `DiscInitState` and `ThrowParameters` both gained a `[NonSerialized] directModel` field. The panel sets it to `discModelLibrary.discs[_discIndex]`. `DiscFlightSimulator.NewThrow()` uses `p.directModel ?? FindByDiscIndex(p.discIndex)` — direct reference takes priority. `DiscVisualizer.LaunchDfisX` threads `directModel` through so it is never lost.

### MAX_STEPS Expanded
`DiscFlightSimulator.MAX_STEPS` raised from 10,000 to 30,000 (~30s at 1kHz). The original 10s cutoff was too short for some understable drivers executing long hyzer-flip arcs.

### SIM-4: Disc Database Expansion

**CSV source:** `Assets/Resources/DiscModels/disc_params_pdga_final.csv` — 1,272 discs from real PDGA-measured data.

**Columns:** `mold_name, manufacturer, disc_type, stability, rim_camber_shape, mass_kg, radius_m, rim_width_m, thickness_m, rim_depth_m, rim_camber_h_m, dome_height_m`

**rim_camber_shape values:** `Flat`, `Concave`, `Convex`, `NONE` — `NONE` is preserved as-is (unknown shape, to be populated later). The `DiscModel.ToBlittable()` treats anything not `Concave`/`Convex` as Flat (int=0), so NONE entries degrade gracefully.

**Import workflow:**
1. `DfisX → Import CSV Disc Database` — creates 1,272 `.asset` files under `Assets/Resources/DiscModels/`; uses `CultureInfo.InvariantCulture` for float parsing; skips existing by default.
2. Select the `DiscModelLibrary` asset → `Assets → Create → DfisX → Auto-populate Selected DiscModelLibrary` — fills the 12 named slots (type × stability) and the `discs` list.

**Disc selection UI (ThrowParameterPanelController):**
- `_selectedModel` is the single authoritative field used by all physics builds; set by dropdown OR recent-pill click.
- `RebuildDiscDropdown()` applies four AND filters: type, stability, manufacturer, free-text search (case-insensitive, matches mold name or manufacturer).
- If `_selectedModel` is still in the filtered list after a filter change, the dropdown scrolls to it rather than resetting to index 0.
- Manufacturer filter choices are populated dynamically from `discModelLibrary.discs` (sorted `SortedSet<string>`).
- Filter dropdowns use the inline-label `compact-filter` USS class — same 26px row height as sliders so the panel density doesn't change.

**Recents:**
- `static List<DiscModel> _recentDiscs` (max 5) — static so it survives re-binds within a play session.
- Populated on each Launch via `AddToRecents(_selectedModel)` (deduplicates, newest first).
- Rendered as pill buttons (`recent-btn` USS class) in `recents-container`; active disc gets `recent-btn--active` highlight.
- Clicking a pill calls `SelectRecent()`: sets `_selectedModel` directly, syncs main dropdown if disc is visible in current filter.

### POL-7: Replay via discStateArray Serialization
`ThrowContainer.discStateArray` is a `NativeArray<DiscState>` (blittable).
Serialize via `.ToArray()` → raw bytes → file. On load: read bytes → `NativeArray.CopyFrom()`.
This is lossless and fast — no JSON overhead for potentially thousands of state steps.

### POL-5: Wind Indicator — Design Notes
The wind indicator HUD should look like a **windsock** rather than a flat arrow.
- The sock body droops/extends in the direction the wind is blowing; length and angle reflect wind speed.
- Add a separate **gust indicator** driven by `DiscEnvironment.gustFactor` (or equivalent gust amplitude field):
  when a gust fires, animate the sock billowing/snapping — e.g., a brief elongation + ripple effect — then return to the mean state.
- Both elements should live in a corner HUD overlay (UI Toolkit or World-Space canvas).

### POL-10: Disc Render Polish — Implementation Notes
**Status: first pass complete.** Procedural mesh + URP Lit material working in-game.

**Files:**
- `Visualization/DiscMeshBuilder.cs` — static class; `Build(DiscModel, segments, domeSegs)` generates a surface-of-revolution mesh.
- `Visualization/DiscVisualController.cs` — `MonoBehaviour` (`RequireComponent(MeshFilter, MeshRenderer)`); call `SetDiscModel(DiscModel)` to rebuild mesh + reset scale.
- `DiscVisualizer.cs` — added `discVisualController` Inspector field; both `LaunchDfisX` overloads call `discVisualController?.SetDiscModel(initState.directModel)` before flight.

**Cross-section profile (clockwise in r,y from centre-top):**
1. Dome — cosine quarter-circle from `(0, thickness)` to `(innerR, rimY)` where `rimY = thickness - domeHeight`
2. Rim overhang — ease-in slope from `(innerR, rimY)` down to `(R, edgeTopY)` — this is the key driver shape; NOT flat
3. Tiny outer edge wall — `(R, edgeTopY)` to `(R, camberH)`
4. Lower rim camber — sweeps from `(R, camberH)` inward to `(innerR, 0)` with Concave/Flat/Convex offset
5. Cavity inner wall — `(innerR, 0)` up to `(innerR, rimDepth)`
6. Cavity floor — `(innerR, rimDepth)` to `(0, rimDepth)`

**Key geometry lesson:** the rim top must slope *downward* from flight plate to edge — a flat rim top produces the chunky UFO look. `edgeTopY ≈ camberH + 0.5mm` so the edge is nearly a point.

**Scene setup (DiscDebug.unity — Disc GameObject):**
- Add `DiscVisualController` component; wire to `DiscVisualizer.discVisualController`.
- Set `fallbackModel` to a specific disc asset (e.g. Destroyer) for pre-throw and live-throw display.
- Remove the second `CapsuleCollider` (was two stacked). Keep one with Radius `0.106`, Height `0.020`, Direction Y-Axis, Center `(0, 0.010, 0)`.
- `localScale` is reset to `Vector3.one` by `DiscVisualController` — old `(0.21, 0.02, 0.21)` capsule scale is gone.

**Material:** URP Lit, off-white (`0.95, 0.95, 0.95`), smoothness `0.55`. Created at runtime — no asset on disk.

**Foil stamp — `Visualization/DiscFoilStamp.cs`:**
- Created by `DiscVisualController.Awake()` as a sibling component; call `SetDiscModel()` to rebuild.
- Hierarchy: `StampRoot` (owns position/rotation/scale) → `StampText` (TMP) + `Deco_*` (LineRenderers).
- Font loaded at runtime from Windows OS fonts via `TMP_FontAsset.CreateFontAsset(familyName, styleName)`. Cached statically per font name.
- 15 recipes using the 7 wildest fonts confirmed installed: Gabriola, MV Boli, Ink Free, Segoe Script, Segoe Print, Comic Sans MS, Impact, Lucida Console, Sylfaen, Franklin Gothic Medium — all with bold/italic/extreme tracking combinations.
- Foil colour = complementary hue (h+0.5) to disc body; vertex gradient (bright→dark) + embossed outline.
- 5 decoration presets (LineRenderers, ZTest=Disabled): Classic Brackets, Starburst, Wing Lines, Orbit Ring, Bold Double Arc.
- All elements use `ZTest=Disabled, renderQueue=3000/3001` so they composite onto the dome at any viewing angle without z-fighting.
- Font + decoration preset both seeded from `manufacturer::moldName` hash (offset +7 from colour seed) — deterministic per mold across sessions.

**Known issues / future work:**
- Disc asset `rimWidth` values appear slightly high (Destroyer shipped as `0.0245`, real PDGA value ~`0.021`). All disc assets generated by `DiscParamsImporter.cs` should be audited against PDGA approval sheets.
- Spin animation (UV rotation or normal-map) not yet implemented — spin is only visible via orientation changes from DfisX.
- `DiscVisualController.fallbackModel` is used for live-camera throws (which have no `directModel`) — should eventually resolve the model from `DiscIndex` via `discModelLibrary`.

### POL-11: Practice Range Visual Improvements
Goal: environment feels like a real wooded/open disc golf range rather than a flat grey plane.
- Terrain with gentle elevation variation.
- Treeline or hedge boundary to give depth/scale cues.
- Better fairway ground material (grass texture, URP Lit).
- Sky / lighting pass (HDRI sky, directional light angle matching time-of-day setting).
- Distance sign posts replacing or augmenting the current floating text markers.

### POL-12: Collision Feedback — Force/Torque Handoff into Flight Model
When the disc clips or strikes scenery (trees, terrain, basket pole) the physics engine applies an
impulse to the Rigidbody, but that impulse is currently not fed back into the DfisX aerodynamic
model — the two systems diverge at the moment of contact.

**Required:**
- On `OnCollisionEnter` / `OnCollisionStay` capture the contact normal, impulse magnitude, and contact point.
- Convert to a **counter-force** and **counter-torque** expressed in the DfisX body frame.
- Feed these back into `DiscFlightSimulator` (or inject as an external force/torque delta each step) so the aero model trajectory is corrected rather than overridden.
- Prevents the disc from tunnelling through scenery and then continuing a phantom flight path calculated from pre-collision state.
- Implementation note: `ThrowContainer` or `DiscState` may need an `externalForceDelta` and `externalTorqueDelta` field (blittable `float3`) that the sim reads each integration step.

---

## Key Architecture Rules (Must Not Violate)

1. Every new input pathway (camera, UI sliders, replay) terminates at
   `DiscVisualizer.LaunchDfisX(DiscInitState)`. Never bypass this entrypoint.
2. No Unity API calls from background threads. Thread boundary = `ConcurrentQueue<KFMeasurement>`.
3. Camera pipeline must output DfisX Z-up frame (X forward, Y right, Z up).
   Reference: `CsvLogReader.ParseLine()` R-matrix formulas.
4. New physics-facing structs must be blittable (no managed refs, no string fields).
5. `DiscVisionDeluxe` scripts may reference `DfisX.Unity` and `DfisX.Runtime`. The reverse is forbidden.
6. CHECK this file AND `porting_notes.md` before creating any new `.cs` file.

---

## File Map (All Post-Port Files)

### New asmdefs
| File | Phase | Notes |
|------|-------|-------|
| `Assets/DiscVisionDeluxe/DiscVisionDeluxe.asmdef` | INF-1 | Refs DfisX.Unity, DfisX.Runtime, Unity.Mathematics, Unity.Collections |
| `Assets/DiscVisionDeluxe/Camera/DiscVisionDeluxe.Camera.asmdef` | CAM-1 | Refs DiscVisionDeluxe, Unity.Cinemachine, DfisX.Unity, DfisX.Runtime, Unity.Mathematics, Unity.Collections. No precompiledRefs. Editor + WindowsStandalone64 only. |
| `Assets/DiscVisionDeluxe/UI/DiscVisionDeluxe.UI.asmdef` | SIM-1 | Refs DiscVisionDeluxe, DiscVisionDeluxe.Camera, DfisX.Unity, DfisX.Runtime, Unity.Mathematics, Unity.Collections |

### New C# files
| File | Phase | Notes |
|------|-------|-------|
| `Camera/SpinnakerCAPI.cs` | CAM-1 | Internal static class; all P/Invoke bindings for SpinnakerC_v140.dll |
| `Camera/SpinnakerCameraCapture.cs` | CAM-1 | MonoBehaviour; background thread acquisition; ConcurrentQueue; RGB24 preview |
| `Camera/AprilTagDetector.cs` | CAM-2 | Not MonoBehaviour; Emgu CV 4.12; ArucoInvoke.DetectMarkers; DetectorParameters.GetDefault() |
| `Camera/CameraCalibration.cs` | CAM-3 | Serializable; fx/fy/cx/cy + distCoeffs[5]; LoadFromJson/SaveToJson/LoadProfile |
| `Camera/GroundPlaneCalibration.cs` | CAM-4 | Serializable data class; worldToCamera[16], cameraToWorld[16], cameraHeightM; LoadProfile/SaveToJson |
| `Camera/GroundPlaneCalibrator.cs` | CAM-4 | MonoBehaviour; averages N frames; computes cameraToWorld; saves ground_plane.json |
| `Camera/LiveDiscTracker.cs` | CAM-5 | MonoBehaviour; subscribes to onTagsDetected; drives DiscKalmanFilter; fires onThrowComplete |
| `Camera/FollowFlightCamera.cs` | SIM-3 | CM3 API; modes: FollowBehind/SideView/Overhead/FirstPerson/BasketView |
| `UI/ThrowParameterPanelController.cs` | SIM-1, SIM-4 | Binds UXML sliders; calls LaunchDfisX; AeroDebug section; multi-filter disc selection (type/stability/manufacturer/search); recents list |
| `UI/WindIndicator.cs` | POL-5 | Rotates UI arrow to match DiscEnvironment.windVectorXYZ |
| `UI/ThrowResultPanelController.cs` | POL-4 | Slides in after landing; shows distance/height/drift |
| `UI/LeaderboardPanelController.cs` | POL-6 | ListView of top 10; persists to leaderboard.json |
| `UI/MiniMap.cs` | POL-3 | Software-drawn Texture2D; Bresenham lines; no extra camera; tee at bottom, flight upward |
| `Environment/DistanceMarkerSpawner.cs` | SIM-2 | Spawns TMP labels at configurable intervals |
| `Visualization/ShotPreviewLine.cs` | POL-1 | Ghost trajectory; debounced 5Hz; white→transparent gradient |
| `Visualization/LandingMarker.cs` | POL-2 | Pooled; TMP distance label; float+fade animation |
| `Scoring/ScoreEntry.cs` | POL-6 | Serializable; playerName, distance, disc, dateTime |
| `Scoring/LeaderboardManager.cs` | POL-6 | Persists leaderboard.json; fires UnityEvent on event |
| `Replay/ThrowReplay.cs` | POL-7 | Serialize discStateArray; play/pause/scrub coroutine |
| `Game/PracticeRangeMode.cs` | POL-6 | Modes: Freestyle, DistanceChallenge, AccuracyChallenge |
| `Course/HoleDefinition.cs` | POL-9 | ScriptableObject: holeName, par, tee/basket positions |
| `Course/CourseDefinition.cs` | POL-9 | ScriptableObject: List<HoleDefinition> |
| `Settings/AppSettings.cs` | INF-5 | PlayerPrefs/JSON; preferMetric, defaultDisc, cameraSerial |

### New Editor files
| File | Phase | Notes |
|------|-------|-------|
| `Camera/Editor/CameraCalibrationEditor.cs` | CAM-3 | EditorWindow; Edit mode; live FLIR preview; checkerboard capture + CalibrateCamera; saves JSON to StreamingAssets |
| `Camera/Editor/DiscVisionDeluxe.Camera.Editor.asmdef` | CAM-3 | Editor-only; refs DiscVisionDeluxe.Camera |
| `DfisX/Editor/DiscParamsImporter.cs` | SIM-4 | EditorWindow (`DfisX → Import CSV Disc Database`); reads `disc_params_pdga_final.csv`; InvariantCulture float parsing; creates one DiscModel .asset per row; preserves `rim_camber_shape=NONE` as-is; progress bar; skip-existing toggle |

### New UXML/USS files
| File | Phase |
|------|-------|
| `UI/ThrowParameterPanel.uxml` + `.uss` | SIM-1 |
| `UI/ThrowResultPanel.uxml` | POL-4 |
| `UI/LeaderboardPanel.uxml` | POL-6 |

### New Scenes
| Scene | Phase | Notes |
|-------|-------|-------|
| `Scenes/LiveCapture.unity` | CAM-5 | DiscDebug hierarchy + SpinnakerCameraCapture + camera preview RawImage |
| `Scenes/PracticeRange.unity` | SIM-2 | Large terrain; tee pad; distance markers; Cinemachine rig |
| `Scenes/MainMenu.unity` | INF-5 | Scene selection UI |

### New Asset folders
| Folder | Phase |
|--------|-------|
| `Assets/StreamingAssets/CameraCalibration/` | CAM-3 |
| `Assets/Plugins/Spinnaker/` | CAM-1 |

### Modified Existing Files
| File | Phase | Change |
|------|-------|--------|
| `Assets/DiscVisionDeluxe/DiscSimulator.cs` | CAM-5 | Add DataSource enum + PushMeasurement() |
| `Assets/DiscVisionDeluxe/DiscVisualizer.cs` | SIM-1, POL-2 | AeroDebug overload + LandingMarker spawn |
| `Assets/DfisX/Runtime/Core/DfisXStructs.cs` | POL-4 | Extend FlightStats: maxHeightM, lateralDriftM |
| `Assets/DfisX/Runtime/DiscFlightSimulator.cs` | POL-4 | Populate new FlightStats fields in BuildStats() |
| `Assets/DfisX/Runtime/Core/Daero.cs` | INF-4 | Swap System.Random → Unity.Mathematics.Random |
| `Assets/DfisX/Runtime/Core/DfisX.Runtime.asmdef` | INF-4 | Add allowUnsafeCode: true |
| `Packages/manifest.json` | INF-2,4,6 | Add URP, Cinemachine, Burst, Jobs |

---

## Verification Checklist

| Phase | How to Verify |
|-------|--------------|
| INF-1 | Project compiles; existing DiscDebug scene still runs |
| INF-2 | DiscDebug scene under URP; no pink materials |
| CAM-1 | ✅ FLIR camera opens in Play mode; grayscale preview visible in RawImage |
| CAM-2 | AprilTag corners overlaid on RawImage preview |
| CAM-3 | Checkerboard edges straight in undistorted frame |
| CAM-5 | Real throw → onThrowComplete fires → disc flies in Unity |
| SIM-1 | Sliders launch disc; Aero Debug section collapses/expands |
| SIM-2 | Distance markers visible every 10m up to 150m |
| SIM-3 | Camera follows disc; C key cycles through angle modes |
| SIM-4 | 1,272 discs imported; type/stability/manufacturer/search filters all work; recents show last 5 thrown |
| POL-1 | Ghost trajectory updates as sliders change (max 5Hz) |
| POL-2 | Distance label appears at landing; fades after 5s |
| POL-3 | Mini-map visible in corner; disc dot moves during flight |
| POL-4 | Stats panel slides in with correct values after landing |
| POL-7 | Throw saved; replays at variable speed |

---

## Session Log

| Session | Work Done |
|---------|-----------|
| Session 7 | Infrastructure setup: created next_steps.md (this file), DiscVisionDeluxe.asmdef (INF-1), updated manifest.json with URP/Cinemachine/Burst/Jobs (INF-2, INF-6). Manual URP Editor steps still required. |
| Session 8 | SIM-1: ThrowParameterPanel (UXML/USS/controller) with disc dropdown, throw sliders, aero debug foldout, camera dropdown, stats label. SIM-2: DistanceMarkerSpawner along +X axis with world-space TMP labels. SIM-3: FollowFlightCamera (4 modes, CM3 API, VCam_Follow manually positioned). Many bug fixes: stylesheet loading, stats label resize, disc dropdown width, marker axis (+X not +Z), label facing direction, CM3 camera priority switching. |
| Session 9 | Fixed camera auto-switch overriding user dropdown selection on launch. Added `groundOffsetY` to DistanceMarkerSpawner. Fixed disc phasing through ground: `HandoffToRigidbody` now syncs Rigidbody position to discTransform and uses `CollisionDetectionMode.Continuous`. Documented VCam Inspector setup (Tracking Target + CinemachineFollow offsets). Grass plane: URP Lit material with green albedo. |
| Session 10 | CAM-1 complete. SpinnakerNET_v140.dll is C++/CLI — cannot be used as managed reference. Rewrote camera integration as P/Invoke against SpinnakerC_v140.dll via SpinnakerCAPI.cs. Copied 12 DLLs to Assets/Plugins/Spinnaker/ (all Native, x64 only, Editor+Standalone); vcomp140.dll sourced from System32 (not in Spinnaker bin64). Missing DLLs found via Dependencies tool + Process Monitor. Full PC reboot required to seat Spinnaker kernel filter driver. spinImageConvert absent in v3.2 — configure camera to Mono8 via node map instead. TextureFormat.R8 displays teal in URP — fixed with RGB24 + manual byte expansion in preview path only. Camera feed confirmed working. |
| Session 11 | CAM-2 complete. Chose Emgu CV (free/NuGet) over OpenCVForUnity (paid). Installed Emgu.CV 4.12.0 via NuGetForUnity; manually placed cvextern.dll + libusb-1.0.dll as native plugins (both found via Dependencies tool). cvextern.dll meta file was auto-generated nearly empty — rewrote with correct Native plugin settings. AprilTag dictionary: (PredefinedDictionaryName)20 integer cast (enum name missing in 4.12). DetectorParameters: GetDefault() factory method required — new DetectorParameters() zero-inits and OpenCV rejects markerBorderBits=0. ArucoDetector class absent — use ArucoInvoke.DetectMarkers static method. SpinnakerCAPI changed from internal to public for Editor assembly access. Detection confirmed working: [AprilTag] ID=0 corners=(387,270). CAM-3 tool built: CameraCalibrationEditor EditorWindow runs in Edit mode (no Play mode conflict), captures checkerboard frames, runs CalibrateCamera, saves JSON to StreamingAssets/CameraCalibration/. Looked up BFS-U3-04S2C-CS specs: 720×540, global shutter, 290 FPS Mono8 / 522 FPS BayerRG8. Fixed targetFps default (60→290), added autoExposureMaxUs=1000µs field to cap auto-exposure and prevent motion blur. |
| Session 12 | CAM-4 complete: GroundPlaneCalibration.cs + GroundPlaneCalibrator.cs. Single reference tag, averages N frames of rvec+tvec, computes worldToCamera/cameraToWorld 4×4 matrices, saves ground_plane.json. CAM-5 complete (PoC): LiveDiscTracker.cs — subscribes to onTagsDetected, builds KFMeasurements, drives DiscKalmanFilter, fires onThrowComplete(DiscInitState) → DiscVisualizer.LaunchDfisX. End-to-end PoC confirmed: disc throw detected, KF resolves, sim runs (Steps=412, Distance=1.8m). Critical bug fixed: solvePnP CopyTo dimension mismatch (1×3 vs 3×1 matrix) caused all tvec values to be zero — fixed by using Matrix<double>(3,1) directly as output. Detection pipeline rearchitected: two threads (CaptureLoop at 290fps + DetectionLoop at ~25-30fps) with 60-frame bounded queue so fast throws are fully buffered. Attempted 522fps BayerRG8 — caused rapid camera disconnect (USB bandwidth); reverted to Mono8 290fps. Ground plane integrated into LiveDiscTracker: cameraToWorld transform applied in BuildMeasurement when profile loaded. Camera moved from nadir (straight down) to 45° looking downrange after confirming nadir cannot measure disc altitude. Ground plane recalibrated. Z axis still inverted due to poor camera calibration quality (RMS 3.4274). Workaround: FireComplete() clamps linearPositionM.z to 1.0f when negative. Open issues: (1) camera calibration RMS too high — needs redo with better lighting, (2) Z axis inverted in ground plane, (3) real throws not detected without establishment, (4) 522fps path needs USB bandwidth investigation. User hypotheses: better lighting will improve calibration; higher framerate (522fps) will improve detection of fast throws. |
| Session 13 | SIM-4 complete. Created disc_params_pdga_final.csv (1,272 discs, user-provided PDGA data). Created DiscParamsImporter.cs EditorWindow (DfisX → Import CSV Disc Database): parses CSV with InvariantCulture floats, creates one DiscModel .asset per row, preserves rim_camber_shape=NONE as-is, progress bar, skip-existing toggle. UI panel disc section rebuilt: old filter rows replaced with compact inline-label dropdowns (same 26px height as slider rows) eliminating panel scale issue. Added four AND-combined disc filters: Type, Stability, Manufacturer (sorted unique from library), and free-text Search (case-insensitive, matches mold name or manufacturer, fires on every keystroke). Added Recents section: static list of last 5 thrown discs shown as pill buttons; clicking a pill selects that disc directly bypassing filters; active pill highlighted green. Disc selection refactored: _selectedModel is single authoritative field for all physics builds; preserved across filter rebuilds when disc still in filtered list. |
