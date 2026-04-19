# Camera Calibration

Two calibration steps are required before live disc tracking works: **lens calibration** (intrinsics) and **ground plane calibration** (extrinsics).

---

## Part 1: Lens Calibration

### Files
- `Camera/CameraCalibration.cs` — runtime data class; holds fx/fy/cx/cy + distortion coefficients
- `Camera/Editor/CameraCalibrationEditor.cs` — EditorWindow; runs in Edit mode (no Play mode needed)

### Target
Use a standard OpenCV checkerboard printed flat and mounted on rigid backing (foam board).
- Default: 9×6 **inner corners** (= 10×7 grid of squares)
- Measure printed square size with calipers — printer scaling affects accuracy
- Flat mount is critical: any warp degrades accuracy

### Procedure
1. Open **DiscVisionDeluxe → Camera Calibration Tool** from the menu bar
2. Connect camera (Edit mode — no Play mode required)
3. Hold the checkerboard in front of the camera; detected corners are overlaid in green/red
4. Capture 20–30 frames with:
   - Varied board tilt and rotation
   - Coverage across corners and edges of the frame (distortion is worst there)
   - Even, diffuse lighting — no harsh shadows on the checkerboard
5. Click **Run Calibration**
6. Enter a profile name and click **Save** → writes to `StreamingAssets/CameraCalibration/{name}.json`

### Quality Target
- RMS < 1.0 pixels: good
- RMS < 0.5 pixels: excellent
- Current calibration: RMS 3.4274 — needs redo (see [Open Issues](../development/open-issues.md#issue-1))

### Using the Calibration at Runtime
```csharp
var cal = CameraCalibration.LoadProfile("camera_calibration");
aprilTagDetector.SetCalibration(cal.fx, cal.fy, cal.cx, cal.cy, cal.distCoeffs);
```

---

## Part 2: Ground Plane Calibration

### Files
- `Camera/GroundPlaneCalibration.cs` — serializable data; holds `worldToCamera[16]`, `cameraToWorld[16]`, `cameraHeightM`
- `Camera/GroundPlaneCalibrator.cs` — MonoBehaviour; averages N frames; saves `ground_plane.json`

### Reference Tag Setup

Place a reference AprilTag flat on the ground at the tee position with:
- **X axis pointing downrange** (disc flight direction)
- Y axis 90° right of X on the ground plane
- Z axis pointing up toward the camera (printed face must be visible)

The printed face of the tag must be visible to the camera. If the tag is face-down, the Z axis will be inverted.

### Inspector Setup
- `SpinnakerCameraCapture` component assigned
- `cameraCalibrationProfile` set to `"camera_calibration"` so intrinsics load on Start
- `referenceTagId` set to the ID of the tag on the ground (confirm with `[AprilTag] ID=X` in Console)

### Procedure
1. Enter Play mode
2. Right-click the GroundPlaneCalibrator component → **Start Ground Calibration**
3. Keep the ground tag fully visible until `_framesCollected` reaches `averageFrames` (default 30)
4. Calibration saves automatically to `StreamingAssets/CameraCalibration/ground_plane.json`
5. Confirm `[GroundCalib] Done! Height=X.XXXm` in Console

### Math Reference
- `worldToCamera = [R | t]` — transforms world points into camera space
- `cameraToWorld = [R^T | -R^T*t]` — transforms camera points into world space
- `cameraHeightM = tz` (Z component of camera position in world frame)
- Row-major 4×4 matrices; homogeneous multiply

### Loading at Runtime
`GroundPlaneCalibration.LoadProfile("ground_plane")` — called by LiveDiscTracker on `OnEnable()`.
