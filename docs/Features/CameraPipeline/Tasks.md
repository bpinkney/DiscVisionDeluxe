# Camera Pipeline — Tasks

## Active Session
- **Last action**: None
- **Next action**: CAM-ISSUE-1 — redo camera calibration (physical setup required)
- **Blocker**: Requires physical recalibration (lighting, checkerboard)

## Current Sprint
- [ ] CAM-ISSUE-1: Redo camera calibration (RMS 3.4274 → target <1.0)
- [ ] CAM-ISSUE-2: Z axis inversion (blocked on ISSUE-1)
- [ ] CAM-ISSUE-3: Real throws not detected without establishment

## CAM-ISSUE-1 Steps
1. Better lighting (even, diffuse — no harsh shadows on checkerboard)
2. Cover frame corners and edges (distortion worst there)
3. 20–30 frames, varied board tilt and rotation
4. Measure printed square size with calipers
5. Confirm new RMS <1.0 before redoing ground plane calibration

## CAM-ISSUE-3 Options (try in order)
1. Reduce image to 360×270 before DetectMarkers (halve intrinsics for solvePnP, or scale corners back up)
2. Lower `primeMinVar` further in Inspector (try 0.01–0.05)
3. Camera ROI mode at higher fps (requires USB bandwidth investigation for BayerRG8)

## Backlog
- [ ] CAM-ISSUE-4: Update Detection Timeout Ms to 2000 in existing scenes (low — old scenes may have 500ms)
- [ ] Investigate BayerRG8 at higher fps with ROI mode (USB bandwidth permitting)

## Completed
- [x] CAM-1: Spinnaker P/Invoke — 12 DLLs, camera feed confirmed (Session 10)
- [x] CAM-2: Emgu CV 4.12 — AprilTag detection confirmed (Session 11)
- [x] CAM-3: CameraCalibrationEditor EditorWindow (Session 11)
- [x] CAM-4: GroundPlaneCalibration + GroundPlaneCalibrator (Session 12)
- [x] CAM-5: LiveDiscTracker PoC — end-to-end throw detected and sim ran (Session 12)
