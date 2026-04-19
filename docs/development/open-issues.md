# Open Issues

Known problems with context, workarounds, and fix approaches. Update when issues are resolved.

---

## ISSUE-1: Camera Calibration RMS Too High
**Severity:** High — blocks accurate pose estimation and ground plane calibration
**As of:** Session 12

Current calibration values: `fx=321.0, fy=320.6, cx=353.6, cy=281.8`, RMS = **3.4274 pixels**
Target: RMS < 1.0 (good), < 0.5 (excellent)

This error propagates into every solvePnP call and is the root cause of ISSUE-2.

**Root cause hypothesis:** Poor lighting during calibration caused noisy sub-pixel corner detection.

**Fix:**
1. Redo calibration with even, diffuse lighting — no harsh shadows on the checkerboard
2. Cover frame corners and edges (distortion is worst there)
3. Use 20–30 frames with varied board tilt and rotation
4. Measure printed square size with calipers
5. Confirm new RMS < 1.0 before redoing ground plane calibration

---

## ISSUE-2: World Z Axis Inverted in Ground Plane
**Severity:** Medium — sim works via workaround, but Z velocity is incorrect
**As of:** Session 12, blocked on ISSUE-1

After applying `cameraToWorld`, disc positions show `kfZ ≈ -1.0 to -1.5m` when held at ~1m height. Expected: `+1.0m`.

**Root cause:** Poor intrinsics from ISSUE-1 cause solvePnP to compute an inaccurate rotation matrix → `cameraToWorld` has Z row pointing in the wrong direction.

**Workaround in place:** `LiveDiscTracker.FireComplete()` clamps `init.linearPositionM.z = 1.0f` when negative. Throws run, but Z velocity (e.g. `Vel.z = -1.63 m/s`) is still affected.

**Fix:** Resolve ISSUE-1 → redo ground plane calibration → Z should self-correct.

---

## ISSUE-3: Real Throws Not Detected Without Establishment
**Severity:** Medium — PoC works with establishment, not with natural throws
**As of:** Session 12

Currently the disc must be held still in the camera frame for ~0.5s before moving for the KF to acquire. A natural throw from outside the frame is not reliably caught.

**Root cause:** Effective detection rate is ~25–30fps (limited by AprilTag detection time per frame). A disc moving at speed gets only 3–6 frames; if velocity variance threshold isn't met, KF doesn't prime.

**Options (try in order):**
1. Reduce image resolution before `DetectMarkers` (e.g. `Imgproc.Resize` to 360×270 → 4× speed improvement). Must either halve intrinsics (`fx/2`, `fy/2`, `cx/2`, `cy/2`) passed to solvePnP, or scale detected corners back up before solvePnP.
2. Lower `primeMinVar` further in Inspector (try 0.01–0.05)
3. ROI mode on camera at higher fps (requires USB bandwidth investigation)

---

## ISSUE-4: Detection Timeout Margin
**Severity:** Low — mitigated by current 2000ms timeout
**As of:** Session 12

At ~30ms detection per frame × 60-frame queue = 1800ms worst-case drain time. Timeout is 2000ms (200ms margin). If detection is slower (high CPU load, larger image), queue may not drain before timeout.

**Note:** Existing scenes may still have the old 500ms timeout from before ISSUE-4 was identified. Manually update `Detection Timeout Ms` to 2000 in the Inspector for the LiveCapture scene.

**Fix:** No code change needed unless detection gets slower. Monitor if resolution reduction (ISSUE-3) changes the drain time calculation.
