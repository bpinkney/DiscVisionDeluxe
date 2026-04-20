# Camera Pipeline — Changelog

## [Active] — Issues Open
- CAM-ISSUE-1: Calibration RMS 3.4274 (needs redo)
- CAM-ISSUE-2: Z axis inversion (blocked on ISSUE-1)
- CAM-ISSUE-3: Natural throw detection without establishment

## Session 12 — CAM-4 + CAM-5
- GroundPlaneCalibration.cs + GroundPlaneCalibrator.cs implemented
- LiveDiscTracker PoC: end-to-end throw detected, sim ran
- solvePnP 3×1 column vector fix (CopyTo dimension mismatch)
- Two-thread pipeline (CaptureLoop + DetectionLoop) established
- Z inversion workaround in FireComplete() (clamp z=1.0f when negative)

## Session 11 — CAM-2 + CAM-3
- Emgu CV 4.12 integrated — AprilTag detection confirmed
- Integer cast dict fix, ArucoInvoke.DetectMarkers, DetectorParameters.GetDefault()
- cvextern.dll + libusb-1.0.dll placed manually
- CameraCalibrationEditor EditorWindow built (checkerboard capture, live preview)

## Session 10 — CAM-1
- Spinnaker P/Invoke bindings in SpinnakerCAPI.cs
- 12 DLLs configured (including vcomp140 from System32)
- Full PC reboot required for Spinnaker kernel filter driver
- Camera feed confirmed at 290fps Mono8
- SpinnakerNET mixed-mode issue identified and bypassed
