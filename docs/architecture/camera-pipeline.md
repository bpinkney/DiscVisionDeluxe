# Camera Pipeline Architecture

## Why P/Invoke, Not SpinnakerNET

`SpinnakerNET_v140.dll` is a **C++/CLI mixed-mode assembly** — a hybrid of managed and unmanaged code that Unity's runtime cannot load as a managed assembly. No plugin configuration makes it work as a C# reference; it always behaves as a native DLL.

The solution is P/Invoke against `SpinnakerC_v140.dll`, which provides a pure C interface. All bindings live in `SpinnakerCAPI.cs`:
```csharp
[DllImport("SpinnakerC_v140")]
public static extern spinError spinCameraGetNextImage(spinCamera hCamera, ref spinImage hImage);
```

## Why Emgu CV, Not OpenCVForUnity

OpenCVForUnity costs ~$95 on the Asset Store. Emgu CV 4.12 is free (NuGet, Apache 2.0). Both wrap the same OpenCV 4.x with the Aruco module. Emgu CV 4.12 was installed via NuGetForUnity.

The native runtime (`cvextern.dll`, ~80MB) must be placed manually in `Assets/Plugins/EmguCV/` — NuGetForUnity does not include it. `libusb-1.0.dll` is a hidden dependency of `cvextern.dll` that must also be placed there.

## Thread Architecture

```
SpinnakerCameraCapture
  ┌─────────────────────────────────────────────────────────────┐
  │ CaptureLoop thread (SpinnakerCapture)                       │
  │   Grabs frames at 290fps from camera                        │
  │   → _pendingDetectQueue (ConcurrentQueue, max 60 frames)    │
  └─────────────────────────────────────────────────────────────┘
  ┌─────────────────────────────────────────────────────────────┐
  │ DetectionLoop thread (AprilTagDetection)                    │
  │   Dequeues frames → ArucoInvoke.DetectMarkers()             │
  │   Effective detection rate: ~25–30 fps                      │
  │   → _detectionQueue (ConcurrentQueue<DetectionFrame>)       │
  │     with original capture timestamp                         │
  └─────────────────────────────────────────────────────────────┘
  ┌─────────────────────────────────────────────────────────────┐
  │ Main thread Update()                                        │
  │   Drains _detectionQueue                                    │
  │   Sets LastDetectionTimestampNs                             │
  │   Fires onTagsDetected (UnityEvent)                         │
  └─────────────────────────────────────────────────────────────┘
```

The 60-frame bounded queue (≈ 200ms at 290fps) ensures all frames containing the disc are buffered even if the disc passes through the frame faster than the detection thread processes.

**Rule:** Never call any Unity API from a background thread. The only cross-thread communication is via `ConcurrentQueue`.

## Clock Synchronisation

A single `Stopwatch` (`_captureClock`) is started in `StartCapture()` and provides all timestamps. Every frame is tagged with its capture timestamp (nanoseconds since `_captureClock.Start()`). `LiveDiscTracker` reads `cameraCapture.ElapsedCaptureNs` for current time and `cameraCapture.LastDetectionTimestampNs` for detection time. Do not mix in `DateTime.Now` or `Time.realtimeSinceStartup`.

## AprilTag Pose Estimation

`AprilTagDetector.Detect()` calls `CvInvoke.SolvePnP()` for each detected tag that has calibration intrinsics set. Returns `rvec` (Rodrigues rotation) and `tvec` (translation in camera space).

Critical: solvePnP outputs 3×1 **column** vectors. `new Matrix<double>(rvec)` creates a row vector — dimension mismatch causes `CopyTo()` to silently fail. Always use `new Matrix<double>(3, 1)` and read with `[row, col]` indexer.

## Frame Rate Note

Mono8 at 290fps is the standard mode. BayerRG8 at 522fps was tested but caused rapid USB disconnect (720×540×522fps ≈ 200MB/s bandwidth). To revisit: add `CvtColor(BayerRG→Gray)` step, use ROI mode to reduce resolution, verify USB 3.0 host controller bandwidth.
