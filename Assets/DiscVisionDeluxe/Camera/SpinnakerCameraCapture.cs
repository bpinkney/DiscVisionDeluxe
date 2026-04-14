using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using System.Threading;
using UnityEngine;
using UnityEngine.Events;
using static DiscVisionDeluxe.Camera.SpinnakerCAPI;

namespace DiscVisionDeluxe.Camera
{
    /// <summary>
    /// Captures frames from a FLIR Spinnaker USB3 camera on a background thread
    /// using the Spinnaker C API (P/Invoke — no managed SpinnakerNET wrapper needed).
    ///
    /// Each frame is converted to Mono8 and queued for the main thread.
    /// Wire onFrameCaptured → AprilTagDetector.OnFrame (CAM-2).
    /// </summary>
    public class SpinnakerCameraCapture : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("Camera")]
        [Tooltip("Index into the Spinnaker camera list (0 = first camera found).")]
        public int cameraIndex = 0;

        [Tooltip(
            "Desired frame rate (fps). 0 = camera default.\n" +
            "BFS-U3-04S2C-CS: Mono8 = 290 fps max at full 720×540 resolution.\n" +
            "Exceeding the camera limit is silently clamped by the SDK.")]
        public float targetFps = 290f;

        [Tooltip(
            "Manual exposure time in microseconds. 0 = auto-exposure (see AutoExposureMaxUs).\n" +
            "BFS-U3-04S2C-CS range: 4 µs – 30 000 000 µs.\n" +
            "For fast disc detection keep this at 0 and rely on AutoExposureMaxUs instead.")]
        public float exposureUs = 0f;

        [Tooltip(
            "Upper limit for auto-exposure in microseconds. Only used when exposureUs = 0.\n" +
            "Prevents the auto-exposure from choosing a long shutter that blurs a spinning disc.\n" +
            "1000 µs (1 ms) is a safe default — increase if the image is too dark.")]
        public float autoExposureMaxUs = 1000f;

        [Tooltip("Gain in dB. 0 = auto-gain. BFS-U3-04S2C-CS range: 0 – 47.99 dB.")]
        public float gainDb = 0f;

        [Header("Preview")]
        [Tooltip("Optional UI RawImage to display the live camera feed.")]
        public UnityEngine.UI.RawImage previewImage;

        [Header("AprilTag Detection")]
        [Tooltip("Physical side length of the AprilTag marker in metres.")]
        public float tagSizeM = 0.165f;

        [Tooltip("Profile name to load from StreamingAssets/CameraCalibration/ on Start. " +
                 "Leave empty to skip — calibration can be loaded later via LoadCalibration().")]
        public string calibrationProfile = "";

        [Header("Events")]
        [Tooltip("Fired on the main thread each frame: (pixels byte[], width int, height int).")]
        public FrameCapturedEvent onFrameCaptured = new FrameCapturedEvent();

        [Tooltip("Fired on the main thread when one or more AprilTags are detected.")]
        public TagsDetectedEvent onTagsDetected = new TagsDetectedEvent();

        // ── Public state ─────────────────────────────────────────────────────

        public bool IsCapturing => _running;
        public int  FrameWidth  { get; private set; }
        public int  FrameHeight { get; private set; }

        // ── Private ──────────────────────────────────────────────────────────

        readonly ConcurrentQueue<CapturedFrame>    _frameQueue     = new ConcurrentQueue<CapturedFrame>();
        readonly ConcurrentQueue<DetectionFrame>   _detectionQueue = new ConcurrentQueue<DetectionFrame>();

        // Bounded queue of frames waiting for AprilTag detection.
        // Capture thread enqueues at 290fps; detection thread drains at its own pace.
        // Cap at 60 frames (~200ms at 290fps) to bound memory use while capturing full throws.
        readonly ConcurrentQueue<PendingDetectFrame> _pendingDetectQueue
            = new ConcurrentQueue<PendingDetectFrame>();
        const int MaxPendingDetect = 60;

        Thread  _detectionThread;

        // Set just before firing onTagsDetected so listeners can read the capture timestamp
        public ulong LastDetectionTimestampNs { get; private set; }

        // Current elapsed time on the capture clock — use this for timeout calculations
        // in listeners so all timestamps share the same reference.
        public ulong ElapsedCaptureNs =>
            _captureClock != null ? (ulong)(_captureClock.Elapsed.TotalSeconds * 1e9) : 0;

        System.Diagnostics.Stopwatch _captureClock;

        IntPtr              _hSystem  = IntPtr.Zero;
        IntPtr              _hCamera  = IntPtr.Zero;
        Thread              _captureThread;
        volatile bool       _running;
        Texture2D           _previewTex;
        AprilTagDetector    _aprilTagDetector;

        // ── Unity lifecycle ──────────────────────────────────────────────────

        void Start()   => StartCapture();
        void OnDestroy() => StopCapture();

        void Update()
        {
            if (_frameQueue.TryDequeue(out CapturedFrame frame))
            {
                UpdatePreview(frame);
                onFrameCaptured.Invoke(frame.pixels, frame.width, frame.height);
            }

            while (_detectionQueue.TryDequeue(out var detFrame))
            {
                if (detFrame.detections.Count == 0) continue;
                LastDetectionTimestampNs = detFrame.timestampNs;
                onTagsDetected.Invoke(detFrame.detections);
            }
        }

        // ── Public API ───────────────────────────────────────────────────────

        /// <summary>
        /// Load a camera calibration profile and apply it to the AprilTag detector.
        /// Can be called at any time after StartCapture(); safe to call before Start().
        /// </summary>
        public void LoadCalibration(CameraCalibration cal)
        {
            if (cal == null) return;
            _aprilTagDetector?.SetCalibration(
                cal.fx, cal.fy, cal.cx, cal.cy,
                cal.distCoeffs[0], cal.distCoeffs[1],
                cal.distCoeffs[2], cal.distCoeffs[3],
                cal.distCoeffs[4]);
            Debug.Log($"[SpinnakerCapture] Calibration loaded: {cal}");
        }

        public void StartCapture()
        {
            if (_running) return;
            try
            {
                _captureClock     = System.Diagnostics.Stopwatch.StartNew();
                _aprilTagDetector = new AprilTagDetector(tagSizeM);

                if (!string.IsNullOrEmpty(calibrationProfile))
                {
                    var cal = CameraCalibration.LoadProfile(calibrationProfile);
                    if (cal != null) LoadCalibration(cal);
                    else Debug.LogWarning($"[SpinnakerCapture] Calibration profile '{calibrationProfile}' not found.");
                }
                InitCamera();
                _running       = true;
                _captureThread = new Thread(CaptureLoop)
                {
                    IsBackground = true,
                    Name         = "SpinnakerCapture"
                };
                _detectionThread = new Thread(DetectionLoop)
                {
                    IsBackground = true,
                    Name         = "AprilTagDetection"
                };
                _captureThread.Start();
                _detectionThread.Start();
                Debug.Log($"[SpinnakerCapture] Started — {FrameWidth}×{FrameHeight} @ {targetFps} fps.");
            }
            catch (Exception e)
            {
                Debug.LogError($"[SpinnakerCapture] Failed to start: {e.Message}");
            }
        }

        public void StopCapture()
        {
            _running = false;
            _captureThread?.Join(2000);
            _captureThread = null;
            _detectionThread?.Join(2000);
            _detectionThread = null;
            _aprilTagDetector?.Dispose();
            _aprilTagDetector = null;

            if (_hCamera != IntPtr.Zero)
            {
                spinCameraEndAcquisition(_hCamera);
                spinCameraDeInit(_hCamera);
                spinCameraRelease(_hCamera);
                _hCamera = IntPtr.Zero;
            }

            if (_hSystem != IntPtr.Zero)
            {
                spinSystemReleaseInstance(_hSystem);
                _hSystem = IntPtr.Zero;
            }
        }

        // ── Camera init ──────────────────────────────────────────────────────

        void InitCamera()
        {
            Spin(spinSystemGetInstance(out _hSystem), "SystemGetInstance");

            Spin(spinCameraListCreateEmpty(out IntPtr hList), "CameraListCreate");
            try
            {
                Spin(spinSystemGetCameras(_hSystem, hList), "GetCameras");

                Spin(spinCameraListGetSize(hList, out UIntPtr count), "CameraListSize");
                if ((int)count == 0)
                    throw new Exception("No Spinnaker cameras found. Check USB connection.");
                if (cameraIndex >= (int)count)
                    throw new Exception($"Camera index {cameraIndex} out of range ({count} found).");

                Spin(spinCameraListGet(hList, (UIntPtr)cameraIndex, out _hCamera), "CameraListGet");
            }
            finally
            {
                spinCameraListClear(hList);
                spinCameraListDestroy(hList);
            }

            Spin(spinCameraInit(_hCamera), "CameraInit");

            Spin(spinCameraGetNodeMap(_hCamera, out IntPtr hNodeMap), "GetNodeMap");

            SetEnum (hNodeMap, "AcquisitionMode", "Continuous");
            SetEnum (hNodeMap, "PixelFormat",      "Mono8");

            if (targetFps > 0f)
            {
                SetBool (hNodeMap, "AcquisitionFrameRateEnable", true);
                SetFloat(hNodeMap, "AcquisitionFrameRate", targetFps);
            }

            if (exposureUs > 0f)
            {
                SetEnum (hNodeMap, "ExposureAuto", "Off");
                SetFloat(hNodeMap, "ExposureTime", exposureUs);
            }
            else
            {
                SetEnum(hNodeMap, "ExposureAuto", "Continuous");

                // Cap auto-exposure to prevent motion blur on a fast disc.
                // Node name on Blackfly S firmware: "AutoExposureTimeUpperLimit".
                if (autoExposureMaxUs > 0f)
                    SetFloat(hNodeMap, "AutoExposureTimeUpperLimit", autoExposureMaxUs);
            }

            if (gainDb > 0f)
            {
                SetEnum (hNodeMap, "GainAuto", "Off");
                SetFloat(hNodeMap, "Gain", gainDb);
            }
            else
            {
                SetEnum(hNodeMap, "GainAuto", "Continuous");
            }

            Spin(spinCameraBeginAcquisition(_hCamera), "BeginAcquisition");

            // Probe first frame for actual resolution
            if (OK(spinCameraGetNextImageEx(_hCamera, 2000, out IntPtr hProbe)))
            {
                spinImageGetWidth (hProbe, out UIntPtr w);
                spinImageGetHeight(hProbe, out UIntPtr h);
                FrameWidth  = (int)w;
                FrameHeight = (int)h;
                spinImageRelease(hProbe);
            }
        }

        // ── Capture thread — grabs frames at full camera rate, never blocks on detection ──

        void CaptureLoop()
        {
            while (_running)
            {
                int err = spinCameraGetNextImageEx(_hCamera, 500, out IntPtr hRaw);

                if (IsTimeout(err)) continue;
                if (!OK(err))
                {
                    if (_running) Debug.LogWarning($"[SpinnakerCapture] GetNextImage error {err}");
                    continue;
                }

                spinImageIsIncomplete(hRaw, out byte incomplete);
                if (incomplete != 0) { spinImageRelease(hRaw); continue; }

                spinImageGetWidth (hRaw, out UIntPtr uw);
                spinImageGetHeight(hRaw, out UIntPtr uh);
                int w = (int)uw, h = (int)uh;

                spinImageGetData(hRaw, out IntPtr dataPtr);
                byte[] pixels = new byte[w * h];
                Marshal.Copy(dataPtr, pixels, 0, pixels.Length);
                spinImageRelease(hRaw);

                ulong captureNs = (ulong)(_captureClock.Elapsed.TotalSeconds * 1e9);

                // Hand frame to detection thread — drop oldest if queue is full
                if (_pendingDetectQueue.Count < MaxPendingDetect)
                    _pendingDetectQueue.Enqueue(new PendingDetectFrame(pixels, w, h, captureNs));
                else if (_pendingDetectQueue.TryDequeue(out _))
                    _pendingDetectQueue.Enqueue(new PendingDetectFrame(pixels, w, h, captureNs));

                // Keep only the most recent frame for preview
                while (_frameQueue.Count > 0) _frameQueue.TryDequeue(out _);
                _frameQueue.Enqueue(new CapturedFrame(pixels, w, h));
            }
        }

        // ── Detection thread — runs AprilTag at its own pace without blocking capture ──

        void DetectionLoop()
        {
            while (_running)
            {
                if (!_pendingDetectQueue.TryDequeue(out var pending))
                {
                    Thread.Sleep(1);
                    continue;
                }

                var detections = _aprilTagDetector?.Detect(pending.pixels, pending.w, pending.h);
                if (detections != null && detections.Count > 0)
                    _detectionQueue.Enqueue(new DetectionFrame(detections, pending.captureNs));
            }
        }

        // ── Preview ──────────────────────────────────────────────────────────

        void UpdatePreview(CapturedFrame frame)
        {
            if (previewImage == null) return;

            if (_previewTex == null
                || _previewTex.width  != frame.width
                || _previewTex.height != frame.height)
            {
                if (_previewTex != null) Destroy(_previewTex);
                _previewTex = new Texture2D(frame.width, frame.height, TextureFormat.RGB24, false);
                previewImage.texture = _previewTex;
            }

            // Expand Mono8 (1 byte/pixel) → RGB24 (3 bytes/pixel) for correct grayscale display
            int pixelCount = frame.pixels.Length;
            byte[] rgb = new byte[pixelCount * 3];
            for (int i = 0; i < pixelCount; i++)
            {
                byte v = frame.pixels[i];
                rgb[i * 3]     = v;
                rgb[i * 3 + 1] = v;
                rgb[i * 3 + 2] = v;
            }

            _previewTex.SetPixelData(rgb, 0);
            _previewTex.Apply(false);
        }

        // ── GenICam helpers ──────────────────────────────────────────────────

        static void SetEnum(IntPtr nm, string nodeName, string entryName)
        {
            try
            {
                if (!OK(spinNodeMapGetNode(nm, nodeName, out IntPtr hNode))) return;
                if (!OK(spinEnumerationGetEntryByName(hNode, entryName, out IntPtr hEntry))) return;
                if (!OK(spinEnumerationEntryGetEnumValue(hEntry, out UIntPtr val))) return;
                spinEnumerationSetEnumValue(hNode, val);
            }
            catch (Exception e) { Debug.LogWarning($"[Spinnaker] SetEnum {nodeName}={entryName}: {e.Message}"); }
        }

        static void SetFloat(IntPtr nm, string nodeName, double value)
        {
            try
            {
                if (!OK(spinNodeMapGetNode(nm, nodeName, out IntPtr hNode))) return;
                spinFloatGetMin(hNode, out double min);
                spinFloatGetMax(hNode, out double max);
                spinFloatSetValue(hNode, Math.Clamp(value, min, max));
            }
            catch (Exception e) { Debug.LogWarning($"[Spinnaker] SetFloat {nodeName}={value}: {e.Message}"); }
        }

        static void SetBool(IntPtr nm, string nodeName, bool value)
        {
            try
            {
                if (!OK(spinNodeMapGetNode(nm, nodeName, out IntPtr hNode))) return;
                spinBooleanSetValue(hNode, value ? (byte)1 : (byte)0);
            }
            catch (Exception e) { Debug.LogWarning($"[Spinnaker] SetBool {nodeName}={value}: {e.Message}"); }
        }

        // Throws if a required Spinnaker call fails
        static void Spin(int err, string context)
        {
            if (!OK(err))
                throw new Exception($"Spinnaker {context} failed (err {err})");
        }

        // ── Inner types ──────────────────────────────────────────────────────

        readonly struct CapturedFrame
        {
            public readonly byte[] pixels;
            public readonly int    width;
            public readonly int    height;

            public CapturedFrame(byte[] pixels, int width, int height)
            {
                this.pixels = pixels;
                this.width  = width;
                this.height = height;
            }
        }

        readonly struct PendingDetectFrame
        {
            public readonly byte[] pixels;
            public readonly int    w, h;
            public readonly ulong  captureNs;

            public PendingDetectFrame(byte[] pixels, int w, int h, ulong captureNs)
            {
                this.pixels    = pixels;
                this.w         = w;
                this.h         = h;
                this.captureNs = captureNs;
            }
        }

        readonly struct DetectionFrame
        {
            public readonly List<AprilTagDetector.TagDetection> detections;
            public readonly ulong                               timestampNs;

            public DetectionFrame(List<AprilTagDetector.TagDetection> detections, ulong timestampNs)
            {
                this.detections  = detections;
                this.timestampNs = timestampNs;
            }
        }
    }

    [Serializable]
    public class FrameCapturedEvent : UnityEvent<byte[], int, int> { }

    [Serializable]
    public class TagsDetectedEvent : UnityEvent<List<AprilTagDetector.TagDetection>> { }
}
