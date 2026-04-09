using System;
using System.Collections.Generic;
using System.IO;
using System.Runtime.InteropServices;
using System.Threading;
using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using UnityEditor;
using UnityEngine;
using static DiscVisionDeluxe.Camera.SpinnakerCAPI;

namespace DiscVisionDeluxe.Camera.Editor
{
    /// <summary>
    /// EditorWindow that drives the FLIR camera directly (no Play mode required),
    /// detects checkerboard corners, collects frames, runs CalibrateCamera, and
    /// saves a CameraCalibration JSON to StreamingAssets/CameraCalibration/.
    ///
    /// Open via menu: DiscVisionDeluxe → Camera Calibration Tool
    /// </summary>
    public class CameraCalibrationEditor : EditorWindow
    {
        // ── Inspector-style fields ────────────────────────────────────────────

        int    _cameraIndex   = 0;
        int    _boardCols     = 9;    // inner corners horizontally
        int    _boardRows     = 6;    // inner corners vertically
        float  _squareMm      = 25f;
        int    _targetFrames  = 20;
        string _profileName   = "camera_calibration";

        // ── Camera state ──────────────────────────────────────────────────────

        IntPtr        _hSystem = IntPtr.Zero;
        IntPtr        _hCamera = IntPtr.Zero;
        bool          _running;
        Thread        _captureThread;
        int           _frameWidth, _frameHeight;

        // Background thread writes here; main thread reads under _frameLock
        readonly object _frameLock    = new object();
        byte[]          _latestPixels;          // Mono8
        bool            _latestCornersFound;
        System.Drawing.PointF[] _latestCorners;

        // ── Capture storage ───────────────────────────────────────────────────

        // Each captured frame: the 2-D corner positions
        readonly List<System.Drawing.PointF[]> _capturedCorners = new List<System.Drawing.PointF[]>();
        bool  _autoCapture;
        float _autoCaptureCooldown;       // seconds until next auto capture
        const float AutoCapturePeriod = 1.5f;

        // ── Preview ───────────────────────────────────────────────────────────

        Texture2D _previewTex;
        Vector2   _scrollPos;

        // ── Result ────────────────────────────────────────────────────────────

        CameraCalibration _result;
        volatile bool     _calibrating;
        CameraCalibration _pendingResult; // written by bg thread, read under _resultLock
        readonly object   _resultLock = new object();

        // ── Menu entry ────────────────────────────────────────────────────────

        [MenuItem("DiscVisionDeluxe/Camera Calibration Tool")]
        static void Open() => GetWindow<CameraCalibrationEditor>("Camera Calibration");

        // ── Lifecycle ─────────────────────────────────────────────────────────

        void OnEnable()
        {
            EditorApplication.update                += OnEditorUpdate;
            EditorApplication.playModeStateChanged  += OnPlayModeChanged;
        }

        void OnDisable()
        {
            EditorApplication.update                -= OnEditorUpdate;
            EditorApplication.playModeStateChanged  -= OnPlayModeChanged;
            StopCamera();
            if (_previewTex != null) DestroyImmediate(_previewTex);
        }

        void OnPlayModeChanged(PlayModeStateChange change)
        {
            // Don't fight Play mode over the camera hardware
            if (change == PlayModeStateChange.ExitingEditMode)
                StopCamera();
        }

        // ── EditorApplication.update (main thread, ~10Hz) ─────────────────────

        void OnEditorUpdate()
        {
            // Drain calibration result posted by background thread
            if (_calibrating)
            {
                lock (_resultLock)
                {
                    if (_pendingResult != null)
                    {
                        _result       = _pendingResult;
                        _pendingResult = null;
                        _calibrating  = false;
                        Debug.Log($"[CalibTool] Calibration done. RMS={_result.rmsError:F4}  {_result}");
                        Repaint();
                    }
                }
            }

            if (!_running) return;

            // Auto-capture cooldown
            _autoCaptureCooldown -= 0.1f; // called ~10Hz

            byte[] pixels;
            bool   cornersFound;
            System.Drawing.PointF[] corners;

            lock (_frameLock)
            {
                if (_latestPixels == null) return;
                pixels       = _latestPixels;
                cornersFound = _latestCornersFound;
                corners      = _latestCorners;
                _latestPixels = null; // consume
            }

            UpdatePreviewTexture(pixels, _frameWidth, _frameHeight, cornersFound, corners);

            if (_autoCapture && cornersFound && _autoCaptureCooldown <= 0f)
            {
                DoCapture(corners);
                _autoCaptureCooldown = AutoCapturePeriod;
            }

            Repaint();
        }

        // ── GUI ───────────────────────────────────────────────────────────────

        void OnGUI()
        {
            _scrollPos = EditorGUILayout.BeginScrollView(_scrollPos);

            // ── Camera ──────────────────────────────────────────────────────
            EditorGUILayout.LabelField("Camera", EditorStyles.boldLabel);
            GUI.enabled = !_running;
            _cameraIndex = EditorGUILayout.IntField("Camera Index", _cameraIndex);
            GUI.enabled = true;

            EditorGUILayout.BeginHorizontal();
            if (!_running)
            {
                if (GUILayout.Button("Connect", GUILayout.Width(100))) StartCamera();
            }
            else
            {
                if (GUILayout.Button("Disconnect", GUILayout.Width(100))) StopCamera();
                GUILayout.Label($"{_frameWidth}×{_frameHeight}", EditorStyles.helpBox);
            }
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(8);

            // ── Board settings ───────────────────────────────────────────────
            EditorGUILayout.LabelField("Checkerboard", EditorStyles.boldLabel);
            _boardCols    = EditorGUILayout.IntField("Inner Corners (cols)", _boardCols);
            _boardRows    = EditorGUILayout.IntField("Inner Corners (rows)", _boardRows);
            _squareMm     = EditorGUILayout.FloatField("Square Size (mm)",    _squareMm);
            _targetFrames = EditorGUILayout.IntField("Target Frame Count",    _targetFrames);

            EditorGUILayout.Space(8);

            // ── Live preview ─────────────────────────────────────────────────
            if (_previewTex != null)
            {
                float aspect = (float)_previewTex.width / _previewTex.height;
                float w = position.width - 24f;
                float h = Mathf.Min(w / aspect, 300f);
                w = h * aspect;
                Rect r = GUILayoutUtility.GetRect(w, h);
                GUI.DrawTexture(r, _previewTex, ScaleMode.ScaleToFit);
            }
            else
            {
                GUILayout.Box("No preview — connect camera first",
                              GUILayout.ExpandWidth(true), GUILayout.Height(120));
            }

            EditorGUILayout.Space(6);

            // Corner status
            bool cornersFoundSnapshot;
            lock (_frameLock) cornersFoundSnapshot = _latestCornersFound;
            var statusStyle = new GUIStyle(EditorStyles.label);
            statusStyle.normal.textColor = cornersFoundSnapshot ? Color.green : Color.red;
            EditorGUILayout.LabelField(
                cornersFoundSnapshot ? "Checkerboard detected" : "Checkerboard not visible",
                statusStyle);

            // ── Capture controls ─────────────────────────────────────────────
            EditorGUILayout.Space(4);
            EditorGUILayout.LabelField(
                $"Captured frames: {_capturedCorners.Count} / {_targetFrames}",
                EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            GUI.enabled = _running && cornersFoundSnapshot;
            if (GUILayout.Button("Capture Frame", GUILayout.Width(120)))
            {
                lock (_frameLock)
                {
                    if (_latestCornersFound && _latestCorners != null)
                        DoCapture(_latestCorners);
                }
            }
            GUI.enabled = _running;
            _autoCapture = GUILayout.Toggle(_autoCapture, "Auto Capture", GUILayout.Width(110));
            GUI.enabled = _capturedCorners.Count > 0;
            if (GUILayout.Button("Clear", GUILayout.Width(60)))
            {
                _capturedCorners.Clear();
                _result = null;
            }
            GUI.enabled = true;
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.Space(8);

            // ── Calibrate ────────────────────────────────────────────────────
            if (_calibrating)
            {
                EditorGUILayout.HelpBox("Calibrating… please wait.", MessageType.Info);
            }
            else
            {
                GUI.enabled = _capturedCorners.Count >= 4 && _frameWidth > 0;
                if (GUILayout.Button("Run Calibration", GUILayout.Height(30)))
                    StartCalibration();
                GUI.enabled = true;
            }

            // ── Results ──────────────────────────────────────────────────────
            if (_result != null)
            {
                EditorGUILayout.Space(8);
                EditorGUILayout.LabelField("Calibration Result", EditorStyles.boldLabel);

                var rmsStyle = new GUIStyle(EditorStyles.label);
                rmsStyle.normal.textColor = _result.rmsError < 1.0 ? Color.green
                                          : _result.rmsError < 2.0 ? Color.yellow : Color.red;
                EditorGUILayout.LabelField(
                    $"RMS Reprojection Error: {_result.rmsError:F4}  (good < 1.0)", rmsStyle);

                EditorGUILayout.LabelField($"fx = {_result.fx:F2}    fy = {_result.fy:F2}");
                EditorGUILayout.LabelField($"cx = {_result.cx:F2}    cy = {_result.cy:F2}");
                EditorGUILayout.LabelField($"k1 = {_result.distCoeffs[0]:F6}    k2 = {_result.distCoeffs[1]:F6}");
                EditorGUILayout.LabelField($"p1 = {_result.distCoeffs[2]:F6}    p2 = {_result.distCoeffs[3]:F6}");
                EditorGUILayout.LabelField($"k3 = {_result.distCoeffs[4]:F6}");

                EditorGUILayout.Space(8);
                _profileName = EditorGUILayout.TextField("Profile Name", _profileName);

                if (GUILayout.Button("Save to StreamingAssets", GUILayout.Height(26)))
                    SaveCalibration();
            }

            EditorGUILayout.EndScrollView();
        }

        // ── Camera start / stop ───────────────────────────────────────────────

        void StartCamera()
        {
            if (_running) return;
            try
            {
                Spin(spinSystemGetInstance(out _hSystem), "SystemGetInstance");

                Spin(spinCameraListCreateEmpty(out IntPtr hList), "CameraListCreate");
                try
                {
                    Spin(spinSystemGetCameras(_hSystem, hList), "GetCameras");
                    Spin(spinCameraListGetSize(hList, out UIntPtr count), "CameraListSize");
                    if ((int)count == 0)  throw new Exception("No Spinnaker cameras found.");
                    if (_cameraIndex >= (int)count)
                        throw new Exception($"Camera index {_cameraIndex} out of range ({count} cameras).");
                    Spin(spinCameraListGet(hList, (UIntPtr)_cameraIndex, out _hCamera), "CameraListGet");
                }
                finally
                {
                    spinCameraListClear(hList);
                    spinCameraListDestroy(hList);
                }

                Spin(spinCameraInit(_hCamera), "CameraInit");
                Spin(spinCameraGetNodeMap(_hCamera, out IntPtr nm), "GetNodeMap");

                SetEnum (nm, "AcquisitionMode", "Continuous");
                SetEnum (nm, "PixelFormat",      "Mono8");
                SetEnum (nm, "ExposureAuto",     "Continuous");
                SetEnum (nm, "GainAuto",         "Continuous");

                Spin(spinCameraBeginAcquisition(_hCamera), "BeginAcquisition");

                // Probe first frame for resolution
                if (OK(spinCameraGetNextImageEx(_hCamera, 2000, out IntPtr hProbe)))
                {
                    spinImageGetWidth (hProbe, out UIntPtr w);
                    spinImageGetHeight(hProbe, out UIntPtr h);
                    _frameWidth  = (int)w;
                    _frameHeight = (int)h;
                    spinImageRelease(hProbe);
                }

                _running       = true;
                _captureThread = new Thread(CaptureLoop) { IsBackground = true, Name = "CalibCapture" };
                _captureThread.Start();
            }
            catch (Exception e)
            {
                Debug.LogError($"[CalibTool] Connect failed: {e.Message}");
                StopCamera();
            }
        }

        void StopCamera()
        {
            _running = false;
            _captureThread?.Join(2000);
            _captureThread = null;

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

        // ── Background capture + corner detection ─────────────────────────────

        void CaptureLoop()
        {
            // Detector parameters initialised once on this thread
            var detectorParams = DetectorParameters.GetDefault();
            var patternSize    = new System.Drawing.Size(_boardCols, _boardRows);

            while (_running)
            {
                int err = spinCameraGetNextImageEx(_hCamera, 500, out IntPtr hRaw);
                if (IsTimeout(err)) continue;
                if (!OK(err)) continue;

                spinImageIsIncomplete(hRaw, out byte incomplete);
                if (incomplete != 0) { spinImageRelease(hRaw); continue; }

                spinImageGetWidth (hRaw, out UIntPtr uw);
                spinImageGetHeight(hRaw, out UIntPtr uh);
                int w = (int)uw, h = (int)uh;

                spinImageGetData(hRaw, out IntPtr dataPtr);
                byte[] pixels = new byte[w * h];
                Marshal.Copy(dataPtr, pixels, 0, pixels.Length);
                spinImageRelease(hRaw);

                // Checkerboard detection
                bool found = false;
                System.Drawing.PointF[] corners = null;

                try
                {
                    using var mat = new Mat(h, w, DepthType.Cv8U, 1);
                    mat.SetTo(pixels);

                    using var cornerVec = new VectorOfPointF();
                    found = CvInvoke.FindChessboardCorners(mat, patternSize, cornerVec,
                                CalibCbType.AdaptiveThresh | CalibCbType.NormalizeImage);

                    if (found)
                    {
                        // Sub-pixel refinement
                        var term = new MCvTermCriteria(30, 0.001);
                        CvInvoke.CornerSubPix(mat, cornerVec,
                            new System.Drawing.Size(11, 11),
                            new System.Drawing.Size(-1, -1), term);
                        corners = cornerVec.ToArray();
                    }
                }
                catch { /* ignore per-frame errors */ }

                lock (_frameLock)
                {
                    _latestPixels       = pixels;
                    _latestCornersFound = found;
                    _latestCorners      = corners;
                }
            }
        }

        // ── Capture a frame ───────────────────────────────────────────────────

        void DoCapture(System.Drawing.PointF[] corners)
        {
            if (_capturedCorners.Count >= _targetFrames) return;
            _capturedCorners.Add((System.Drawing.PointF[])corners.Clone());
            Debug.Log($"[CalibTool] Captured frame {_capturedCorners.Count}/{_targetFrames}");
        }

        // ── Run calibration (background thread) ──────────────────────────────

        void StartCalibration()
        {
            if (_capturedCorners.Count < 4 || _frameWidth == 0 || _calibrating) return;

            // Snapshot data needed by the background thread (don't share live lists)
            var framesCopy  = _capturedCorners.ConvertAll(f => (System.Drawing.PointF[])f.Clone());
            int w           = _frameWidth;
            int h           = _frameHeight;
            int cols        = _boardCols;
            int rows        = _boardRows;
            float squareMm  = _squareMm;
            string profile  = _profileName;

            _calibrating = true;

            new Thread(() =>
            {
                try
                {
                    // 3-D object points (Z=0 plane, unit = mm)
                    var objPt = new MCvPoint3D32f[cols * rows];
                    for (int r = 0; r < rows; r++)
                        for (int c = 0; c < cols; c++)
                            objPt[r * cols + c] = new MCvPoint3D32f(c * squareMm, r * squareMm, 0f);

                    using var objPtsVec = new VectorOfVectorOfPoint3D32F();
                    using var imgPtsVec = new VectorOfVectorOfPointF();

                    foreach (var frame in framesCopy)
                    {
                        using var op = new VectorOfPoint3D32F(objPt);
                        using var ip = new VectorOfPointF(frame);
                        objPtsVec.Push(op);
                        imgPtsVec.Push(ip);
                    }

                    var camMat    = new Matrix<double>(3, 3);
                    var distCoMat = new Matrix<double>(1, 5);
                    camMat.SetIdentity();

                    using var rvecs   = new VectorOfMat();
                    using var tvecs   = new VectorOfMat();
                    var imgSize       = new System.Drawing.Size(w, h);
                    var termCrit      = new MCvTermCriteria(100, 1e-6);

                    double rms = CvInvoke.CalibrateCamera(
                        objPtsVec, imgPtsVec, imgSize,
                        camMat, distCoMat,
                        rvecs, tvecs,
                        CalibType.Default, termCrit);

                    var cal = new CameraCalibration
                    {
                        profileName  = profile,
                        imageWidth   = w,
                        imageHeight  = h,
                        rmsError     = rms,
                        cameraMatrix = new double[]
                        {
                            camMat[0,0], camMat[0,1], camMat[0,2],
                            camMat[1,0], camMat[1,1], camMat[1,2],
                            camMat[2,0], camMat[2,1], camMat[2,2]
                        },
                        distCoeffs = new double[]
                        {
                            distCoMat[0,0], distCoMat[0,1],
                            distCoMat[0,2], distCoMat[0,3],
                            distCoMat[0,4]
                        }
                    };

                    lock (_resultLock)
                        _pendingResult = cal;
                }
                catch (Exception e)
                {
                    Debug.LogError($"[CalibTool] Calibration failed: {e.Message}");
                    _calibrating = false;
                }
            })
            { IsBackground = true, Name = "Calibration" }.Start();
        }

        // ── Save ─────────────────────────────────────────────────────────────

        void SaveCalibration()
        {
            if (_result == null) return;
            _result.profileName = _profileName;

            string dir  = Path.Combine(Application.streamingAssetsPath, "CameraCalibration");
            string path = Path.Combine(dir, _profileName + ".json");
            _result.SaveToJson(path);

            AssetDatabase.Refresh();
            Debug.Log($"[CalibTool] Saved → {path}");
            EditorUtility.DisplayDialog("Saved",
                $"Calibration saved to:\n{path}\n\nRMS error: {_result.rmsError:F4}", "OK");
        }

        // ── Preview texture ───────────────────────────────────────────────────

        void UpdatePreviewTexture(byte[] mono8, int w, int h,
                                  bool cornersFound, System.Drawing.PointF[] corners)
        {
            if (_previewTex == null || _previewTex.width != w || _previewTex.height != h)
            {
                if (_previewTex != null) DestroyImmediate(_previewTex);
                _previewTex = new Texture2D(w, h, TextureFormat.RGB24, false);
            }

            // Mono8 → RGB24
            byte[] rgb = new byte[w * h * 3];
            for (int i = 0; i < w * h; i++)
            {
                byte v = mono8[i];
                rgb[i * 3]     = v;
                rgb[i * 3 + 1] = v;
                rgb[i * 3 + 2] = v;
            }

            // Draw detected corners as coloured dots
            if (cornersFound && corners != null)
            {
                int n = corners.Length;
                for (int ci = 0; ci < n; ci++)
                {
                    // Colour gradient: first corner = green, last = red
                    byte r = (byte)(255 * ci / (n - 1));
                    byte g = (byte)(255 * (n - 1 - ci) / (n - 1));
                    DrawDot(rgb, w, h, (int)corners[ci].X, h - 1 - (int)corners[ci].Y, r, g, 0, 5);
                }
            }

            _previewTex.SetPixelData(rgb, 0);
            _previewTex.Apply(false);
        }

        static void DrawDot(byte[] rgb, int w, int h, int px, int py,
                            byte r, byte g, byte b, int radius)
        {
            for (int dy = -radius; dy <= radius; dy++)
            for (int dx = -radius; dx <= radius; dx++)
            {
                if (dx * dx + dy * dy > radius * radius) continue;
                int x = px + dx, y = py + dy;
                if (x < 0 || x >= w || y < 0 || y >= h) continue;
                int idx = (y * w + x) * 3;
                rgb[idx]     = r;
                rgb[idx + 1] = g;
                rgb[idx + 2] = b;
            }
        }

        // ── GenICam helpers (mirrors SpinnakerCameraCapture) ──────────────────

        static void SetEnum(IntPtr nm, string nodeName, string entryName)
        {
            try
            {
                if (!OK(spinNodeMapGetNode(nm, nodeName, out IntPtr hNode))) return;
                if (!OK(spinEnumerationGetEntryByName(hNode, entryName, out IntPtr hEntry))) return;
                if (!OK(spinEnumerationEntryGetEnumValue(hEntry, out UIntPtr val))) return;
                spinEnumerationSetEnumValue(hNode, val);
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[CalibTool] SetEnum {nodeName}={entryName}: {e.Message}");
            }
        }

        static void Spin(int err, string ctx)
        {
            if (!OK(err)) throw new Exception($"Spinnaker {ctx} failed (err {err})");
        }
    }
}
