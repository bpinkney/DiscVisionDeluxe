using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace DiscVisionDeluxe.Camera
{
    /// <summary>
    /// CAM-4 — Ground plane calibration.
    ///
    /// Place a single AprilTag flat on the ground at the tee (world origin).
    /// Orient the tag so its X axis points down-range (flight direction).
    ///
    /// Setup:
    ///   1. Assign SpinnakerCameraCapture in the Inspector.
    ///   2. Set Reference Tag Id to match the tag on the ground.
    ///   3. Set Camera Calibration Profile to the name saved from CAM-3.
    ///   4. Enter Play mode — the calibration will be loaded automatically.
    ///   5. Right-click the component → "Start Ground Calibration",
    ///      or call StartGroundCalibration() from another script / UI button.
    ///   6. Hold the tag visible until the frame count reaches averageFrames.
    ///   7. Calibration is saved automatically to:
    ///      StreamingAssets/CameraCalibration/{outputProfile}.json
    /// </summary>
    public class GroundPlaneCalibrator : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("References")]
        public SpinnakerCameraCapture cameraCapture;

        [Header("Reference Tag")]
        [Tooltip("AprilTag ID placed flat on the ground at the tee (world origin).")]
        public int referenceTagId = 0;

        [Header("Calibration")]
        [Tooltip("Camera calibration profile name (from CAM-3) loaded at Start.")]
        public string cameraCalibrationProfile = "camera_calibration";

        [Tooltip("Number of frames to average for a stable result. 30 is plenty.")]
        public int averageFrames = 30;

        [Tooltip("Output profile name written to StreamingAssets/CameraCalibration/.")]
        public string outputProfile = "ground_plane";

        [Header("Status (read-only)")]
        [SerializeField] bool   _isCalibrating;
        [SerializeField] int    _framesCollected;
        [SerializeField] string _statusMessage = "Idle";

        // ── Private ──────────────────────────────────────────────────────────

        List<double[]> _rvecs;
        List<double[]> _tvecs;

        // ── Unity lifecycle ──────────────────────────────────────────────────

        void Start()
        {
            if (cameraCapture == null)
            {
                Debug.LogError("[GroundCalib] SpinnakerCameraCapture not assigned.");
                return;
            }

            // Load camera intrinsics so solvePnP produces real distances
            if (!string.IsNullOrEmpty(cameraCalibrationProfile))
            {
                var cal = CameraCalibration.LoadProfile(cameraCalibrationProfile);
                if (cal != null)
                {
                    cameraCapture.LoadCalibration(cal);
                    _statusMessage = $"Camera calibration loaded: {cal.profileName}";
                }
                else
                {
                    Debug.LogWarning($"[GroundCalib] Camera calibration '{cameraCalibrationProfile}' not found — " +
                                     "pose accuracy will be degraded.");
                    _statusMessage = "WARNING: no camera calibration";
                }
            }
        }

        void OnDestroy()
        {
            if (_isCalibrating && cameraCapture != null)
                cameraCapture.onTagsDetected.RemoveListener(OnTagsDetected);
        }

        // ── Public API ───────────────────────────────────────────────────────

        /// <summary>
        /// Begin collecting frames. Finishes automatically once averageFrames are collected.
        /// Can also be triggered via right-click → "Start Ground Calibration".
        /// </summary>
        [ContextMenu("Start Ground Calibration")]
        public void StartGroundCalibration()
        {
            if (_isCalibrating)
            {
                Debug.LogWarning("[GroundCalib] Already calibrating.");
                return;
            }
            if (cameraCapture == null || !cameraCapture.IsCapturing)
            {
                Debug.LogError("[GroundCalib] Camera is not running — enter Play mode first.");
                return;
            }

            _rvecs           = new List<double[]>(averageFrames);
            _tvecs           = new List<double[]>(averageFrames);
            _framesCollected = 0;
            _isCalibrating   = true;
            _statusMessage   = $"Calibrating… 0/{averageFrames}";

            cameraCapture.onTagsDetected.AddListener(OnTagsDetected);
            Debug.Log($"[GroundCalib] Started — keep tag {referenceTagId} visible.");
        }

        [ContextMenu("Cancel Calibration")]
        public void CancelCalibration()
        {
            if (!_isCalibrating) return;
            _isCalibrating = false;
            cameraCapture?.onTagsDetected.RemoveListener(OnTagsDetected);
            _statusMessage = "Cancelled";
            Debug.Log("[GroundCalib] Calibration cancelled.");
        }

        // ── Tag detection listener (main thread) ─────────────────────────────

        void OnTagsDetected(List<AprilTagDetector.TagDetection> detections)
        {
            if (!_isCalibrating) return;

            foreach (var d in detections)
            {
                if (d.id != referenceTagId || d.rvec == null || d.tvec == null)
                    continue;

                _rvecs.Add((double[])d.rvec.Clone());
                _tvecs.Add((double[])d.tvec.Clone());
                _framesCollected++;
                _statusMessage = $"Calibrating… {_framesCollected}/{averageFrames}";

                if (_framesCollected >= averageFrames)
                {
                    FinishCalibration();
                    return;
                }
            }
        }

        // ── Compute and save ─────────────────────────────────────────────────

        void FinishCalibration()
        {
            _isCalibrating = false;
            cameraCapture.onTagsDetected.RemoveListener(OnTagsDetected);

            // Average rvec and tvec across all collected frames
            double[] avgRvec = new double[3];
            double[] avgTvec = new double[3];

            for (int i = 0; i < 3; i++)
            {
                foreach (var rv in _rvecs) avgRvec[i] += rv[i];
                foreach (var tv in _tvecs) avgTvec[i] += tv[i];
                avgRvec[i] /= _rvecs.Count;
                avgTvec[i] /= _tvecs.Count;
            }

            // Rodrigues → 3×3 rotation matrix
            double[,] R = RodriguesToMatrix(avgRvec);

            // worldToCamera = [R | t; 0 0 0 1]  (tag frame → camera frame)
            double[] wtc = new double[16]
            {
                R[0,0], R[0,1], R[0,2], avgTvec[0],
                R[1,0], R[1,1], R[1,2], avgTvec[1],
                R[2,0], R[2,1], R[2,2], avgTvec[2],
                0,      0,      0,      1
            };

            // cameraToWorld = [R^T | -R^T * t; 0 0 0 1]  (camera frame → tag/world frame)
            double tx = -(R[0,0]*avgTvec[0] + R[1,0]*avgTvec[1] + R[2,0]*avgTvec[2]);
            double ty = -(R[0,1]*avgTvec[0] + R[1,1]*avgTvec[1] + R[2,1]*avgTvec[2]);
            double tz = -(R[0,2]*avgTvec[0] + R[1,2]*avgTvec[1] + R[2,2]*avgTvec[2]);

            double[] ctw = new double[16]
            {
                R[0,0], R[1,0], R[2,0], tx,
                R[0,1], R[1,1], R[2,1], ty,
                R[0,2], R[1,2], R[2,2], tz,
                0,      0,      0,      1
            };

            // Camera height above ground = Z of camera position in tag frame
            // (tag Z axis points up when tag is flat on the ground)
            double cameraHeightM = tz;

            var result = new GroundPlaneCalibration
            {
                profileName    = outputProfile,
                referenceTagId = referenceTagId,
                worldToCamera  = wtc,
                cameraToWorld  = ctw,
                cameraHeightM  = cameraHeightM
            };

            string path = Path.Combine(Application.streamingAssetsPath,
                                       "CameraCalibration", outputProfile + ".json");
            result.SaveToJson(path);

            _statusMessage = $"Done! Height={cameraHeightM:F3}m  Saved: {outputProfile}.json";
            Debug.Log($"[GroundCalib] {_statusMessage}");

#if UNITY_EDITOR
            UnityEditor.AssetDatabase.Refresh();
#endif
        }

        // ── Rodrigues formula ────────────────────────────────────────────────

        /// <summary>
        /// Converts a Rodrigues rotation vector to a 3×3 rotation matrix.
        /// This is the same formula used internally by OpenCV's Rodrigues().
        /// </summary>
        static double[,] RodriguesToMatrix(double[] rvec)
        {
            double theta = Math.Sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);

            if (theta < 1e-10)
                return new double[,] { { 1,0,0 }, { 0,1,0 }, { 0,0,1 } };

            double x   = rvec[0] / theta;
            double y   = rvec[1] / theta;
            double z   = rvec[2] / theta;
            double cos = Math.Cos(theta);
            double sin = Math.Sin(theta);
            double t   = 1.0 - cos;

            return new double[,]
            {
                { t*x*x + cos,     t*x*y - sin*z, t*x*z + sin*y },
                { t*x*y + sin*z,   t*y*y + cos,   t*y*z - sin*x },
                { t*x*z - sin*y,   t*y*z + sin*x, t*z*z + cos   }
            };
        }
    }
}
