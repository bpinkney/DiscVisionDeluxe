using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace DiscVisionDeluxe.Camera
{
    /// <summary>
    /// CAM-5 — Live disc tracking pipeline.
    ///
    /// Converts AprilTag detections from SpinnakerCameraCapture into KFMeasurements,
    /// feeds them into a DiscKalmanFilter, and fires onThrowComplete when a throw
    /// is resolved — the same DiscInitState that DiscVisualizer.LaunchDfisX() expects.
    ///
    /// Coordinate mapping (camera → KF world frame, no ground-plane calibration):
    ///   KF  X (forward)  = camera Z (depth)
    ///   KF  Y (right)    = camera X
    ///   KF  Z (up)       = -camera Y
    ///
    /// Wire up in the scene:
    ///   1. Assign SpinnakerCameraCapture.
    ///   2. Set Disc Tag Id to the AprilTag ID on the disc.
    ///   3. Connect onThrowComplete → DiscVisualizer.LaunchDfisX (or LaunchFromInitState).
    ///   4. Enter Play mode — tracking starts automatically when the disc tag is seen.
    /// </summary>
    public class LiveDiscTracker : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("References")]
        public SpinnakerCameraCapture cameraCapture;

        [Tooltip("Ground plane calibration profile name (from GroundPlaneCalibrator). " +
                 "Leave empty to use a raw camera-space fallback mapping (horizontal camera only).")]
        public string groundPlaneProfile = "ground_plane";

        [Header("Disc Tag")]
        [Tooltip("AprilTag ID attached to the disc.")]
        public int discTagId = 0;

        [Tooltip("Disc mold to pass into the physics sim after tracking.")]
        public DiscIndex discMold = DiscIndex.MIDRANGE;

        [Header("Tracking")]
        [Tooltip("Milliseconds without a detection before detection-lost is signalled.\n" +
                 "Must exceed (max queue depth × detection time) so the queue drains before timeout.\n" +
                 "At 30ms/frame × 60 frames = 1800ms max drain. 2000ms is safe.")]
        public float detectionTimeoutMs = 2000f;

        [Header("Kalman Filter")]
        public DiscKalmanFilter.KFParams kfParams = new DiscKalmanFilter.KFParams
        {
            // Relaxed priming for live camera — a fast throw may only produce 3-10 detected frames.
            // Force prime after 3 frames instead of 8.
            primeMaxEntries = 3,
            primeCount      = 3,
            primeMinVar     = 0.5,  // lower than CSV default — fewer samples means more noise
        };

        [Header("Events")]
        [Tooltip("Fired on the main thread when a throw is resolved. " +
                 "Wire to DiscVisualizer.LaunchDfisX / LaunchFromInitState.")]
        public UnityEvent<DiscInitState> onThrowComplete = new UnityEvent<DiscInitState>();

        [Tooltip("Fired every KF step while tracking (optional — for debug visualisation).")]
        public UnityEvent<KFState> onNewState = new UnityEvent<KFState>();

        // ── Read-only status ─────────────────────────────────────────────────

        [Header("Status (read-only)")]
        [SerializeField] TrackingState _state = TrackingState.Idle;
        [SerializeField] int           _measurementsThisThrow;
        [SerializeField] float         _msSinceLastDetection;

        public enum TrackingState { Idle, Tracking, Processing, Complete }

        // ── Private ──────────────────────────────────────────────────────────

        DiscKalmanFilter       _kf;
        GroundPlaneCalibration _groundPlane;
        ulong            _lastKFStepNs;
        ulong            _lastDetectionNs;
        uint             _frameCounter;

        // ── Unity lifecycle ──────────────────────────────────────────────────

        void OnEnable()
        {
            _kf = new DiscKalmanFilter(kfParams);

            if (!string.IsNullOrEmpty(groundPlaneProfile))
            {
                _groundPlane = GroundPlaneCalibration.LoadProfile(groundPlaneProfile);
                if (_groundPlane != null)
                    UnityEngine.Debug.Log($"[LiveTracker] Ground plane loaded: {_groundPlane}");
                else
                    UnityEngine.Debug.LogWarning($"[LiveTracker] Ground plane profile '{groundPlaneProfile}' not found — " +
                                                 "using raw camera-space fallback (horizontal camera only).");
            }

            if (cameraCapture != null)
                cameraCapture.onTagsDetected.AddListener(OnTagsDetected);
            else
                UnityEngine.Debug.LogError("[LiveTracker] SpinnakerCameraCapture not assigned.");
        }

        void OnDisable()
        {
            if (cameraCapture != null)
                cameraCapture.onTagsDetected.RemoveListener(OnTagsDetected);
        }

        void Update()
        {
            if (_state == TrackingState.Idle || _state == TrackingState.Complete)
                return;

            ulong nowNs = cameraCapture.ElapsedCaptureNs;

            // ── Detection timeout ────────────────────────────────────────────
            if (_state == TrackingState.Tracking)
            {
                float msSinceLast = (nowNs - _lastDetectionNs) * 1e-6f;
                _msSinceLastDetection = msSinceLast;

                if (msSinceLast > detectionTimeoutMs)
                {
                    UnityEngine.Debug.Log($"[LiveTracker] Detection lost after {_measurementsThisThrow} frames. Processing...");
                    _kf.SignalDetectionLost();
                    _state = TrackingState.Processing;
                }
            }

            // ── Step KF ──────────────────────────────────────────────────────
            if (_state == TrackingState.Processing || _state == TrackingState.Tracking)
            {
                ulong stepNs = (ulong)(kfParams.predDtS * 1e9);

                while (_lastKFStepNs + stepNs <= nowNs &&
                       _kf.CurrentStage != DiscKalmanFilter.Stage.Complete)
                {
                    bool newIdeal = _kf.Step(_lastKFStepNs);
                    _lastKFStepNs += stepNs;

                    if (_kf.CurrentState != null)
                        onNewState.Invoke(_kf.CurrentState);

                    if (newIdeal && _kf.IdealState != null)
                    {
                        // Log progress
                    }
                }

                if (_kf.CurrentStage == DiscKalmanFilter.Stage.Complete)
                    FireComplete();
            }
        }

        // ── Tag detection listener (main thread) ─────────────────────────────

        void OnTagsDetected(List<AprilTagDetector.TagDetection> detections)
        {
            if (_state == TrackingState.Complete) return;

            foreach (var d in detections)
            {
                if (d.id != discTagId || d.rvec == null || d.tvec == null)
                    continue;

                // Use the timestamp recorded at capture time (background thread) so that
                // multiple detections drained in one Update() get distinct timestamps.
                ulong nowNs = cameraCapture.LastDetectionTimestampNs;

                // First detection → reset and start fresh
                if (_state == TrackingState.Idle)
                {
                    ResetKF(nowNs);
                    UnityEngine.Debug.Log($"[LiveTracker] Disc tag {discTagId} acquired. Tracking started.");
                }

                var meas = BuildMeasurement(d, nowNs);
                if (meas == null)
                {
                    UnityEngine.Debug.LogWarning($"[LiveTracker] Frame rejected (tvec out of range): " +
                        $"cam=({d.tvec[0]:F2}, {d.tvec[1]:F2}, {d.tvec[2]:F2})");
                    continue;
                }

                // Log first 5 frames so we can verify pose values look sane
                if (_measurementsThisThrow < 5)
                    UnityEngine.Debug.Log($"[LiveTracker] Frame {_measurementsThisThrow}: " +
                        $"tvec=({d.tvec[0]:F3}, {d.tvec[1]:F3}, {d.tvec[2]:F3})  " +
                        $"kf=({meas.linX:F3}, {meas.linY:F3}, {meas.linZ:F3})");

                _kf.AddMeasurement(meas);

                _lastDetectionNs    = nowNs;
                _lastKFStepNs       = nowNs;
                _measurementsThisThrow++;
                _state = TrackingState.Tracking;
                _msSinceLastDetection = 0f;
                break; // only process the first matching tag per frame
            }
        }

        // ── KFMeasurement construction ───────────────────────────────────────

        /// <summary>Returns null if tvec contains NaN or implausibly large values.</summary>
        KFMeasurement BuildMeasurement(AprilTagDetector.TagDetection d, ulong timestampNs)
        {
            double camX = d.tvec[0];
            double camY = d.tvec[1];
            double camZ = d.tvec[2];

            // Reject degenerate poses
            if (double.IsNaN(camX) || double.IsNaN(camY) || double.IsNaN(camZ) ||
                Math.Abs(camX) > 50 || Math.Abs(camY) > 50 || Math.Abs(camZ) > 50)
                return null;

            double kfX, kfY, kfZ;
            double[,] R;

            if (_groundPlane != null)
            {
                // Transform camera-space position into world frame using ground plane calibration.
                // cameraToWorld is row-major 4×4: p_world = cameraToWorld * p_camera
                double[] ctw = _groundPlane.cameraToWorld;
                kfX = ctw[0]*camX + ctw[1]*camY + ctw[2]*camZ + ctw[3];
                kfY = ctw[4]*camX + ctw[5]*camY + ctw[6]*camZ + ctw[7];
                kfZ = ctw[8]*camX + ctw[9]*camY + ctw[10]*camZ + ctw[11];

                // Rotate the disc's rotation matrix into world frame: R_world = R_cam_to_world * R_disc_cam
                double[,] Rctw = new double[3, 3]
                {
                    { ctw[0], ctw[1], ctw[2] },
                    { ctw[4], ctw[5], ctw[6] },
                    { ctw[8], ctw[9], ctw[10] }
                };
                double[,] Rdisc = RodriguesToMatrix(d.rvec);
                R = MatMul3x3(Rctw, Rdisc);
            }
            else
            {
                // Fallback: raw camera-space mapping (assumes horizontal forward-facing camera).
                // Camera frame: X=right, Y=down, Z=depth(forward) → KF: X=forward, Y=right, Z=up
                kfX =  camZ;
                kfY =  camX;
                kfZ = -camY;
                R = RodriguesToMatrix(d.rvec);
            }

            // Angle extraction — mirrors angle_hyzer_pitch_spin_from_R in dvd_DvisEst_estimate.cpp
            double hyzer = Math.Asin(Clamp(R[1, 2], -1.0, 1.0));
            double pitch = Math.Asin(Clamp(R[0, 2], -1.0, 1.0));
            double spin  = Math.Atan2(R[0, 1], R[0, 0]);

            return new KFMeasurement
            {
                timestampNs = timestampNs,
                frameId     = _frameCounter++,
                linX        = kfX,
                linY        = kfY,
                linZ        = kfZ,
                angHyzer    = hyzer,
                angPitch    = pitch,
                angSpin     = spin,
                discIndex   = discMold,
                player      = 0
            };
        }

        // ── Helpers ──────────────────────────────────────────────────────────

        void ResetKF(ulong nowNs)
        {
            _kf.Reset();
            _measurementsThisThrow = 0;
            _lastKFStepNs          = nowNs;
            _lastDetectionNs       = nowNs;
            _frameCounter          = 0;
        }

        void FireComplete()
        {
            _state = TrackingState.Complete;

            if (_kf.HasIdealState)
            {
                var init = DiscInitState.FromKFState(_kf.IdealState);
                init.discMold = discMold;

                var pos = init.linearPositionM;
                var vel = init.linearVelocityMs;
                float speedKph = vel.magnitude * 3.6f;

                UnityEngine.Debug.Log(
                    $"[LiveTracker] Throw complete. " +
                    $"Frames={_measurementsThisThrow}  " +
                    $"Speed={speedKph:F1} kph  " +
                    $"Pos=({pos.x:F2}, {pos.y:F2}, {pos.z:F2})  " +
                    $"Vel=({vel.x:F2}, {vel.y:F2}, {vel.z:F2})");

                // Guard against NaN / physically impossible values (bad calibration artifact)
                bool posNaN = float.IsNaN(pos.x) || float.IsNaN(pos.y) || float.IsNaN(pos.z);
                bool velNaN = float.IsNaN(vel.x) || float.IsNaN(vel.y) || float.IsNaN(vel.z);
                if (posNaN || velNaN || speedKph > 400f || speedKph < 5f)
                {
                    UnityEngine.Debug.LogWarning(
                        $"[LiveTracker] Discarding throw — speed {speedKph:F1} kph is outside [5, 400] kph range. " +
                        "Throw the disc through the camera frame rather than holding it in view.");
                }
                else
                {
                    // Position is in camera space (no ground-plane calibration applied yet).
                    // The sim requires Z > 0 to run; clamp to a typical disc release height.
                    // Once ground-plane calibration (CAM-4) is wired in, remove this override.
                    if (init.linearPositionM.z <= 0f)
                        init.linearPositionM.z = 1.0f;

                    onThrowComplete.Invoke(init);
                }
            }
            else
            {
                UnityEngine.Debug.LogWarning("[LiveTracker] KF complete but no ideal state — not enough frames.");
            }

            // Auto-reset so next throw can be captured
            _state = TrackingState.Idle;
        }

        static double[,] MatMul3x3(double[,] A, double[,] B)
        {
            var C = new double[3, 3];
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    for (int k = 0; k < 3; k++)
                        C[i, j] += A[i, k] * B[k, j];
            return C;
        }

        static double Clamp(double v, double min, double max) =>
            v < min ? min : v > max ? max : v;

        /// <summary>Rodrigues rotation vector → 3×3 rotation matrix.</summary>
        static double[,] RodriguesToMatrix(double[] rvec)
        {
            double theta = Math.Sqrt(rvec[0]*rvec[0] + rvec[1]*rvec[1] + rvec[2]*rvec[2]);

            if (theta < 1e-10)
                return new double[,] { { 1,0,0 }, { 0,1,0 }, { 0,0,1 } };

            double x = rvec[0] / theta, y = rvec[1] / theta, z = rvec[2] / theta;
            double c = Math.Cos(theta), s = Math.Sin(theta), t = 1.0 - c;

            return new double[,]
            {
                { t*x*x + c,     t*x*y - s*z,   t*x*z + s*y },
                { t*x*y + s*z,   t*y*y + c,     t*y*z - s*x },
                { t*x*z - s*y,   t*y*z + s*x,   t*z*z + c   }
            };
        }
    }
}
