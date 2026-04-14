using System;
using System.Collections.Generic;
using Emgu.CV;
using Emgu.CV.Aruco;
using Emgu.CV.CvEnum;
using Emgu.CV.Structure;
using Emgu.CV.Util;
using UnityEngine;

namespace DiscVisionDeluxe.Camera
{
    /// <summary>
    /// Detects AprilTag 36H11 markers in a Mono8 frame and solves 6-DOF pose
    /// for each detected tag using the supplied camera intrinsics.
    ///
    /// Not a MonoBehaviour — instantiate from SpinnakerCameraCapture.
    /// All methods are called from the background capture thread.
    /// Do NOT call any Unity API from inside this class.
    /// </summary>
    public class AprilTagDetector : IDisposable
    {
        // ── Public result type ───────────────────────────────────────────────

        public readonly struct TagDetection
        {
            /// <summary>AprilTag ID</summary>
            public readonly int   id;
            /// <summary>Corner pixels in image space (TL, TR, BR, BL)</summary>
            public readonly System.Drawing.PointF[] corners;
            /// <summary>Rotation vector (Rodrigues) from solvePnP</summary>
            public readonly double[] rvec;
            /// <summary>Translation vector (metres) from solvePnP</summary>
            public readonly double[] tvec;

            public TagDetection(int id, System.Drawing.PointF[] corners,
                                double[] rvec, double[] tvec)
            {
                this.id      = id;
                this.corners = corners;
                this.rvec    = rvec;
                this.tvec    = tvec;
            }
        }

        // ── Config ───────────────────────────────────────────────────────────

        /// <summary>Physical side length of the AprilTag (metres).</summary>
        public float tagSizeM = 0.165f;

        // ── Private ──────────────────────────────────────────────────────────

        readonly Dictionary         _dictionary;
        readonly DetectorParameters _detectorParams;

        // Camera intrinsics — set via SetCalibration()
        Matrix<double> _cameraMatrix;
        Matrix<double> _distCoeffs;
        bool           _hasCalibration;

        // 3-D object points for one tag (metres, Z=0 plane)
        readonly MCvPoint3D32f[] _tagPoints3D;

        // ── Construction ─────────────────────────────────────────────────────

        /// <param name="tagSizeM">Physical side length of the AprilTag in metres.</param>
        public AprilTagDetector(float tagSizeM = 0.165f)
        {
            this.tagSizeM = tagSizeM;

            // DICT_APRILTAG_36H11 = 20 (fixed OpenCV constant — enum name varies between Emgu CV versions)
            _dictionary = new Dictionary((Dictionary.PredefinedDictionaryName)20);

            // In Emgu CV 4.12, DetectorParameters is a struct with PascalCase fields and a static
            // GetDefault() factory. Use GetDefault() so all fields are set to OpenCV defaults
            // (including MarkerBorderBits=1 — OpenCV rejects 0 with an assert).
            _detectorParams = DetectorParameters.GetDefault();

            float h = tagSizeM * 0.5f;
            _tagPoints3D = new MCvPoint3D32f[]
            {
                new MCvPoint3D32f(-h,  h, 0),   // TL
                new MCvPoint3D32f( h,  h, 0),   // TR
                new MCvPoint3D32f( h, -h, 0),   // BR
                new MCvPoint3D32f(-h, -h, 0),   // BL
            };

            // Identity intrinsics until calibration is loaded
            _cameraMatrix = new Matrix<double>(3, 3);
            _cameraMatrix.SetIdentity();
            _distCoeffs = new Matrix<double>(1, 5);
        }

        // ── Calibration ──────────────────────────────────────────────────────

        /// <summary>
        /// Supply camera intrinsics. Call once after loading calibration file.
        /// fx, fy = focal lengths in pixels. cx, cy = principal point.
        /// k1..k4 = distortion coefficients (fisheye or standard model).
        /// </summary>
        public void SetCalibration(double fx, double fy, double cx, double cy,
                                   double k1, double k2, double p1, double p2, double k3 = 0)
        {
            _cameraMatrix[0, 0] = fx;
            _cameraMatrix[1, 1] = fy;
            _cameraMatrix[0, 2] = cx;
            _cameraMatrix[1, 2] = cy;
            _cameraMatrix[2, 2] = 1.0;

            _distCoeffs[0, 0] = k1;
            _distCoeffs[0, 1] = k2;
            _distCoeffs[0, 2] = p1;
            _distCoeffs[0, 3] = p2;
            _distCoeffs[0, 4] = k3;

            _hasCalibration = true;
        }

        // ── Detection ────────────────────────────────────────────────────────

        /// <summary>
        /// Detect AprilTags in a Mono8 frame and return pose-solved detections.
        /// Safe to call from a background thread.
        /// Returns an empty list if no tags are found or calibration is missing.
        /// </summary>
        public List<TagDetection> Detect(byte[] mono8, int width, int height)
        {
            var results = new List<TagDetection>();

            using var mat = new Mat(height, width, DepthType.Cv8U, 1);
            mat.SetTo(mono8);

            using var ids     = new VectorOfInt();
            using var corners = new VectorOfVectorOfPointF();
            using var rejected = new VectorOfVectorOfPointF();

            ArucoInvoke.DetectMarkers(mat, _dictionary, corners, ids, _detectorParams, rejected);

            int count = ids.Size;
            if (count == 0) return results;

            for (int i = 0; i < count; i++)
            {
                int tagId = ids[i];
                var tagCorners = corners[i].ToArray();

                // Build 2-D image points
                var imagePoints = new System.Drawing.PointF[4];
                for (int c = 0; c < 4; c++)
                    imagePoints[c] = tagCorners[c];

                // solvePnP — only if calibration has been supplied
                double[] rvec = null;
                double[] tvec = null;
                if (_hasCalibration)
                {
                    using var objPts    = new VectorOfPoint3D32F(_tagPoints3D);
                    using var imgPts    = new VectorOfPointF(imagePoints);
                    using var rvecOut   = new Matrix<double>(3, 1);
                    using var tvecOut   = new Matrix<double>(3, 1);

                    bool solved = CvInvoke.SolvePnP(
                        objPts, imgPts,
                        _cameraMatrix, _distCoeffs,
                        rvecOut, tvecOut,
                        false, SolvePnpMethod.Iterative);

                    if (solved)
                    {
                        rvec = new double[] { rvecOut[0,0], rvecOut[1,0], rvecOut[2,0] };
                        tvec = new double[] { tvecOut[0,0], tvecOut[1,0], tvecOut[2,0] };
                    }
                }

                results.Add(new TagDetection(tagId, imagePoints, rvec, tvec));
            }

            return results;
        }

        // ── IDisposable ──────────────────────────────────────────────────────

        public void Dispose()
        {
            _dictionary?.Dispose();
            _cameraMatrix?.Dispose();
            _distCoeffs?.Dispose();
        }
    }
}
