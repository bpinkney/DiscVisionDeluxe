using System;
using System.IO;
using UnityEngine;

namespace DiscVisionDeluxe.Camera
{
    /// <summary>
    /// Stores and serializes camera intrinsics produced by CameraCalibrationEditor.
    /// Files live in StreamingAssets/CameraCalibration/*.json.
    /// Call SetCalibration() on AprilTagDetector after loading.
    /// </summary>
    [Serializable]
    public class CameraCalibration
    {
        public string profileName = "default";
        public int    imageWidth;
        public int    imageHeight;

        // Row-major 3×3: [ fx, 0, cx, 0, fy, cy, 0, 0, 1 ]
        public double[] cameraMatrix = new double[9];

        // Standard distortion: [k1, k2, p1, p2, k3]
        public double[] distCoeffs = new double[5];

        // Reprojection RMS error from calibration run
        public double rmsError;

        // ── Convenience accessors ─────────────────────────────────────────────

        public double fx => cameraMatrix[0];
        public double fy => cameraMatrix[4];
        public double cx => cameraMatrix[2];
        public double cy => cameraMatrix[5];

        // ── Persistence ───────────────────────────────────────────────────────

        public static CameraCalibration LoadFromJson(string path)
        {
            if (!File.Exists(path))
                throw new FileNotFoundException($"Calibration file not found: {path}");

            return JsonUtility.FromJson<CameraCalibration>(File.ReadAllText(path));
        }

        public void SaveToJson(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            File.WriteAllText(path, JsonUtility.ToJson(this, prettyPrint: true));
        }

        /// <summary>
        /// Load by profile name from StreamingAssets/CameraCalibration/{name}.json.
        /// Returns null if not found.
        /// </summary>
        public static CameraCalibration LoadProfile(string name)
        {
            string path = Path.Combine(Application.streamingAssetsPath,
                                       "CameraCalibration", name + ".json");
            return File.Exists(path) ? LoadFromJson(path) : null;
        }

        public override string ToString() =>
            $"{profileName} [{imageWidth}×{imageHeight}] fx={fx:F1} fy={fy:F1} " +
            $"cx={cx:F1} cy={cy:F1} rms={rmsError:F4}";
    }
}
