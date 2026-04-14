using System;
using System.IO;
using UnityEngine;

namespace DiscVisionDeluxe.Camera
{
    /// <summary>
    /// Stores the camera extrinsics relative to the ground-plane reference tag.
    ///
    /// Coordinate convention (world frame = tag frame):
    ///   Origin : centre of the reference AprilTag
    ///   X      : tag's local X axis (point the tag so this faces down-range)
    ///   Y      : tag's local Y axis (90° right of X on the ground)
    ///   Z      : up (perpendicular to ground, toward camera)
    ///
    /// To convert a detected point from camera space into world space:
    ///   p_world = cameraToWorld * p_camera  (homogeneous, row-major)
    /// </summary>
    [Serializable]
    public class GroundPlaneCalibration
    {
        public string profileName   = "ground_plane";
        public int    referenceTagId;

        // Row-major 4×4 — transforms world (tag) points into camera space
        public double[] worldToCamera = new double[16];

        // Row-major 4×4 — transforms camera points into world (tag) space
        public double[] cameraToWorld = new double[16];

        // Camera height above the ground plane in metres (Z of camera in tag frame)
        public double cameraHeightM;

        // ── Persistence ───────────────────────────────────────────────────────

        public static GroundPlaneCalibration LoadFromJson(string path)
        {
            if (!File.Exists(path))
                throw new FileNotFoundException($"Ground plane calibration not found: {path}");
            return JsonUtility.FromJson<GroundPlaneCalibration>(File.ReadAllText(path));
        }

        public void SaveToJson(string path)
        {
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            File.WriteAllText(path, JsonUtility.ToJson(this, prettyPrint: true));
        }

        /// <summary>Load by profile name from StreamingAssets/CameraCalibration/{name}.json.</summary>
        public static GroundPlaneCalibration LoadProfile(string name)
        {
            string path = Path.Combine(Application.streamingAssetsPath,
                                       "CameraCalibration", name + ".json");
            return File.Exists(path) ? LoadFromJson(path) : null;
        }

        public override string ToString() =>
            $"{profileName}  tag={referenceTagId}  height={cameraHeightM:F3}m";
    }
}
