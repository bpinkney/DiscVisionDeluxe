using UnityEngine;
using Unity.Mathematics;

namespace DfisX
{
    [CreateAssetMenu(fileName = "NewDiscModel", menuName = "DfisX/Disc Model")]
    public class DiscModel : ScriptableObject
    {
        [Header("Identity")]
        public string moldName       = "Unknown";
        public string manufacturer   = "Unknown";
        public string discType       = "Driver";

        [Tooltip("Overstable / Stable / Understable")]
        public string stability      = "Stable";

        [Tooltip("Concave / Convex / Flat")]
        public string rimCamberShape = "Flat";

        [Header("Physical Dimensions")]

        [Tooltip("Mass (kg). Typical range 0.150 - 0.176")]
        [Range(0.100f, 0.200f)]
        public float mass            = 0.175f;

        [Tooltip("Disc radius (m). Typical range 0.104 - 0.110")]
        [Range(0.095f, 0.120f)]
        public float radius          = 0.106f;

        [Tooltip("Rim width from inner cavity edge to disc edge (m)")]
        [Range(0.005f, 0.030f)]
        public float rimWidth        = 0.020f;

        [Tooltip("Total thickness from bottom of rim to top of dome (m)")]
        [Range(0.010f, 0.040f)]
        public float thickness       = 0.017f;

        [Tooltip("Cavity depth measured at the edge of the cavity (m)")]
        [Range(0.005f, 0.020f)]
        public float rimDepth        = 0.012f;

        [Tooltip("Height of the lower rim camber (m)")]
        [Range(0.000f, 0.015f)]
        public float rimCamberHeight = 0.007f;

        [Tooltip("Height of the upper dome camber above the rim (m)")]
        [Range(0.000f, 0.015f)]
        public float domeHeight      = 0.003f;

        // ---- Convenience (editor/debug only) ----
        public float EdgeHeight => Mathf.Max(0f, thickness - rimCamberHeight - domeHeight);
        public float Iz         => 0.5f  * mass * radius * radius;
        public float Ixy        => 0.25f * mass * radius * radius;

        // ---- Blittable conversion ----
        // Builds DiscModelData directly here — no dependency on FromScriptableObject()
        public DiscModelData ToBlittable()
        {
            int camber = 0;
            if (rimCamberShape == "Concave") camber = 1;
            else if (rimCamberShape == "Convex") camber = 2;

            return new DiscModelData
            {
                mass              = mass,
                radius            = radius,
                rimWidth          = rimWidth,
                thickness         = thickness,
                rimDepth          = rimDepth,
                rimCamberHeight   = rimCamberHeight,
                domeHeight        = domeHeight,
                rimCamberShapeInt = camber
            };
        }
    }
}
