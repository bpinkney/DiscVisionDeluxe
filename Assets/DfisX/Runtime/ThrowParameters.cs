using UnityEngine;

namespace DfisX
{
    /// <summary>
    /// Initial throw conditions output by dvd_DvisEst (the Kalman Filter).
    /// This is the handoff struct between the vision system and DfisX.
    ///
    /// Maps directly to the key=value string printed by dvd_DvisEst:
    ///   posx:0.115,posy:-0.516,posz:2.103,velx:18.696,...
    /// </summary>
    [System.Serializable]
    public class ThrowParameters
    {
        [Header("Position (m, world frame)")]
        public Vector3 position = new Vector3(0f, 0f, 1.5f);

        [Header("Velocity (m/s, world frame)")]
        public Vector3 velocity = new Vector3(22f, 0f, 0f);

        [Header("Orientation")]
        [Tooltip("Hyzer angle (rad). Positive = disc tilted right for RHBH.")]
        public float hyzer = 0f;

        [Tooltip("Pitch / nose angle (rad). Positive = nose up.")]
        public float pitch = 0f;

        [Header("Spin")]
        [Tooltip("Spin rate (rad/s). Negative = RHBH clockwise from above.")]
        public float spinRate = -75f;

        [Header("Wobble")]
        [Tooltip("Wobble magnitude [0, 1]. 0 = no wobble, 1 = extreme wobble.")]
        [Range(0f, 1f)]
        public float wobble = 0f;

        [Header("Disc")]
        [Tooltip("Which disc mold was detected.")]
        public DiscLayoutIndex discIndex = DiscLayoutIndex.DRIVER;

        /// <summary>
        /// Optional direct reference to a DiscModel asset.
        /// When set, DiscFlightSimulator uses this instead of the discIndex → library lookup.
        /// Set by ThrowParameterPanelController so the dropdown selection is respected exactly.
        /// Not serialized — runtime only.
        /// </summary>
        [System.NonSerialized]
        public DiscModel directModel;

        // ---- Derived ----

        /// <summary>
        /// Compute the disc normal unit vector from hyzer and pitch.
        /// Matches the frame used by dvd_DvisEst:
        ///   X = forward, Y = right, Z = up.
        /// </summary>
        public Vector3 DiscNormal()
        {
            float x = Mathf.Sin(-pitch) * Mathf.Cos(hyzer);
            float y = Mathf.Sin(hyzer)  * Mathf.Cos(pitch);
            float z = Mathf.Cos(pitch)  * Mathf.Cos(hyzer);
            return new Vector3(x, y, z).normalized;
        }

        // ---- Parsing ----

        /// <summary>
        /// Parse the comma-delimited key:value string printed by dvd_DvisEst stdout.
        /// Example: "posx:0.115,posy:-0.516,posz:2.103,velx:18.696,..."
        /// Unknown keys are silently skipped.
        /// </summary>
        public static ThrowParameters Parse(string line)
        {
            var p = new ThrowParameters();
            if (string.IsNullOrWhiteSpace(line)) return p;

            var tokens = line.Split(',');
            foreach (var token in tokens)
            {
                var pair = token.Trim().Split(':');
                if (pair.Length != 2) continue;
                if (!float.TryParse(pair[1],
                    System.Globalization.NumberStyles.Float,
                    System.Globalization.CultureInfo.InvariantCulture,
                    out float v)) continue;

                switch (pair[0].Trim())
                {
                    case "posx":     p.position.x  = v; break;
                    case "posy":     p.position.y  = v; break;
                    case "posz":     p.position.z  = v; break;
                    case "velx":     p.velocity.x  = v; break;
                    case "vely":     p.velocity.y  = v; break;
                    case "velz":     p.velocity.z  = v; break;
                    case "hyzer":    p.hyzer        = v; break;
                    case "pitch":    p.pitch        = v; break;
                    case "spin_d":   p.spinRate      = v; break;
                    case "wobble":   p.wobble        = v; break;
                    case "discmold": p.discIndex     = (DiscLayoutIndex)(int)v; break;
                }
            }
            return p;
        }

        public override string ToString() =>
            $"pos({position.x:F3},{position.y:F3},{position.z:F3}) " +
            $"vel({velocity.x:F3},{velocity.y:F3},{velocity.z:F3}) " +
            $"hyzer:{Mathf.Rad2Deg * hyzer:F1}° pitch:{Mathf.Rad2Deg * pitch:F1}° " +
            $"spin:{spinRate:F1}rad/s wobble:{wobble:F2} disc:{discIndex}";
    }
}
