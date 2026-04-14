using System.Collections.Generic;
using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe.Visualization
{
    /// <summary>
    /// Deterministic per-mold colour lookup shared by DiscVisualController (disc body)
    /// and DiscVisualizer (trail).  Same mold always returns the same colour within a
    /// session and across sessions, seeded from manufacturer + moldName.
    /// </summary>
    public static class DiscColorPalette
    {
        // HSV ranges — moderate saturation, high value → visible plastic colours.
        const float kSatMin = 0.35f;
        const float kSatMax = 0.75f;
        const float kValMin = 0.75f;
        const float kValMax = 0.95f;

        static readonly Dictionary<string, Color> s_cache = new Dictionary<string, Color>();

        /// <summary>
        /// Returns a deterministic colour for the given disc model.
        /// Returns white if model is null.
        /// </summary>
        public static Color ForModel(DiscModel model)
        {
            if (model == null) return Color.white;

            string key = model.manufacturer + "::" + model.moldName;
            if (s_cache.TryGetValue(key, out Color cached))
                return cached;

            // Polynomial hash of the key string — same algorithm as Java String.hashCode().
            // Using System.Random (not Unity's global RNG) to avoid disturbing sim randomness.
            int seed = 0;
            unchecked { foreach (char c in key) seed = seed * 31 + c; }

            var rng   = new System.Random(seed);
            float hue = (float)rng.NextDouble();
            float sat = kSatMin + (float)rng.NextDouble() * (kSatMax - kSatMin);
            float val = kValMin + (float)rng.NextDouble() * (kValMax - kValMin);

            Color color = Color.HSVToRGB(hue, sat, val);
            s_cache[key] = color;
            return color;
        }

        /// <summary>
        /// Returns a trail colour for a model: same hue as the disc body but slightly
        /// desaturated and brightened so the trail reads clearly without overwhelming
        /// the disc itself.
        /// </summary>
        public static Color TrailForModel(DiscModel model)
        {
            Color body = ForModel(model);
            Color.RGBToHSV(body, out float h, out float s, out float v);
            return Color.HSVToRGB(h, Mathf.Max(0f, s - 0.15f), Mathf.Min(1f, v + 0.05f));
        }
    }
}
