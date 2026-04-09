using System.Collections.Generic;
using UnityEngine;
using TMPro;

namespace DiscVisionDeluxe.Visualization
{
    /// <summary>
    /// POL-2: Floating distance readout that appears at the disc landing position.
    ///
    /// Self-contained pooled component — no prefab required.
    /// Call LandingMarker.Spawn() from DiscVisualizer.HandleSimFinished.
    ///
    /// Each marker floats upward and fades out over fadeDurationS, then returns to pool.
    /// </summary>
    public class LandingMarker : MonoBehaviour
    {
        // ---------------------------------------------------------------
        // Configuration (set before Spawn, or edit defaults below)
        // ---------------------------------------------------------------
        public static float FontSize       = 2.2f;
        public static float FloatSpeedMps  = 0.6f;   // metres per second upward
        public static float FadeDurationS  = 5f;
        public static Color LabelColor     = Color.white;

        // ---------------------------------------------------------------
        // Pool
        // ---------------------------------------------------------------
        static readonly List<LandingMarker> _pool = new List<LandingMarker>();

        /// <summary>
        /// Spawn (or recycle) a landing marker at worldPos showing distanceM.
        /// </summary>
        public static LandingMarker Spawn(Vector3 worldPos, float distanceM)
        {
            LandingMarker m = null;

            for (int i = _pool.Count - 1; i >= 0; i--)
            {
                if (_pool[i] != null)
                {
                    m = _pool[i];
                    _pool.RemoveAt(i);
                    break;
                }
            }

            if (m == null)
                m = CreateNew();

            m.transform.position = worldPos;
            m.gameObject.SetActive(true);
            m.Begin(distanceM);
            return m;
        }

        // ---------------------------------------------------------------
        // Private state
        // ---------------------------------------------------------------
        TextMeshProUGUI _tmp;
        float           _elapsed;

        // ---------------------------------------------------------------
        // Factory
        // ---------------------------------------------------------------
        static LandingMarker CreateNew()
        {
            var go = new GameObject("LandingMarker");
            var marker = go.AddComponent<LandingMarker>();

            // World-space canvas (same pattern as DistanceMarkerSpawner)
            var labelGo = new GameObject("Label");
            labelGo.transform.SetParent(go.transform, false);

            var canvas = labelGo.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;

            var rt = labelGo.GetComponent<RectTransform>();
            rt.sizeDelta  = new Vector2(3f, 1.2f);
            rt.localScale = Vector3.one * 0.012f;   // 1 unit ≈ 1.2 cm in world space

            marker._tmp = labelGo.AddComponent<TextMeshProUGUI>();
            marker._tmp.fontSize             = FontSize * 100f;
            marker._tmp.color                = LabelColor;
            marker._tmp.alignment            = TextAlignmentOptions.Center;
            marker._tmp.textWrappingMode     = TMPro.TextWrappingModes.NoWrap;
            marker._tmp.fontStyle            = FontStyles.Bold;

            return marker;
        }

        // ---------------------------------------------------------------
        // Begin animation
        // ---------------------------------------------------------------
        void Begin(float distanceM)
        {
            _elapsed = 0f;

            if (_tmp != null)
            {
                float ft = distanceM * 3.281f;
                _tmp.text  = $"{distanceM:F1} m\n{ft:F0} ft";
                _tmp.color = LabelColor;
            }
        }

        // ---------------------------------------------------------------
        // Animation loop
        // ---------------------------------------------------------------
        void Update()
        {
            if (_tmp == null) return;

            _elapsed += Time.deltaTime;
            float t = Mathf.Clamp01(_elapsed / FadeDurationS);

            // Float upward
            transform.position += Vector3.up * FloatSpeedMps * Time.deltaTime;

            // Fade — keep full opacity for first 20% of the duration, then fade
            float fadeT   = Mathf.Clamp01((t - 0.2f) / 0.8f);
            Color c       = _tmp.color;
            c.a           = 1f - fadeT;
            _tmp.color    = c;

            // Billboard: face the main camera so the label is always readable
            if (UnityEngine.Camera.main != null)
                transform.rotation = UnityEngine.Camera.main.transform.rotation;

            // Return to pool when fully faded
            if (_elapsed >= FadeDurationS)
            {
                gameObject.SetActive(false);
                _pool.Add(this);
            }
        }
    }
}
