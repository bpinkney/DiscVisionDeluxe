using UnityEngine;
using TMPro;

namespace DiscVisionDeluxe.Environment
{
    /// <summary>
    /// Spawns distance marker posts along the throw axis (Unity +X, which is DfisX forward).
    /// Each post has a world-space TMP label (with outline) and an optional ground stripe.
    ///
    /// Scene setup:
    ///   - Place on any GameObject at or near the tee position.
    ///   - Posts spawn along +X from the tee at markerIntervalM intervals.
    ///   - Call Rebuild() at runtime if any Inspector values change.
    /// </summary>
    public class DistanceMarkerSpawner : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("Spacing")]
        [Tooltip("Distance between markers (metres).")]
        public float markerIntervalM = 10f;

        [Tooltip("Maximum distance to spawn markers (metres).")]
        public float maxDistanceM = 150f;

        [Tooltip("How far to the side (+Z) of the flight path the posts sit.")]
        public float sideOffsetM = 3f;

        [Tooltip("Height above the ground plane. Raise if posts sink into terrain.")]
        public float groundOffsetY = 0f;

        [Header("Display")]
        public bool showMetres = true;
        public bool showFeet   = true;

        [Header("Post Appearance")]
        [Tooltip("Height of the marker post (metres).")]
        public float postHeight = 1.2f;

        [Tooltip("Radius of the marker post cylinder.")]
        public float postRadius = 0.03f;

        public Color postColor = new Color(0.85f, 0.85f, 0.85f, 1f);

        [Header("Label")]
        [Tooltip("Font size (world-space metres). Smaller = more compact markers.")]
        public float labelFontSize = 1.0f;

        public Color labelColor = Color.white;

        [Tooltip("Outline width applied to the TMP label (0 = none, 0.25 = visible stroke).")]
        [Range(0f, 0.5f)]
        public float labelOutlineWidth = 0.25f;

        public Color labelOutlineColor = new Color(0f, 0f, 0f, 1f);

        [Tooltip("Height offset of the label above the top of the post.")]
        public float labelHeightOffset = 0.1f;

        [Header("Ground Lines")]
        [Tooltip("Draw a thin line on the ground at each distance marker.")]
        public bool showGroundLines = true;

        [Tooltip("Total width of the ground line (metres), centred on the flight path.")]
        public float groundLineHalfWidth = 10f;

        [Tooltip("Line renderer width (metres).")]
        public float groundLineWidth = 0.04f;

        public Color groundLineColor = new Color(1f, 1f, 1f, 0.25f);

        // ── Private ──────────────────────────────────────────────────────────

        private GameObject _markerRoot;

        // ── Unity lifecycle ──────────────────────────────────────────────────

        void Start()
        {
            Rebuild();
        }

        // ── Public API ───────────────────────────────────────────────────────

        /// <summary>Destroy and respawn all markers. Call after changing Inspector values at runtime.</summary>
        public void Rebuild()
        {
            if (_markerRoot != null)
                Destroy(_markerRoot);

            _markerRoot = new GameObject("DistanceMarkers");
            _markerRoot.transform.SetParent(transform.parent, false);

            float dist = markerIntervalM;
            while (dist <= maxDistanceM + 0.01f)
            {
                SpawnMarker(dist);
                dist += markerIntervalM;
            }
        }

        // ── Private ──────────────────────────────────────────────────────────

        void SpawnMarker(float distM)
        {
            var marker = new GameObject($"Marker_{distM:F0}m");
            marker.transform.SetParent(_markerRoot.transform, false);
            marker.transform.localPosition = new Vector3(distM, groundOffsetY, sideOffsetM);

            // ── Post (cylinder) ──────────────────────────────────────────────
            var post = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            post.name = "Post";
            post.transform.SetParent(marker.transform, false);
            post.transform.localPosition = new Vector3(0f, postHeight * 0.5f, 0f);
            post.transform.localScale    = new Vector3(postRadius * 2f, postHeight * 0.5f, postRadius * 2f);
            Destroy(post.GetComponent<Collider>());

            var postRenderer = post.GetComponent<Renderer>();
            if (postRenderer != null)
            {
                var mat = new Material(Shader.Find("Universal Render Pipeline/Lit")
                       ?? Shader.Find("Standard"));
                mat.color = postColor;
                postRenderer.sharedMaterial = mat;
            }

            // ── Label (world-space Canvas + TMP) ────────────────────────────
            var labelGo = new GameObject("Label");
            labelGo.transform.SetParent(marker.transform, false);
            labelGo.transform.localPosition = new Vector3(0f, postHeight + labelHeightOffset, 0f);

            var canvas = labelGo.AddComponent<Canvas>();
            canvas.renderMode = RenderMode.WorldSpace;

            var rt = labelGo.GetComponent<RectTransform>();
            rt.sizeDelta     = new Vector2(2f, 1f);
            rt.localScale    = Vector3.one * 0.008f;
            rt.localRotation = Quaternion.Euler(0f, 90f, 0f);

            var tmp = labelGo.AddComponent<TextMeshProUGUI>();
            tmp.text                = BuildLabelText(distM);
            tmp.fontSize            = labelFontSize * 100f;
            tmp.color               = labelColor;
            tmp.alignment           = TextAlignmentOptions.Center;
            tmp.textWrappingMode    = TMPro.TextWrappingModes.NoWrap;

            // Outline — accessing fontMaterial creates a per-instance copy
            // so this doesn't modify the shared font asset material.
            tmp.fontMaterial.EnableKeyword("OUTLINE_ON");
            tmp.outlineWidth = labelOutlineWidth;
            tmp.outlineColor = labelOutlineColor;

            // ── Ground line ──────────────────────────────────────────────────
            if (showGroundLines)
                SpawnGroundLine(distM);
        }

        void SpawnGroundLine(float distM)
        {
            var lineGo = new GameObject($"GroundLine_{distM:F0}m");
            lineGo.transform.SetParent(_markerRoot.transform, false);

            var lr = lineGo.AddComponent<LineRenderer>();
            lr.useWorldSpace     = false;
            lr.positionCount     = 2;
            lr.startWidth        = groundLineWidth;
            lr.endWidth          = groundLineWidth;
            lr.startColor        = groundLineColor;
            lr.endColor          = groundLineColor;
            lr.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
            lr.receiveShadows    = false;

            // Auto-create a vertex-color-capable material
            Shader shader = Shader.Find("Universal Render Pipeline/Particles/Unlit")
                         ?? Shader.Find("Sprites/Default");
            if (shader != null)
                lr.material = new Material(shader) { color = Color.white };

            // Line runs perpendicular to the throw axis (along Z) at ground level
            lr.SetPosition(0, new Vector3(distM, groundOffsetY + 0.01f, -groundLineHalfWidth));
            lr.SetPosition(1, new Vector3(distM, groundOffsetY + 0.01f,  groundLineHalfWidth));
        }

        string BuildLabelText(float distM)
        {
            if (showMetres && showFeet)
                return $"{distM:F0}m\n{distM * 3.281f:F0}ft";
            if (showMetres)
                return $"{distM:F0}m";
            if (showFeet)
                return $"{distM * 3.281f:F0}ft";
            return $"{distM:F0}";
        }
    }
}
