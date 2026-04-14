using UnityEngine;
using UnityEngine.UI;
using TMPro;

namespace DiscVisionDeluxe.UI
{
    /// <summary>
    /// POL-3: 2D software-drawn mini-map.
    /// Tee at top, flight direction goes downward. World Z lateral → horizontal.
    /// </summary>
    public class MiniMap : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("References")]
        public Transform      discTransform;
        public DiscVisualizer discVisualizer;

        [Header("World bounds")]
        public float worldXMin  = 0f;
        public float worldXMax  = 160f;
        public float worldZHalf = 55f;

        [Header("Distance grid (feet)")]
        public float gridIntervalFt   = 50f;
        public Color gridColor        = new Color(1f, 1f, 1f, 0.20f);
        [Range(1, 4)]
        public int   gridThicknessPx  = 1;

        [Header("Distance labels (every N feet)")]
        public float labelIntervalFt  = 50f;
        public float labelFontSize    = 11f;
        public Color labelColor       = new Color(1f, 1f, 1f, 0.75f);

        [Header("Trail")]
        public Color trailColor       = new Color(1f, 0.85f, 0.1f, 1f);
        [Range(1, 8)]
        public int   trailThicknessPx = 3;

        [Header("Disc dot")]
        public Color discColor        = new Color(1f, 0.35f, 0.1f, 1f);
        [Range(2, 10)]
        public int   discRadiusPx     = 4;

        [Header("Panel")]
        public Vector2Int texResolution = new Vector2Int(160, 280);
        public Vector2    panelSizePx   = new Vector2(140f, 245f);
        public Vector2    marginPx      = new Vector2(14f, 14f);
        public Color      bgColor       = new Color(0.04f, 0.09f, 0.04f, 0.45f);
        public Color      borderColor   = new Color(1f, 1f, 1f, 0.25f);
        public float      borderWidthPx = 1.5f;

        // ── Private ──────────────────────────────────────────────────────────

        Texture2D _tex;
        RawImage  _rawImage;
        Color32[] _clearBuffer;

        // ── Lifecycle ─────────────────────────────────────────────────────────

        void Start()
        {
            if (discVisualizer == null)
                discVisualizer = FindAnyObjectByType<DiscVisualizer>();
            if (discTransform == null && discVisualizer != null)
                discTransform = discVisualizer.discTransform;

            BuildTexture();
            BuildUI();
        }

        void Update() => Redraw();

        // ── Build ────────────────────────────────────────────────────────────

        void BuildTexture()
        {
            _tex             = new Texture2D(texResolution.x, texResolution.y, TextureFormat.RGBA32, false);
            _tex.filterMode  = FilterMode.Bilinear;
            _clearBuffer     = new Color32[texResolution.x * texResolution.y];
            Color32 bg32     = bgColor;
            for (int i = 0; i < _clearBuffer.Length; i++)
                _clearBuffer[i] = bg32;
        }

        void BuildUI()
        {
            // ── Canvas ───────────────────────────────────────────────────────
            var canvasGo            = new GameObject("MiniMap_Canvas");
            canvasGo.transform.SetParent(transform, false);
            var canvas              = canvasGo.AddComponent<Canvas>();
            canvas.renderMode       = RenderMode.ScreenSpaceOverlay;
            canvas.sortingOrder     = 10;
            var scaler              = canvasGo.AddComponent<CanvasScaler>();
            scaler.uiScaleMode      = CanvasScaler.ScaleMode.ConstantPixelSize;
            canvasGo.AddComponent<GraphicRaycaster>();

            // ── Border ───────────────────────────────────────────────────────
            var borderGo            = new GameObject("MiniMap_Border");
            borderGo.transform.SetParent(canvasGo.transform, false);
            borderGo.AddComponent<Image>().color = borderColor;
            var borderRt            = borderGo.GetComponent<RectTransform>();
            borderRt.anchorMin      = new Vector2(0f, 1f);
            borderRt.anchorMax      = new Vector2(0f, 1f);
            borderRt.pivot          = new Vector2(0f, 1f);
            borderRt.sizeDelta      = panelSizePx + Vector2.one * borderWidthPx * 2f;
            borderRt.anchoredPosition = new Vector2(marginPx.x, -marginPx.y);

            // ── Background fill ───────────────────────────────────────────────
            var fillGo              = new GameObject("MiniMap_Fill");
            fillGo.transform.SetParent(borderGo.transform, false);
            fillGo.AddComponent<Image>().color = bgColor;
            var fillRt              = fillGo.GetComponent<RectTransform>();
            fillRt.anchorMin        = Vector2.zero;
            fillRt.anchorMax        = Vector2.one;
            fillRt.offsetMin        = Vector2.one * borderWidthPx;
            fillRt.offsetMax        = -Vector2.one * borderWidthPx;

            // ── Map image ─────────────────────────────────────────────────────
            var imgGo               = new GameObject("MiniMap_Image");
            imgGo.transform.SetParent(fillGo.transform, false);
            _rawImage               = imgGo.AddComponent<RawImage>();
            _rawImage.texture       = _tex;
            _rawImage.color         = new Color(1f, 1f, 1f, 1f);
            var imgRt               = imgGo.GetComponent<RectTransform>();
            imgRt.anchorMin         = Vector2.zero;
            imgRt.anchorMax         = Vector2.one;
            imgRt.offsetMin         = Vector2.zero;
            imgRt.offsetMax         = Vector2.zero;

            // ── Distance labels overlay ───────────────────────────────────────
            // Labels are children of the image rect so they scale with it.
            float ftToM2   = 1f / 3.28084f;
            float distFt   = labelIntervalFt;
            while (distFt * ftToM2 < worldXMax - 0.01f)
            {
                float distM = distFt * ftToM2;
                float normY = distM / worldXMax;

                var lblGo  = new GameObject($"Label_{distFt:F0}ft");
                lblGo.transform.SetParent(imgGo.transform, false);

                var tmp        = lblGo.AddComponent<TextMeshProUGUI>();
                tmp.text       = $"{distFt:F0}";
                tmp.fontSize   = labelFontSize;
                tmp.color      = labelColor;
                tmp.alignment  = TextAlignmentOptions.Center;
                tmp.textWrappingMode   = TMPro.TextWrappingModes.NoWrap;
                tmp.fontStyle  = FontStyles.Bold;
                tmp.outlineWidth = 0.2f;
                tmp.outlineColor = new Color32(0, 0, 0, 200);

                var rt          = lblGo.GetComponent<RectTransform>();
                rt.anchorMin    = new Vector2(0f, normY);
                rt.anchorMax    = new Vector2(1f, normY);
                rt.pivot        = new Vector2(0.5f, 0.5f);
                rt.sizeDelta    = new Vector2(0f, labelFontSize * 1.5f);
                rt.anchoredPosition = Vector2.zero;

                distFt += labelIntervalFt;
            }
        }

        // ── Draw ─────────────────────────────────────────────────────────────

        void Redraw()
        {
            if (_tex == null) return;

            _tex.SetPixels32(_clearBuffer);

            // Grid lines (interval in feet, converted to metres for world lookup)
            float ftToM = 1f / 3.28084f;
            float d = gridIntervalFt * ftToM;
            while (d <= worldXMax + 0.01f)
            {
                DrawHLine(WorldToPixelY(d), gridThicknessPx, gridColor);
                d += gridIntervalFt * ftToM;
            }

            // DfisX active trail
            if (discVisualizer != null)
            {
                var lr = discVisualizer.dfisxTrajectoryLine;
                DrawTrail(lr, trailColor);

                foreach (var archived in discVisualizer.GetArchivedTrails())
                    DrawTrail(archived, archived != null ? archived.startColor : trailColor);
            }

            // Disc dot
            if (discTransform != null)
                DrawDisc(WorldToPixelX(discTransform.position.z),
                         WorldToPixelY(discTransform.position.x),
                         discRadiusPx, discColor);

            _tex.Apply();
        }

        void DrawTrail(LineRenderer lr, Color c)
        {
            if (lr == null || lr.positionCount < 2) return;
            var pts = new Vector3[lr.positionCount];
            lr.GetPositions(pts);
            for (int i = 1; i < pts.Length; i++)
            {
                DrawLine(WorldToPixelX(pts[i-1].z), WorldToPixelY(pts[i-1].x),
                         WorldToPixelX(pts[i  ].z), WorldToPixelY(pts[i  ].x),
                         trailThicknessPx, c);
            }
        }

        // ── Coordinate mapping ────────────────────────────────────────────────

        // Tee at top → worldXMin maps to top of texture (py = texHeight-1), far end to bottom.
        // Vertical flip from previous: was (1-t), now t (tee at bottom was wrong).
        int WorldToPixelY(float worldX)
        {
            float t = Mathf.InverseLerp(worldXMin, worldXMax, worldX);
            return Mathf.RoundToInt(t * (texResolution.y - 1));
        }

        // Horizontal flip: swap -worldZHalf/worldZHalf order relative to previous version.
        int WorldToPixelX(float worldZ)
        {
            float t = Mathf.InverseLerp(worldZHalf, -worldZHalf, worldZ);
            return Mathf.RoundToInt(t * (texResolution.x - 1));
        }

        // ── Drawing primitives ────────────────────────────────────────────────

        void SetPixel(int x, int y, Color32 c)
        {
            if (x < 0 || x >= texResolution.x || y < 0 || y >= texResolution.y) return;
            _tex.SetPixel(x, y, c);
        }

        void DrawHLine(int py, int thickness, Color c)
        {
            Color32 c32 = c;
            int half = thickness / 2;
            for (int t = -half; t <= half; t++)
                for (int x = 0; x < texResolution.x; x++)
                    SetPixel(x, py + t, c32);
        }

        void DrawDisc(int cx, int cy, int r, Color c)
        {
            Color32 c32 = c;
            for (int dy = -r; dy <= r; dy++)
                for (int dx = -r; dx <= r; dx++)
                    if (dx * dx + dy * dy <= r * r)
                        SetPixel(cx + dx, cy + dy, c32);
        }

        void DrawLine(int x0, int y0, int x1, int y1, int thickness, Color c)
        {
            Color32 c32 = c;
            int dx   =  Mathf.Abs(x1 - x0);
            int dy   = -Mathf.Abs(y1 - y0);
            int sx   = x0 < x1 ? 1 : -1;
            int sy   = y0 < y1 ? 1 : -1;
            int err  = dx + dy;
            int half = thickness / 2;

            while (true)
            {
                for (int ty = -half; ty <= half; ty++)
                    for (int tx = -half; tx <= half; tx++)
                        SetPixel(x0 + tx, y0 + ty, c32);

                if (x0 == x1 && y0 == y1) break;
                int e2 = 2 * err;
                if (e2 >= dy) { err += dy; x0 += sx; }
                if (e2 <= dx) { err += dx; y0 += sy; }
            }
        }
    }
}
