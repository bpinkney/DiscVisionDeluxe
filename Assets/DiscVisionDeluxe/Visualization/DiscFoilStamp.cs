using System.Collections.Generic;
using UnityEngine;
using TMPro;
using DfisX;

namespace DiscVisionDeluxe.Visualization
{
    /// <summary>
    /// Renders the disc's mold name as a stylized foil stamp on the dome face,
    /// plus procedural decorative elements (arcs, starbursts, rings, diamonds).
    ///
    /// Hierarchy (all created at runtime):
    ///   DiscGO
    ///     └── StampRoot  ← position=domePeak, rotation=Euler(90,angle,0), scale=stampScale
    ///           ├── StampText  (TextMeshPro)
    ///           └── Deco_*    (LineRenderers — one per decoration element)
    ///
    /// StampRoot owns the scale so TMP and all LineRenderers share the same
    /// local coordinate space.  One local unit = 1/stampScale metres in world.
    ///
    /// Line widths are specified in WORLD space (metres) — LineRenderer ignores
    /// parent scale for width even when useWorldSpace=false for positions.
    /// </summary>
    public class DiscFoilStamp : MonoBehaviour
    {
        // Local-space Z for all points — sits just above disc surface.
        // ZTest=Disabled means depth doesn't matter, but a small positive Z
        // (= tiny world-Y lift) prevents edge cases with shadow casters.
        const float kZ = 0.05f;

        GameObject  _stampRoot;
        TextMeshPro _tmp;
        readonly List<GameObject> _decoGOs = new List<GameObject>();

        Color _bright, _mid, _dark;

        // ---------------------------------------------------------------
        // Font recipes — each bundles a typeface name with the styles and
        // casing that suit that font's character.
        // Fonts are loaded from Windows system fonts on first use and cached.
        // If a font is not installed, the recipe falls back to the TMP default.
        // ---------------------------------------------------------------
        struct FontRecipe
        {
            public string   fontName;
            public string   display;    // "upper", "title", "lower"
            public FontStyles style;
            public float    csMin, csMax;  // character spacing range
            public float    outlineWidth;

            public FontRecipe(string fn, string disp, FontStyles fs,
                              float csMin, float csMax, float ow)
            { fontName=fn; display=disp; style=fs;
              this.csMin=csMin; this.csMax=csMax; outlineWidth=ow; }
        }

        static readonly FontRecipe[] kRecipes = new FontRecipe[]
        {
            // Gabriola — Renaissance swirl-fest, looks like a wizard wrote it drunk
            new FontRecipe("Gabriola",               "title", FontStyles.Normal,            0f, 12f, 0.22f),
            // Gabriola italic — even more unhinged loops
            new FontRecipe("Gabriola",               "title", FontStyles.Italic,            0f, 10f, 0.24f),
            // Gabriola bold italic at extreme tracking — barely readable
            new FontRecipe("Gabriola",               "upper", FontStyles.Bold | FontStyles.Italic, 30f, 70f, 0.20f),
            // MV Boli — Maldivian script, extremely alien-looking for Latin text
            new FontRecipe("MV Boli",                "title", FontStyles.Normal,            0f,  8f, 0.20f),
            // MV Boli with extreme tracking — letters float apart like hieroglyphs
            new FontRecipe("MV Boli",                "upper", FontStyles.Normal,           40f, 90f, 0.18f),
            // Ink Free — wild loose brushwork, like a toddler with a marker
            new FontRecipe("Ink Free",               "title", FontStyles.Normal,            0f, 10f, 0.18f),
            // Ink Free italic — unstable, falling-over energy
            new FontRecipe("Ink Free",               "title", FontStyles.Italic,            0f,  8f, 0.20f),
            // Segoe Script Bold italic — cursive gone fully sideways
            new FontRecipe("Segoe Script Bold",      "title", FontStyles.Bold | FontStyles.Italic, 0f, 8f, 0.24f),
            // Segoe Script extreme tracking — individual letters drift apart
            new FontRecipe("Segoe Script",           "title", FontStyles.Italic,           20f, 60f, 0.22f),
            // Segoe Print Bold italic — crayon-on-poster-board energy
            new FontRecipe("Segoe Print Bold",       "upper", FontStyles.Bold | FontStyles.Italic, 8f, 24f, 0.20f),
            // Comic Sans Bold italic with extreme tracking — weaponized Comic Sans
            new FontRecipe("Comic Sans MS Bold",     "upper", FontStyles.Bold | FontStyles.Italic, 20f, 60f, 0.22f),
            // Impact italic at extreme tracking — somehow both compressed AND spread
            new FontRecipe("Impact",                 "upper", FontStyles.Italic,           40f, 90f, 0.10f),
            // Lucida Console bold italic — typewriter from another dimension
            new FontRecipe("Lucida Console",         "upper", FontStyles.Bold | FontStyles.Italic, 20f, 55f, 0.14f),
            // Sylfaen bold italic — Georgian-origin characters look alien in caps
            new FontRecipe("Sylfaen",                "upper", FontStyles.Bold | FontStyles.Italic, 10f, 35f, 0.22f),
            // Franklin Gothic Medium italic extreme — feels like it's screaming
            new FontRecipe("Franklin Gothic Medium", "upper", FontStyles.Bold | FontStyles.Italic, 25f, 65f, 0.16f),
        };

        // Session cache: fontName → TMP asset (null entry = confirmed not installed)
        static readonly Dictionary<string, TMP_FontAsset> s_fontCache
            = new Dictionary<string, TMP_FontAsset>();

        // Populated once from Font.GetOSInstalledFontNames() on first call.
        static HashSet<string> s_installedFonts;

        static TMP_FontAsset GetTMPFont(string fontName)
        {
            if (s_fontCache.TryGetValue(fontName, out var cached)) return cached;

            // Build the installed-font set once per session.
            if (s_installedFonts == null)
            {
                s_installedFonts = new HashSet<string>(
                    Font.GetOSInstalledFontNames(),
                    System.StringComparer.OrdinalIgnoreCase);
            }

            if (!s_installedFonts.Contains(fontName))
            {
                s_fontCache[fontName] = null;
                return null;
            }

            // Use the family+style overload — TMP looks up the font file internally
            // via the OS font registry, same as Font.GetPathsToOSFonts() does internally.
            // Split "Franklin Gothic Medium Italic" → family="Franklin Gothic Medium", style="Italic"
            // For names with no embedded style suffix, style="Regular".
            SplitFamilyStyle(fontName, out string family, out string style);
            var asset = TMP_FontAsset.CreateFontAsset(family, style);

            s_fontCache[fontName] = asset;   // null if creation failed — won't retry
            return asset;
        }

        // ---------------------------------------------------------------
        // Public API
        // ---------------------------------------------------------------
        public void SetDiscModel(DiscModel model)
        {
            if (model == null) { Hide(); return; }
            EnsureHierarchy();
            ApplyStyle(model);
        }

        void Hide() { if (_stampRoot != null) _stampRoot.SetActive(false); }

        // Splits "Georgia Bold Italic" → ("Georgia", "Bold Italic")
        // TMP's CreateFontAsset(familyName, styleName) needs them separate.
        static readonly string[] kStyleSuffixes =
            { "Bold Italic", "Bold", "Italic", "Regular" };

        static void SplitFamilyStyle(string fontName, out string family, out string style)
        {
            foreach (var suffix in kStyleSuffixes)
            {
                if (fontName.EndsWith(" " + suffix, System.StringComparison.OrdinalIgnoreCase))
                {
                    family = fontName.Substring(0, fontName.Length - suffix.Length - 1);
                    style  = suffix;
                    return;
                }
            }
            family = fontName;
            style  = "Regular";
        }

        // ---------------------------------------------------------------
        // Hierarchy bootstrap (called once)
        // ---------------------------------------------------------------
        void EnsureHierarchy()
        {
            if (_stampRoot != null) return;

            _stampRoot = new GameObject("StampRoot");
            _stampRoot.transform.SetParent(transform, false);

            var textGO = new GameObject("StampText");
            textGO.transform.SetParent(_stampRoot.transform, false);

            _tmp = textGO.AddComponent<TextMeshPro>();
            _tmp.fontSize           = 36f;
            _tmp.alignment          = TextAlignmentOptions.Center;
            _tmp.textWrappingMode   = TextWrappingModes.NoWrap;
            _tmp.overflowMode       = TextOverflowModes.Overflow;

            // ZTest=Disabled so text always composites onto the disc face
            // regardless of viewing angle or dome curvature.
            var mat = _tmp.fontMaterial;
            mat.SetFloat("_ZTest", 0f);
            mat.renderQueue = 3000;
        }

        void ClearDecos()
        {
            foreach (var go in _decoGOs) if (go != null) Destroy(go);
            _decoGOs.Clear();
        }

        // ---------------------------------------------------------------
        // Style — text + decorations
        // ---------------------------------------------------------------
        void ApplyStyle(DiscModel model)
        {
            ClearDecos();

            // Deterministic seed — same hash as DiscColorPalette, offset +7
            string key = model.manufacturer + "::" + model.moldName;
            int seed = 0;
            unchecked { foreach (char c in key) seed = seed * 31 + c; }
            var rng = new System.Random(seed + 7);

            // ---- Foil colours (complementary hue to disc body) ----
            Color body = DiscColorPalette.ForModel(model);
            Color.RGBToHSV(body, out float h, out float s, out float v);
            float fh = (h + 0.5f) % 1f;
            float fs = Mathf.Min(1f, s + 0.25f);
            _bright = Color.HSVToRGB(fh, fs,        1.00f);
            _mid    = Color.HSVToRGB(fh, fs,        0.78f);
            _dark   = Color.HSVToRGB(fh, fs * 0.7f, 0.55f);

            // ---- Disc dimensions ----
            float R      = model.radius    > 0f ? model.radius    : 0.106f;
            float rW     = Mathf.Clamp(model.rimWidth, 0.005f, R * 0.40f);
            float innerR = R - rW;                     // flight plate radius (metres)
            float thick  = model.thickness > 0f ? model.thickness : 0.018f;

            // ---- Font recipe — pick deterministically, try to load typeface ----
            int recipeStart = rng.Next(kRecipes.Length);
            FontRecipe recipe = kRecipes[recipeStart];
            TMP_FontAsset chosenFont = null;

            for (int attempt = 0; attempt < kRecipes.Length; attempt++)
            {
                var r = kRecipes[(recipeStart + attempt) % kRecipes.Length];
                var fa = GetTMPFont(r.fontName);
                if (fa != null) { recipe = r; chosenFont = fa; break; }
            }

            if (chosenFont == null)
                Debug.LogWarning($"[DiscFoilStamp] No recipe font found for '{model.moldName}' — using TMP default.");

            // Apply typeface (falls back to default SDF font if chosenFont is null)
            if (chosenFont != null)
                _tmp.font = chosenFont;

            // Casing
            string text = recipe.display == "upper" ? model.moldName.ToUpper()
                        : recipe.display == "lower"  ? model.moldName.ToLower()
                        : model.moldName;   // title

            float cs  = recipe.csMin + (float)rng.NextDouble() * (recipe.csMax - recipe.csMin);
            float ow  = recipe.outlineWidth;

            _tmp.text             = text;
            _tmp.fontStyle        = recipe.style;
            _tmp.characterSpacing = cs;

            // Foil gradient — bright top-left → dark bottom-right
            _tmp.enableVertexGradient = true;
            _tmp.colorGradient = new VertexGradient(_bright, _mid, _mid, _dark);

            // Embossed outline — foil-stamp look
            var fmat = _tmp.fontMaterial;
            fmat.EnableKeyword("OUTLINE_ON");
            fmat.SetFloat("_OutlineWidth", ow);
            fmat.SetColor("_OutlineColor", _dark);

            // ---- Fit text to 50-70% of flight plate diameter ----
            // Measure at scale=1, derive target scale from measured width.
            _stampRoot.transform.localScale = Vector3.one;
            _tmp.ForceMeshUpdate();
            float tw    = Mathf.Max(0.001f, _tmp.bounds.size.x);
            float th    = Mathf.Max(0.001f, _tmp.bounds.size.y);
            float fill  = 0.50f + (float)rng.NextDouble() * 0.20f;
            float tgtW  = innerR * 2f * fill;
            float scl   = Mathf.Clamp(tgtW / tw, 0.001f, (innerR * 1.85f) / tw);
            _stampRoot.transform.localScale = Vector3.one * scl;

            // ---- Position and rotation ----
            float angle = (float)rng.NextDouble() * 360f;
            _stampRoot.transform.localPosition = new Vector3(0f, thick, 0f);
            _stampRoot.transform.localRotation = Quaternion.Euler(90f, angle, 0f);

            // ---- Local-space geometry for decorations ----
            // All coordinates below are in StampRoot's pre-scale local space.
            // 1 local unit = 1/scl metres in world.
            float maxR  = innerR / scl;     // disc inner edge in local units
            float bound = maxR * 0.92f;     // decoration boundary (leave small margin)
            float hw    = tw * 0.5f;        // text half-width (local units)
            float hh    = th * 0.5f;        // text half-height (local units)

            // World-space line widths (LineRenderer ignores parent scale for width)
            float wW = innerR * 0.032f;     // main wide line
            float tW = innerR * 0.016f;     // thin accent line
            float dS = innerR * 0.040f;     // diamond/dot size

            // ---- Build decorations ----
            int dp = rng.Next(5);
            switch (dp)
            {
                case 0: DecoClassicBrackets(rng, hw, hh, bound, wW, tW, dS); break;
                case 1: DecoStarburst      (rng, hw, hh, bound, tW);          break;
                case 2: DecoWingLines      (rng, hw, hh, bound, wW, tW, dS); break;
                case 3: DecoOrbitRing      (rng, hw, hh, bound, tW, dS);     break;
                case 4: DecoBoldDoubleArc  (rng, hw, hh, bound, wW, tW, dS); break;
            }

            _stampRoot.SetActive(true);
        }

        // ================================================================
        // Decoration presets
        // ================================================================

        // Preset 0 — Classic disc stamp: arcs above + below with dot end-caps
        void DecoClassicBrackets(System.Random rng, float hw, float hh,
            float bound, float wW, float tW, float dS)
        {
            float gap   = hh * 0.35f;
            float arcY  = hh + gap;
            float bulge = bound * 0.20f;

            Arc(-bound, arcY, bound, arcY, bulge,  _mid, wW);    // upper
            Arc(-bound,-arcY, bound,-arcY,-bulge,  _mid, wW);    // lower

            // Thinner secondary arcs further out
            Arc(-bound * 0.80f, arcY  + bound * 0.09f,
                 bound * 0.80f, arcY  + bound * 0.09f,
                 bound * 0.10f, _dark, tW);
            Arc(-bound * 0.80f,-(arcY + bound * 0.09f),
                 bound * 0.80f,-(arcY + bound * 0.09f),
                -bound * 0.10f, _dark, tW);

            // Dot end-caps
            Dot(-bound,  arcY, dS, _bright);
            Dot( bound,  arcY, dS, _bright);
            Dot(-bound, -arcY, dS, _bright);
            Dot( bound, -arcY, dS, _bright);
        }

        // Preset 1 — Starburst: rays radiating from a ring around the text
        void DecoStarburst(System.Random rng, float hw, float hh,
            float bound, float tW)
        {
            int rays    = 10 + rng.Next(4) * 2;          // 10,12,14,16
            float inner = Mathf.Sqrt(hw * hw + hh * hh) * 1.15f;

            for (int i = 0; i < rays; i++)
            {
                float a   = (float)i / rays * Mathf.PI * 2f;
                float cos = Mathf.Cos(a), sin = Mathf.Sin(a);
                bool  alt = i % 2 == 0;
                Line(cos * inner, sin * inner,
                     cos * bound, sin * bound,
                     alt ? _mid : _dark,
                     alt ? tW * 1.6f : tW * 0.9f);
            }
            // Inner ring to cap the rays cleanly
            Circle(0f, 0f, inner, _bright, tW, 36);
            // Outer ring at boundary
            Circle(0f, 0f, bound, _dark, tW * 0.6f, 48);
        }

        // Preset 2 — Wing lines: flowing horizontal curves above + below + diamonds
        void DecoWingLines(System.Random rng, float hw, float hh,
            float bound, float wW, float tW, float dS)
        {
            float gap = hh * 0.30f;
            float y1  = hh + gap;
            float y2  = y1 + bound * 0.10f;

            Arc(-bound, y1, bound, y1, bound * 0.14f, _bright, wW);
            Arc(-bound * 0.78f, y2, bound * 0.78f, y2, bound * 0.07f, _dark, tW);
            Arc(-bound,-y1, bound,-y1,-bound * 0.14f, _bright, wW);
            Arc(-bound * 0.78f,-y2, bound * 0.78f,-y2,-bound * 0.07f, _dark, tW);

            // Diamond end-caps at wing tips
            Diamond(-bound,  y1, dS, _bright);
            Diamond( bound,  y1, dS, _bright);
            Diamond(-bound, -y1, dS, _bright);
            Diamond( bound, -y1, dS, _bright);

            // Small centre diamonds top/bottom on the outer arcs
            Diamond(0f,  y1 + bound * 0.14f, dS * 0.65f, _mid);
            Diamond(0f, -y1 - bound * 0.14f, dS * 0.65f, _mid);
        }

        // Preset 3 — Orbit ring: ellipse with tick marks and cardinal dots
        void DecoOrbitRing(System.Random rng, float hw, float hh,
            float bound, float tW, float dS)
        {
            float rx = bound * 0.90f;
            float ry = Mathf.Clamp(hh * 2.6f, bound * 0.22f, bound * 0.58f);

            Ellipse(rx, ry, _mid, tW * 1.5f, 48);

            // Tick marks at equal intervals around the ellipse
            int ticks  = 12;
            float tickL = bound * 0.07f;
            for (int i = 0; i < ticks; i++)
            {
                float a  = (float)i / ticks * Mathf.PI * 2f;
                float ex = Mathf.Cos(a) * rx;
                float ey = Mathf.Sin(a) * ry;
                // Outward normal on ellipse
                float nx = Mathf.Cos(a), ny = Mathf.Sin(a) * (rx / Mathf.Max(ry, 0.001f));
                float m  = Mathf.Sqrt(nx * nx + ny * ny);
                nx /= m; ny /= m;
                float w = (i % 3 == 0) ? tW * 1.4f : tW * 0.8f;
                Line(ex, ey, ex + nx * tickL, ey + ny * tickL, i % 3 == 0 ? _bright : _dark, w);
            }

            // Cardinal dots
            Dot( rx,  0f, dS, _bright);
            Dot(-rx,  0f, dS, _bright);
            Dot( 0f,  ry, dS, _bright);
            Dot( 0f, -ry, dS, _bright);
        }

        // Preset 4 — Bold double arc: thick sweeping arcs + corner diamonds
        void DecoBoldDoubleArc(System.Random rng, float hw, float hh,
            float bound, float wW, float tW, float dS)
        {
            float gap  = hh * 0.25f;
            float arcY = hh + gap;
            float b1   = bound * 0.28f;   // large bulge
            float b2   = bound * 0.10f;   // secondary bulge

            // Main bold arcs
            Arc(-bound, arcY, bound, arcY, b1, _bright, wW * 1.4f);
            Arc(-bound,-arcY, bound,-arcY,-b1, _bright, wW * 1.4f);

            // Thinner outer secondary arcs
            float r2 = bound * 0.82f;
            float y2 = arcY + bound * 0.13f;
            Arc(-r2, y2, r2, y2, b2, _mid, tW);
            Arc(-r2,-y2, r2,-y2,-b2, _mid, tW);

            // Diamond at each main arc endpoint
            Diamond(-bound,  arcY, dS, _bright);
            Diamond( bound,  arcY, dS, _bright);
            Diamond(-bound, -arcY, dS, _bright);
            Diamond( bound, -arcY, dS, _bright);

            // Flourish diamonds above/below the arc peaks
            float peakY = arcY + b1;
            Diamond(0f,  peakY, dS * 0.75f, _mid);
            Diamond(0f, -peakY, dS * 0.75f, _mid);
        }

        // ================================================================
        // Drawing primitives — all create LineRenderer children of StampRoot
        // ================================================================

        void Arc(float x0, float y0, float x1, float y1,
                 float bulge, Color col, float worldW)
        {
            const int segs = 24;
            var pts = new Vector3[segs + 1];
            for (int i = 0; i <= segs; i++)
            {
                float t = (float)i / segs;
                pts[i] = new Vector3(
                    Mathf.Lerp(x0, x1, t),
                    Mathf.Lerp(y0, y1, t) + bulge * Mathf.Sin(t * Mathf.PI),
                    kZ);
            }
            MakeLine(pts, col, worldW);
        }

        void Line(float x0, float y0, float x1, float y1, Color col, float worldW)
        {
            MakeLine(new[]
            {
                new Vector3(x0, y0, kZ),
                new Vector3(x1, y1, kZ)
            }, col, worldW);
        }

        void Circle(float cx, float cy, float r, Color col, float worldW, int segs)
        {
            var pts = new Vector3[segs + 1];
            for (int i = 0; i <= segs; i++)
            {
                float a = (float)i / segs * Mathf.PI * 2f;
                pts[i] = new Vector3(cx + Mathf.Cos(a) * r, cy + Mathf.Sin(a) * r, kZ);
            }
            MakeLine(pts, col, worldW);
        }

        void Ellipse(float rx, float ry, Color col, float worldW, int segs)
        {
            var pts = new Vector3[segs + 1];
            for (int i = 0; i <= segs; i++)
            {
                float a = (float)i / segs * Mathf.PI * 2f;
                pts[i] = new Vector3(Mathf.Cos(a) * rx, Mathf.Sin(a) * ry, kZ);
            }
            MakeLine(pts, col, worldW);
        }

        void Dot(float cx, float cy, float size, Color col)
        {
            Circle(cx, cy, size, col, size * 0.7f, 12);
        }

        void Diamond(float cx, float cy, float size, Color col)
        {
            MakeLine(new[]
            {
                new Vector3(cx,        cy + size, kZ),
                new Vector3(cx + size, cy,        kZ),
                new Vector3(cx,        cy - size, kZ),
                new Vector3(cx - size, cy,        kZ),
                new Vector3(cx,        cy + size, kZ),
            }, col, size * 0.32f);
        }

        void MakeLine(Vector3[] pts, Color col, float worldW)
        {
            var go = new GameObject("Deco");
            go.transform.SetParent(_stampRoot.transform, false);

            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace     = false;
            lr.positionCount     = pts.Length;
            lr.SetPositions(pts);
            lr.startWidth        = worldW;
            lr.endWidth          = worldW;
            lr.startColor        = col;
            lr.endColor          = col;
            lr.numCapVertices    = 4;
            lr.numCornerVertices = 4;

            Shader sh = Shader.Find("Universal Render Pipeline/Particles/Unlit")
                     ?? Shader.Find("Sprites/Default");
            if (sh != null)
            {
                var mat = new Material(sh) { color = col };
                mat.SetFloat("_ZTest", 0f);
                mat.renderQueue = 3001;
                lr.sharedMaterial = mat;
            }

            _decoGOs.Add(go);
        }
    }
}
