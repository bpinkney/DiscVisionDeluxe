using System.Collections.Generic;
using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe.Visualization
{
    /// <summary>
    /// Generates a procedural disc mesh from a DiscModel's physical dimensions.
    /// Uses a surface-of-revolution (lathe) approach around the Y axis.
    ///
    /// Coordinate conventions (local space):
    ///   The bottom outer rim edge sits at y = rimCamberHeight (NOT zero).
    ///   The dome peak sits at y = DiscModel.thickness.
    ///   The disc lies flat in the XZ plane; Y is the disc spin axis.
    ///
    /// Cross-section profile (traced clockwise in (r, y) from top-center):
    ///   Dome (cosine)  →  flat rim top  →  outer edge (EdgeHeight tall)  →
    ///   lower rim camber (Flat/Concave/Convex)  →  cavity inner wall  →  cavity floor
    /// </summary>
    public static class DiscMeshBuilder
    {
        const float kAxisEps = 1e-5f;

        /// <param name="model">Disc model — dimensions must be in metres.</param>
        /// <param name="segments">Angular subdivisions around Y axis (>=8, default 32).</param>
        /// <param name="domeSegs">Subdivisions along the dome curve (>=4, default 10).</param>
        public static Mesh Build(DiscModel model, int segments = 32, int domeSegs = 10)
        {
            segments = Mathf.Max(8,  segments);
            domeSegs = Mathf.Max(4,  domeSegs);

            float R         = model.radius;
            float rW        = Mathf.Clamp(model.rimWidth,        0.005f, R * 0.40f);
            float H         = model.thickness;
            float dH        = Mathf.Clamp(model.domeHeight,      0f,     H * 0.70f);
            float rD        = Mathf.Clamp(model.rimDepth,        0f,     H - 0.001f);
            float camberH   = Mathf.Clamp(model.rimCamberHeight, 0f,     H * 0.40f);
            float innerR    = Mathf.Max(kAxisEps * 100f, R - rW);
            float rimY      = H - dH;   // y at dome base / inner rim junction

            // ----------------------------------------------------------------
            // Build the 2D cross-section profile as a list of (r, y) points.
            // Traversal is CLOCKWISE in (r, y) space starting from the centre top.
            //
            // Driver rim anatomy:
            //   The rim top is NOT flat — it slopes DOWNWARD from the flight plate
            //   (innerR, rimY) out to the edge (R, edgeTopY).  This is what makes
            //   drivers look thin and sharp rather than having a chunky shelf.
            //   Below the edge, the lower camber sweeps inward to (innerR, 0).
            //   edgeTopY ≈ camberH so the edge is nearly a point (1-2 mm thick).
            // ----------------------------------------------------------------
            const int rimSegs    = 5;   // segments along the sloped rim top
            const int camberSegs = 5;   // segments along the lower camber sweep

            // Very small residual wall at the outermost edge so normals don't collapse.
            float edgeTopY = camberH + Mathf.Max(0.0005f, H * 0.03f);

            bool concave = model.rimCamberShape == "Concave";
            bool convex  = model.rimCamberShape == "Convex";

            var profile = new List<Vector2>(domeSegs + rimSegs + camberSegs + 6);

            // 1. Dome: cosine curve from (0, H) outward to (innerR, rimY)
            for (int i = 0; i <= domeSegs; i++)
            {
                float t = (float)i / domeSegs;
                float r = Mathf.Lerp(0f, innerR, t);
                float y = rimY + (H - rimY) * Mathf.Cos(t * Mathf.PI * 0.5f);
                profile.Add(new Vector2(r, y));
            }
            // Dome ends at (innerR, rimY). Rim overhang starts from i=1.

            // 2. Rim top overhang — slopes DOWN from (innerR, rimY) to (R, edgeTopY).
            //    This replaces the old flat rim + tall outer wall.
            //    Uses a cosine ease-out so the slope starts gentle at the flight plate
            //    junction and steepens toward the edge — matches real disc geometry.
            for (int i = 1; i <= rimSegs; i++)
            {
                float t = (float)i / rimSegs;
                float r = Mathf.Lerp(innerR, R, t);
                // Ease-in: slow start, fast finish — rim drops steeply near the edge
                float ease = 1f - Mathf.Cos(t * Mathf.PI * 0.5f);
                float y    = Mathf.Lerp(rimY, edgeTopY, ease);
                profile.Add(new Vector2(r, y));
            }
            // Now at (R, edgeTopY).

            // 3. Tiny outer edge wall — from (R, edgeTopY) to (R, camberH).
            profile.Add(new Vector2(R, camberH));

            // 4. Lower rim camber — sweeps from (R, camberH) inward-down to (innerR, 0).
            //    Concave (e.g. Destroyer): surface bows UPWARD from the straight line
            //                             → a concave channel when viewed from below.
            //    Convex : surface bows DOWNWARD → rounded underbelly.
            //    Flat   : straight line.
            for (int i = 1; i <= camberSegs; i++)
            {
                float t      = (float)i / camberSegs;
                float r      = Mathf.Lerp(R, innerR, t);
                float yLin   = Mathf.Lerp(camberH, 0f, t);
                float offset = 0f;
                if (concave) offset = +camberH * 0.55f * Mathf.Sin(t * Mathf.PI);
                if (convex)  offset = -camberH * 0.55f * Mathf.Sin(t * Mathf.PI);
                profile.Add(new Vector2(r, Mathf.Max(0f, yLin + offset)));
            }
            // Lands at (innerR, 0).

            // 5. Cavity inner wall — step up from (innerR, 0) to (innerR, rD)
            profile.Add(new Vector2(innerR, rD));

            // 6. Cavity floor — from (innerR, rD) inward to centre
            profile.Add(new Vector2(0f, rD));

            return Revolve(profile, segments);
        }

        /// <summary>
        /// Build a fallback disc mesh using approximate Innova Destroyer dimensions.
        /// Safe to call with no DiscModel asset.
        /// </summary>
        public static Mesh BuildDefault(int segments = 32)
        {
            var tmp = ScriptableObject.CreateInstance<DiscModel>();
            // ~Innova Destroyer (driver)
            tmp.radius          = 0.106f;
            tmp.rimWidth        = 0.021f;
            tmp.thickness       = 0.018f;
            tmp.domeHeight      = 0.004f;
            tmp.rimDepth        = 0.012f;
            tmp.rimCamberHeight = 0.004f;
            tmp.rimCamberShape  = "Flat";
            var mesh = Build(tmp, segments);
            Object.Destroy(tmp);
            return mesh;
        }

        // ----------------------------------------------------------------
        // Surface of revolution — profile → full 3D mesh
        // ----------------------------------------------------------------
        static Mesh Revolve(List<Vector2> profile, int M)
        {
            int N = profile.Count;

            // Vertex layout: ring j (j=0..M inclusive) × profile point i (i=0..N-1)
            // Total: (M+1)*N vertices — the j=M ring duplicates j=0 so UV u=1 is distinct
            // from u=0 even though they share world position (needed for correct UV seam).
            int vertCount = (M + 1) * N;
            var verts = new Vector3[vertCount];
            var uvs   = new Vector2[vertCount];

            for (int j = 0; j <= M; j++)
            {
                float angle  = 2f * Mathf.PI * j / M;
                float cosA   = Mathf.Cos(angle);
                float sinA   = Mathf.Sin(angle);
                float uAngle = (float)j / M;

                for (int i = 0; i < N; i++)
                {
                    float r = profile[i].x;
                    float y = profile[i].y;
                    verts[j * N + i] = new Vector3(r * cosA, y, r * sinA);
                    uvs  [j * N + i] = new Vector2(uAngle, (float)i / (N - 1));
                }
            }

            // ---- Triangle winding ----
            // Winding analysis (Unity left-hand system, right-hand cross for normals):
            //   Full quad  (a,b,c) + (a,c,d) → outward-facing side/rim strips.
            //   aOnAxis    (a,c,d)            → top dome fan (normal up).
            //   dOnAxis    (a,b,c)            → bottom cavity fan (normal down).
            // Verified per-strip by symbolic cross-product; RecalculateNormals confirms.
            var tris = new List<int>(M * (N - 1) * 6);

            for (int j = 0; j < M; j++)
            {
                for (int i = 0; i < N - 1; i++)
                {
                    int a = j       * N + i;
                    int b = (j + 1) * N + i;
                    int c = (j + 1) * N + (i + 1);
                    int d = j       * N + (i + 1);

                    bool aOnAxis = profile[i    ].x < kAxisEps;
                    bool dOnAxis = profile[i + 1].x < kAxisEps;

                    if (aOnAxis && dOnAxis)
                        continue;   // degenerate — both on axis, skip

                    if (aOnAxis)
                    {
                        // Upper end on axis → fan triangle pointing upward
                        tris.Add(a); tris.Add(c); tris.Add(d);
                    }
                    else if (dOnAxis)
                    {
                        // Lower end on axis → fan triangle pointing downward
                        tris.Add(a); tris.Add(b); tris.Add(c);
                    }
                    else
                    {
                        // Full quad → two triangles
                        tris.Add(a); tris.Add(b); tris.Add(c);
                        tris.Add(a); tris.Add(c); tris.Add(d);
                    }
                }
            }

            var mesh = new Mesh { name = "DiscProceduralMesh" };
            mesh.SetVertices(verts);
            mesh.uv = uvs;
            mesh.SetTriangles(tris, 0);
            mesh.RecalculateNormals();
            mesh.RecalculateBounds();
            mesh.RecalculateTangents();
            return mesh;
        }
    }
}
