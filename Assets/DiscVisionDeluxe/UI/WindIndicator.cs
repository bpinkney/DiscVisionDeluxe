using System.Collections.Generic;
using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe.UI
{
    /// <summary>
    /// Animates a 3D windsock in an offscreen rig (y=5000) and renders it to
    /// SockRT for display as a UIToolkit background image.
    ///
    /// Rig hierarchy (all programmatic, moved to y=5000 in Awake):
    ///   WindIndicator
    ///     └── _SockAzimuthPivot  ← rotates around world Y (compass direction)
    ///           └── _SockMesh    ← MeshFilter; procedural mesh rebuilt each frame
    ///     └── _SockPreviewCam    ← fixed camera, renders SockRT
    ///
    /// Each mesh segment has its own lift threshold: segment s becomes horizontal
    /// when wind speed reaches (s + 1) × knotsPerSegment knots. Segments below
    /// their threshold hang with a pronounced droop and flutter with gusts.
    /// </summary>
    public class WindIndicator : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("References")]
        public WindField windField;

        [Header("Sock Material")]
        [Tooltip("URP Lit or Unlit material for the windsock mesh. Backface culling is disabled at runtime.")]
        public Material sockMaterial;

        [Header("Sock Geometry")]
        [Tooltip("Number of segments. Each segment lifts at knotsPerSegment increments.")]
        public int   sockSegments   = 6;
        public int   sockSides      = 8;
        public float sockLength     = 0.6f;
        public float sockBaseRadius = 0.08f;
        public float sockTipRadius  = 0.02f;

        [Header("Wind Behaviour")]
        [Tooltip("Wind speed in knots at which each successive segment lifts to horizontal.")]
        public float knotsPerSegment = 5f;

        [Header("Sock Dynamics")]
        [Range(0.5f, 20f)]
        public float springFrequency = 3f;
        [Range(0f, 2f)]
        public float springDamping   = 0.7f;

        [Header("Preview Render Texture")]
        public int sockRTWidth  = 64;
        public int sockRTHeight = 64;

        // ── Public API ───────────────────────────────────────────────────────

        public RenderTexture SockRT { get; private set; }

        // ── Spring state ─────────────────────────────────────────────────────

        float _azimuth;
        float _azimuthVel;
        // Single spring drives all segment elevations for coherent animation.
        float _springKnots;
        float _springKnotsVel;

        // ── Mesh state ───────────────────────────────────────────────────────

        Mesh      _mesh;
        Vector3[] _verts;
        float     _ripplePhase;

        // ── Scene rig (created programmatically in Awake) ────────────────────

        Transform  _azimuthPivot;
        MeshFilter _sockMeshFilter;

        // ── Unity lifecycle ──────────────────────────────────────────────────

        void Awake()
        {
            SockRT = new RenderTexture(sockRTWidth, sockRTHeight, 16, RenderTextureFormat.ARGB32);
            SockRT.Create();

            // Move entire rig far above scene — no custom layer needed.
            transform.position = new Vector3(0f, 5000f, 0f);

            // ── Azimuth pivot ────────────────────────────────────────────────
            var pivotGO = new GameObject("_SockAzimuthPivot") { hideFlags = HideFlags.DontSave };
            pivotGO.transform.SetParent(transform);
            pivotGO.transform.localPosition = Vector3.zero;
            _azimuthPivot = pivotGO.transform;

            // ── Sock mesh ────────────────────────────────────────────────────
            var meshGO = new GameObject("_SockMesh") { hideFlags = HideFlags.DontSave };
            meshGO.transform.SetParent(_azimuthPivot);
            meshGO.transform.localPosition = Vector3.zero;
            _sockMeshFilter = meshGO.AddComponent<MeshFilter>();
            var mr = meshGO.AddComponent<MeshRenderer>();
            if (sockMaterial != null)
            {
                mr.material = new Material(sockMaterial);
                if (mr.material.HasProperty("_Cull"))
                    mr.material.SetInt("_Cull", (int)UnityEngine.Rendering.CullMode.Off);
            }

            BuildSockMesh();

            // ── Preview camera ───────────────────────────────────────────────
            // Fixed relative to the rig (not under azimuth pivot) — the sock
            // rotates in view as wind direction changes, while the camera stays
            // at a consistent overhead-angled position to show droop clearly.
            var camGO = new GameObject("_SockPreviewCam") { hideFlags = HideFlags.DontSave };
            camGO.transform.SetParent(transform);
            Vector3 camPos  = new Vector3(0f, 0.7f, -0.3f);
            Vector3 lookAt  = new Vector3(0f, -0.1f, 0.3f);
            camGO.transform.localPosition = camPos;
            camGO.transform.localRotation = Quaternion.LookRotation(lookAt - camPos, Vector3.up);

            var cam = camGO.AddComponent<Camera>();
            cam.clearFlags      = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0f, 0f, 0f, 0f);
            cam.fieldOfView     = 55f;
            cam.nearClipPlane   = 0.05f;
            cam.farClipPlane    = 3.0f;
            cam.targetTexture   = SockRT;
            cam.depth           = -20;
        }

        void OnDestroy()
        {
            if (_mesh  != null) Destroy(_mesh);
            if (SockRT != null) { SockRT.Release(); Destroy(SockRT); }
        }

        void Update()
        {
            if (windField == null) return;

            // ── Azimuth: compass direction sock points toward ─────────────────
            float targetAzimuth = (windField.windDirectionDeg + 180f) % 360f;
            SpringDampAngle(ref _azimuth, ref _azimuthVel, targetAzimuth, Time.deltaTime);
            if (_azimuthPivot != null)
                _azimuthPivot.localEulerAngles = new Vector3(0f, _azimuth, 0f);

            // ── Effective knots: single spring drives all segment elevations ──
            SpringDampLinear(ref _springKnots, ref _springKnotsVel,
                             windField.WindSpeedKnots, Time.deltaTime);

            if (_sockMeshFilter != null)
                UpdateMesh();
        }

        // ── Mesh update ──────────────────────────────────────────────────────

        void UpdateMesh()
        {
            float gustT    = (float)windField.gustFactor / 10f;
            float gustFreq = 1f + gustT * 3f;
            _ripplePhase   = (_ripplePhase + Time.deltaTime * gustFreq * 2f * Mathf.PI) % (Mathf.PI * 2f);

            int     vPerRing = sockSides + 1;
            float   segLen   = sockLength / sockSegments;
            Vector3 pos      = Vector3.zero;

            for (int s = 0; s <= sockSegments; s++)
            {
                float t = s / (float)sockSegments;

                // Each ring has a threshold: ring s lifts at (s+1)*knotsPerSegment.
                // blend = 0 → hanging, blend = 1 → fully horizontal.
                float threshold = (s + 1) * knotsPerSegment;
                float blend     = Mathf.Clamp01(_springKnots / threshold);

                // Power-2 curve: sock hangs deep until near the threshold, then lifts sharply.
                float elevDeg = Mathf.Lerp(-90f, 0f, blend * blend);
                float elevRad = elevDeg * Mathf.Deg2Rad;
                float radius  = Mathf.Lerp(sockBaseRadius, sockTipRadius, t);

                // gustT gates all flutter — zero gusts means zero movement for every segment.
                // Drooping segments get droop² scaling on top; erect segments get a small
                // base term so they still ripple slightly in strong gusts.
                float droop      = 1f - blend;
                float flutterAmp = gustT * (0.01f + droop * droop * 0.05f);
                float ripple     = Mathf.Sin(_ripplePhase - t * Mathf.PI * 4f) * flutterAmp;

                // Ring frame: tangent along sock, N/B perpendicular so cross-sections stay circular.
                Vector3 tangent = new Vector3(0f, Mathf.Sin(elevRad), Mathf.Cos(elevRad));
                Vector3 N = Vector3.Cross(Vector3.up, tangent);
                if (N.sqrMagnitude < 0.001f) N = Vector3.right;
                N.Normalize();
                Vector3 B = Vector3.Cross(tangent, N).normalized;

                for (int v = 0; v <= sockSides; v++)
                {
                    float   angle  = v / (float)sockSides * 2f * Mathf.PI;
                    Vector3 radial = (Mathf.Cos(angle) * N + Mathf.Sin(angle) * B) * radius;
                    _verts[s * vPerRing + v] = pos + radial + N * ripple;
                }

                if (s < sockSegments)
                    pos += tangent * segLen;
            }

            _mesh.SetVertices(_verts);
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();
        }

        // ── Spring helpers ───────────────────────────────────────────────────

        void SpringDampAngle(ref float current, ref float vel, float target, float dt)
        {
            float omega = 2f * Mathf.PI * springFrequency;
            float accel = omega * omega * Mathf.DeltaAngle(current, target)
                        - 2f * springDamping * omega * vel;
            vel     += accel * dt;
            current += vel * dt;
            current  = ((current % 360f) + 360f) % 360f;
        }

        void SpringDampLinear(ref float current, ref float vel, float target, float dt)
        {
            float omega = 2f * Mathf.PI * springFrequency;
            float accel = omega * omega * (target - current)
                        - 2f * springDamping * omega * vel;
            vel     += accel * dt;
            current += vel * dt;
        }

        // ── Build mesh ────────────────────────────────────────────────────────

        void BuildSockMesh()
        {
            if (_sockMeshFilter == null) return;

            int vPerRing   = sockSides + 1;
            int totalVerts = vPerRing * (sockSegments + 1);

            _verts   = new Vector3[totalVerts];
            var uvs  = new Vector2[totalVerts];
            var tris = new List<int>(sockSides * sockSegments * 6);

            // Initial calm pose: hang straight down. Positions are overwritten every
            // frame by UpdateMesh; UVs are set once here and never change.
            for (int s = 0; s <= sockSegments; s++)
            {
                float   t      = s / (float)sockSegments;
                float   radius = Mathf.Lerp(sockBaseRadius, sockTipRadius, t);
                Vector3 pos    = new Vector3(0f, -sockLength * t, 0f);

                for (int v = 0; v <= sockSides; v++)
                {
                    float angle = v / (float)sockSides * 2f * Mathf.PI;
                    _verts[s * vPerRing + v] = pos + new Vector3(
                        Mathf.Cos(angle) * radius, 0f, Mathf.Sin(angle) * radius);
                    uvs[s * vPerRing + v] = new Vector2(v / (float)sockSides, t);
                }
            }

            for (int s = 0; s < sockSegments; s++)
            {
                for (int v = 0; v < sockSides; v++)
                {
                    int i0 =  s      * vPerRing + v;
                    int i1 =  s      * vPerRing + v + 1;
                    int i2 = (s + 1) * vPerRing + v;
                    int i3 = (s + 1) * vPerRing + v + 1;
                    tris.Add(i0); tris.Add(i1); tris.Add(i2);
                    tris.Add(i1); tris.Add(i3); tris.Add(i2);
                }
            }

            _mesh = new Mesh { name = "Windsock" };
            _mesh.SetVertices(_verts);
            _mesh.SetUVs(0, uvs);
            _mesh.SetTriangles(tris, 0);
            _mesh.RecalculateNormals();
            _mesh.RecalculateBounds();
            _sockMeshFilter.mesh = _mesh;
        }
    }
}
