using System.Collections;
using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe.UI
{
    /// <summary>
    /// Renders a copy of the disc mesh from behind into a RenderTexture for
    /// display in the Throw Results panel.
    ///
    /// Attach to any GameObject in the scene.
    /// Wire sourceDisc → the scene's DiscVisualController (the main disc mesh).
    ///
    /// The rig (camera + disc copy) is moved to y=5000 in Awake() so it never
    /// appears in the main cameras — no custom layer required.
    /// </summary>
    public class DiscPreviewController : MonoBehaviour
    {
        [Header("References")]
        [Tooltip("The main scene disc visual — preview mirrors its mesh and materials.")]
        public DiscVisionDeluxe.Visualization.DiscVisualController sourceDisc;

        [Header("Render texture")]
        public int rtWidth  = 288;
        public int rtHeight = 172;

        // ── Public API ───────────────────────────────────────────────────────
        public RenderTexture PreviewRT { get; private set; }

        // ── Private ──────────────────────────────────────────────────────────
        Transform    _discPivot;
        MeshFilter   _mf;
        MeshRenderer _mr;

        // ─────────────────────────────────────────────────────────────────────

        void Awake()
        {
            // Build RT
            PreviewRT = new RenderTexture(rtWidth, rtHeight, 16, RenderTextureFormat.ARGB32);
            PreviewRT.Create();

            // Move entire rig far above scene (no layer gymnastics needed)
            transform.position = new Vector3(0f, 5000f, 0f);

            // ── Disc pivot ───────────────────────────────────────────────────
            var discGO = new GameObject("_DiscPreviewMesh") { hideFlags = HideFlags.DontSave };
            discGO.transform.SetParent(transform);
            discGO.transform.localPosition = Vector3.zero;
            _discPivot = discGO.transform;
            _mf = discGO.AddComponent<MeshFilter>();
            _mr = discGO.AddComponent<MeshRenderer>();

            // ── Preview camera ───────────────────────────────────────────────
            // From in front of the disc, slightly above — nearly parallel to ground.
            // Shows hyzer as left/right tilt and nose angle as face pitch.
            Vector3 camOffset = new Vector3(0f, 0.4f, 1.3f);
            var camGO = new GameObject("_DiscPreviewCam") { hideFlags = HideFlags.DontSave };
            camGO.transform.SetParent(transform);
            camGO.transform.localPosition = camOffset;
            camGO.transform.localRotation = Quaternion.LookRotation(-camOffset.normalized, Vector3.up);

            var cam = camGO.AddComponent<UnityEngine.Camera>();
            cam.clearFlags      = CameraClearFlags.SolidColor;
            cam.backgroundColor = new Color(0f, 0f, 0f, 0f); // transparent — disc floats on panel bg
            cam.fieldOfView     = 14f;
            cam.nearClipPlane   = 0.05f;
            cam.farClipPlane    = 3.0f;
            cam.targetTexture   = PreviewRT;
            cam.depth           = -20; // render before main cameras
        }

        IEnumerator Start()
        {
            yield return null; // wait one frame — sourceDisc.Awake() guaranteed done
            MirrorDiscAppearance();
        }

        void OnDestroy()
        {
            if (PreviewRT != null) { PreviewRT.Release(); Destroy(PreviewRT); }
        }

        // ── Public methods ───────────────────────────────────────────────────

        /// <summary>
        /// Orient the preview disc to match the given throw angles.
        /// Uses the same axis convention as DiscFlightSimulator.ToUnityRotation:
        ///   transform.up      = disc top-face normal
        ///   transform.forward = throw direction (+Z)
        /// </summary>
        public void UpdateOrientation(float hyzerDeg, float noseAngleDeg)
        {
            if (_discPivot == null) return;

            float h = hyzerDeg      * Mathf.Deg2Rad;
            float p = noseAngleDeg  * Mathf.Deg2Rad;

            // Disc normal in DfisX frame (ThrowParameters.DiscNormal() formula)
            float nx = Mathf.Sin(-p) * Mathf.Cos(h);
            float ny = Mathf.Sin(h)  * Mathf.Cos(p);
            float nz = Mathf.Cos(p)  * Mathf.Cos(h);
            // DfisX (x, y, z) → Unity (x, z, y)
            Vector3 discNormalUnity = new Vector3(nx, nz, ny);

            _discPivot.localRotation = Quaternion.LookRotation(Vector3.forward, discNormalUnity);
        }

        /// <summary>Copy mesh + material from the world disc so colours match.</summary>
        public void MirrorDiscAppearance()
        {
            if (_mf == null || _mr == null || sourceDisc == null) return;

            var srcMF = sourceDisc.GetComponent<MeshFilter>();
            var srcMR = sourceDisc.GetComponent<MeshRenderer>();

            if (srcMF?.sharedMesh != null)
                _mf.sharedMesh = srcMF.sharedMesh;

            if (srcMR != null)
            {
                var orig = srcMR.sharedMaterials;
                var copy = new Material[orig.Length];
                for (int i = 0; i < orig.Length; i++)
                    copy[i] = orig[i] != null ? new Material(orig[i]) : null;
                _mr.sharedMaterials = copy;
            }
        }
    }
}
