using System.Collections.Generic;
using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe.Visualization
{
    /// <summary>
    /// Draws a ghost trajectory preview using a synchronous DfisX simulation.
    ///
    /// POL-1: Call RequestUpdate() whenever throw parameters change.
    /// The component self-throttles to 5Hz so it can be called from every slider event.
    ///
    /// Wire up:
    ///   - Assign a LineRenderer to previewLine (or let it auto-create on the same GameObject).
    ///   - Assign the same DiscModelLibrary used by DiscVisualizer.
    ///   - Call RequestUpdate() from ThrowParameterPanelController each time a slider changes.
    /// </summary>
    public class ShotPreviewLine : MonoBehaviour
    {
        // ---------------------------------------------------------------
        // Inspector
        // ---------------------------------------------------------------
        [Header("References")]
        [Tooltip("LineRenderer used to draw the ghost arc. Auto-created on this GameObject if not assigned.")]
        public LineRenderer previewLine;

        [Tooltip("Same DiscModelLibrary asset used by DiscVisualizer.")]
        public DiscModelLibrary discModelLibrary;

        [Header("Style")]
        [Tooltip("Base color of the preview line. Alpha is used at the start; the end fades to zero.")]
        public Color previewColor = new Color(1f, 1f, 1f, 0.55f);

        [Tooltip("Width at the start (tee end) of the ghost trail.")]
        public float previewWidth = 0.04f;

        [Header("Sampling")]
        [Tooltip("Emit one position every N simulation steps. 5 → ~200 points for a ~7s flight.")]
        [Range(1, 20)]
        public int sampleEveryNSteps = 5;

        [Tooltip("Maximum updates per second. Keeps CPU load low while sliders are dragged.")]
        [Range(1f, 30f)]
        public float maxUpdatesPerSecond = 5f;

        // ---------------------------------------------------------------
        // Private
        // ---------------------------------------------------------------
        bool               _dirty;
        float              _nextAllowedUpdate;

        ThrowParameters    _pendingParams;
        DiscEnvironment    _pendingEnv;
        AeroDebugSettings  _pendingAero;

        DiscFlightSimulator _previewSim;

        // ---------------------------------------------------------------
        // Unity lifecycle
        // ---------------------------------------------------------------
        void Awake()
        {
            if (previewLine == null)
            {
                previewLine = gameObject.AddComponent<LineRenderer>();
                previewLine.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                previewLine.receiveShadows    = false;
            }

            // Ensure material supports vertex colors
            if (previewLine.sharedMaterial == null)
            {
                Shader shader = Shader.Find("Universal Render Pipeline/Particles/Unlit")
                             ?? Shader.Find("Sprites/Default");
                if (shader != null)
                    previewLine.material = new Material(shader) { color = Color.white };
            }

            previewLine.useWorldSpace = true;
            previewLine.positionCount = 0;
            previewLine.startWidth    = previewWidth;
            previewLine.endWidth      = 0f;
            ApplyGradient();
        }

        void Update()
        {
            if (!_dirty || discModelLibrary == null || previewLine == null) return;
            if (Time.unscaledTime < _nextAllowedUpdate) return;

            _dirty             = false;
            _nextAllowedUpdate = Time.unscaledTime + 1f / maxUpdatesPerSecond;

            RebuildPreview();
        }

        void OnDestroy()
        {
            _previewSim?.Container?.Dispose();
        }

        // ---------------------------------------------------------------
        // Public API
        // ---------------------------------------------------------------

        /// <summary>
        /// Request a preview update with new throw parameters.
        /// Safe to call every slider event — internally throttled to maxUpdatesPerSecond.
        /// </summary>
        public void RequestUpdate(ThrowParameters p, DiscEnvironment env,
                                  AeroDebugSettings aero = default)
        {
            _pendingParams = p;
            _pendingEnv    = env;
            _pendingAero   = aero;
            _dirty         = true;
        }

        /// <summary>Hide the ghost trail (e.g. once a real throw begins).</summary>
        public void HidePreview()
        {
            _dirty = false;
            if (previewLine != null)
                previewLine.positionCount = 0;
        }

        // ---------------------------------------------------------------
        // Internal
        // ---------------------------------------------------------------
        void RebuildPreview()
        {
            if (_previewSim == null)
                _previewSim = new DiscFlightSimulator(discModelLibrary);

            // Run full throw synchronously (~1ms for a normal disc flight)
            _previewSim.NewThrow(_pendingParams, _pendingEnv, _pendingAero);
            _previewSim.SimulateThrow();

            var container = _previewSim.Container;
            int count     = container.discStateCount;

            if (count == 0)
            {
                previewLine.positionCount = 0;
                return;
            }

            var pts = new List<Vector3>(count / sampleEveryNSteps + 2);

            for (int i = 0; i < count; i += sampleEveryNSteps)
                pts.Add(DiscFlightSimulator.ToUnitySpace(container.discStateArray[i].discLocation));

            // Always close with the landing point
            Vector3 landing = DiscFlightSimulator.ToUnitySpace(
                container.discStateArray[count - 1].discLocation);
            if (pts.Count == 0 || pts[pts.Count - 1] != landing)
                pts.Add(landing);

            previewLine.positionCount = pts.Count;
            previewLine.SetPositions(pts.ToArray());
        }

        void ApplyGradient()
        {
            if (previewLine == null) return;
            var grad = new Gradient();
            grad.SetKeys(
                new[] { new GradientColorKey(previewColor, 0f), new GradientColorKey(previewColor, 1f) },
                new[] { new GradientAlphaKey(previewColor.a, 0f), new GradientAlphaKey(0f, 1f) }
            );
            previewLine.colorGradient = grad;
        }
    }
}
