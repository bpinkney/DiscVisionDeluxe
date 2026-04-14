using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe.Visualization
{
    /// <summary>
    /// Manages the visual appearance of the disc GameObject:
    ///   • Rebuilds a procedural disc mesh from DiscModel physical dimensions.
    ///   • Creates and maintains a URP Lit material with configurable colour/sheen.
    ///   • Assigns a deterministic random colour per disc mold — same colour every
    ///     run for the same mold, seeded from manufacturer+moldName.
    ///
    /// Attach to the same GameObject that carries the disc's MeshFilter + MeshRenderer.
    /// Wire DiscVisualizer.discVisualController → this component in the Inspector.
    /// </summary>
    [RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
    public class DiscVisualController : MonoBehaviour
    {
        // ---------------------------------------------------------------
        // Inspector
        // ---------------------------------------------------------------
        [Header("Appearance")]
        [Range(0f, 1f)]
        [Tooltip("Surface smoothness. 0 = matte, 1 = mirror.")]
        public float smoothness = 0.8f;

        [Header("Fallback Model")]
        [Tooltip("Model used when LaunchDfisX has no directModel (e.g. live throws or index-only throws).")]
        public DiscModel fallbackModel;

        [Header("Mesh Quality")]
        [Tooltip("Angular subdivisions around the spin axis. 32 is smooth; raise to 48+ for close-ups.")]
        public int revolutionSegments = 32;

        [Tooltip("Subdivisions along the dome curve. More = smoother dome profile.")]
        public int domeSubdivisions = 10;

        // ---------------------------------------------------------------
        // Private
        // ---------------------------------------------------------------
        MeshFilter      _mf;
        MeshRenderer    _mr;
        Material        _mat;
        DiscModel       _activeModel;
        Color           _activeColor = Color.white;
        DiscFoilStamp   _foilStamp;

        // ---------------------------------------------------------------
        // Unity lifecycle
        // ---------------------------------------------------------------
        void Awake()
        {
            _mf = GetComponent<MeshFilter>();
            _mr = GetComponent<MeshRenderer>();
            EnsureMaterial();
            _foilStamp = GetComponent<DiscFoilStamp>()
                      ?? gameObject.AddComponent<DiscFoilStamp>();
        }

        void Start()
        {
            if (_mf.sharedMesh == null || _mf.sharedMesh.name != "DiscProceduralMesh")
                RebuildMesh(_activeModel);

            if (fallbackModel != null)
            {
                _activeColor = DiscColorPalette.ForModel(fallbackModel);
                ApplyMaterialProperties();
                _foilStamp?.SetDiscModel(fallbackModel);
            }
        }

        // ---------------------------------------------------------------
        // Public API
        // ---------------------------------------------------------------
        /// <summary>
        /// Rebuild the disc mesh and update the colour to match the given model.
        /// Pass null to fall back to fallbackModel (or built-in defaults).
        /// </summary>
        public void SetDiscModel(DiscModel model)
        {
            _activeModel = model;
            _activeColor = DiscColorPalette.ForModel(model != null ? model : fallbackModel);
            RebuildMesh(_activeModel);
            ApplyMaterialProperties();
            _foilStamp?.SetDiscModel(model != null ? model : fallbackModel);
        }

        // ---------------------------------------------------------------
        // Colour — delegated to shared DiscColorPalette
        // ---------------------------------------------------------------

        // ---------------------------------------------------------------
        // Mesh
        // ---------------------------------------------------------------
        void RebuildMesh(DiscModel model)
        {
            DiscModel effective = model != null ? model : fallbackModel;

            Mesh mesh = effective != null
                ? DiscMeshBuilder.Build(effective, revolutionSegments, domeSubdivisions)
                : DiscMeshBuilder.BuildDefault(revolutionSegments);

            if (_mf.sharedMesh != null && _mf.sharedMesh.name == "DiscProceduralMesh")
                Destroy(_mf.sharedMesh);

            _mf.mesh = mesh;
            transform.localScale = Vector3.one;
        }

        // ---------------------------------------------------------------
        // Material
        // ---------------------------------------------------------------
        void EnsureMaterial()
        {
            if (_mat != null) return;

            Shader shader = Shader.Find("Universal Render Pipeline/Lit")
                         ?? Shader.Find("Standard");

            if (shader == null)
            {
                Debug.LogWarning("[DiscVisualController] Neither URP/Lit nor Standard shader found. " +
                                 "Assign a material manually.");
                return;
            }

            _mat = new Material(shader) { name = "Disc_Procedural_Mat" };
            ApplyMaterialProperties();
            _mr.material = _mat;
        }

        void ApplyMaterialProperties()
        {
            if (_mat == null) return;
            _mat.SetColor("_BaseColor",  _activeColor);
            _mat.SetColor("_Color",      _activeColor);  // Standard fallback
            _mat.SetFloat("_Smoothness", smoothness);
            _mat.SetFloat("_Glossiness", smoothness);    // Standard fallback
            _mat.SetFloat("_Metallic",   0f);
        }

#if UNITY_EDITOR
        void OnValidate()
        {
            if (_mat != null)
                ApplyMaterialProperties();
        }
#endif
    }
}
