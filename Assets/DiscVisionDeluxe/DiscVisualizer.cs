using System.Collections.Generic;
using UnityEngine;
using TMPro;
using DfisX;
using DiscVisionDeluxe.Visualization;

namespace DiscVisionDeluxe
{
    /// <summary>
    /// Visualizes disc state from the Kalman Filter (KF phase),
    /// then launches DfisX flight simulation once the KF ideal state is ready,
    /// and hands off to a Rigidbody at landing.
    ///
    /// Can also be launched directly by DiscThrowDebugger without going
    /// through the KF pipeline — call LaunchDfisX(DiscInitState) publicly.
    ///
    /// Attach to a GameObject that has:
    ///   - A child transform for the disc mesh (assign to discTransform)
    ///   - A LineRenderer component for the KF trail
    ///   - A Rigidbody (isKinematic = true in prefab)
    ///   - A Collider
    ///   - Optionally a TMP_Text for HUD
    ///
    /// Wire up via Inspector:
    ///   DiscSimulator.onNewState        → DiscVisualizer.OnStateUpdate
    ///   DiscSimulator.onIdealStateReady → DiscVisualizer.OnIdealState
    ///   DiscSimulator.onThrowComplete   → DiscVisualizer.OnThrowComplete
    /// </summary>
    public class DiscVisualizer : MonoBehaviour
    {
        // ---------------------------------------------------------------
        // Inspector — KF visualisation (existing)
        // ---------------------------------------------------------------
        [Header("Disc Object")]
        [Tooltip("Transform of the disc mesh to move and rotate")]
        public Transform discTransform;

        [Tooltip("Scale factor: DVDvis uses metres, Unity default is metres — leave at 1 unless your scene is scaled")]
        public float sceneScale = 1f;

        [Header("KF Trajectory Trail")]
        public LineRenderer trajectoryLine;
        [Tooltip("Maximum number of points to store in the KF trail")]
        public int maxTrailPoints = 500;
        public Gradient trailColorBySpeed;
        public float maxSpeedKph = 120f;

        [Header("Ideal State Marker")]
        [Tooltip("A small object placed at the ideal (minimum variance) state position")]
        public Transform idealMarker;

        [Header("HUD")]
        public TMP_Text hudLabel;

        [Header("Coordinate Frame")]
        [Tooltip("DVDvis frame: X=forward, Y=right, Z=up. Unity: X=right, Y=up, Z=forward.")]
        public bool remapCoordinateFrame = true;

        // ---------------------------------------------------------------
        // Inspector — DfisX flight simulation
        // ---------------------------------------------------------------
        [Header("DfisX Flight Simulation")]
        [Tooltip("Library of disc models. Create via Assets > Create > DfisX > Disc Model Library")]
        public DiscModelLibrary discModelLibrary;

        [Header("Disc Mesh & Material")]
        [Tooltip("Attach DiscVisualController to the disc mesh GameObject and wire it here. " +
                 "It rebuilds the procedural disc mesh whenever the selected model changes.")]
        public DiscVisionDeluxe.Visualization.DiscVisualController discVisualController;

        [Tooltip("Wind vector in DfisX Z-up world frame (m/s). X=forward, Y=right, Z=up.")]
        public Vector3 windVectorZUp = Vector3.zero;

        [Tooltip("Air density (kg/m3). ISA sea-level = 1.225")]
        public float airDensity = 1.225f;

        [Header("DfisX Rigidbody Handoff")]
        [Tooltip("Rigidbody on this GameObject. Set isKinematic = true in prefab.")]
        public Rigidbody discRigidbody;

        public float landingLinearDrag  = 2.0f;
        public float landingAngularDrag = 5.0f;

        [Tooltip("Scale spin rate to Rigidbody angular velocity at landing. Tune against real footage.")]
        [Range(0f, 1f)]
        public float spinTransferFactor = 0.3f;

        // ---------------------------------------------------------------
        // Inspector — DfisX trajectory trail
        // ---------------------------------------------------------------
        [Header("Landing Marker")]
        [Tooltip("Show a floating distance readout when the disc lands.")]
        public bool showLandingMarker = true;

        [Tooltip("Height above the landing point where the marker first appears (metres).")]
        public float landingMarkerHeightM = 1.5f;

        [Header("DfisX Trajectory Trail")]
        [Tooltip("LineRenderer used to draw the DfisX flight arc. Assign a separate LineRenderer from the KF trail.")]
        public LineRenderer dfisxTrajectoryLine;

        [Tooltip("Default width of the DfisX trajectory line.")]
        public float dfisxTrailWidth = 0.05f;

        [Tooltip("Default color of the DfisX trajectory line (manual throws without a disc-type override).")]
        public Color dfisxTrailColor = Color.yellow;

        [Tooltip("When true, each throw adds a new arc instead of replacing the previous one.")]
        public bool keepAllThrows = false;

        [Header("Trail — Live Throw Style")]
        [Tooltip("Trail color used when the throw originated from the live camera (LaunchDfisXFromLive).")]
        public Color liveThrowTrailColor = new Color(1f, 0.45f, 0f); // orange

        [Tooltip("Trail width for live camera throws.")]
        public float liveThrowTrailWidth = 0.08f;

        [Header("Trail — Per-Disc-Type Colors (Keep All Throws)")]
        [Tooltip("When keepAllThrows is enabled, each disc type can have its own trail color and width. " +
                 "Types not listed here fall back to dfisxTrailColor / dfisxTrailWidth.")]
        public List<DiscTypeTrailStyle> discTypeTrailStyles = new List<DiscTypeTrailStyle>();

        [System.Serializable]
        public class DiscTypeTrailStyle
        {
            public DiscIndex discType;
            public Color     color = Color.white;
            [Range(0.01f, 0.3f)]
            public float     width = 0.05f;
        }

        private enum ThrowSource { Manual, Live }

        // ---------------------------------------------------------------
        // Private — KF
        // ---------------------------------------------------------------
        private readonly List<Vector3> _trailPoints = new List<Vector3>();
        private DiscInitState _lastIdealState;

        // ---------------------------------------------------------------
        // Private — DfisX
        // ---------------------------------------------------------------
        private DiscFlightSimulator _sim;
        private bool _isSimFlying = false;

        // Accumulated DfisX trail points for the current throw
        private readonly List<Vector3> _dfisxTrailPoints = new List<Vector3>();

        // When keepAllThrows is true, we spawn new LineRenderers rather than
        // reusing dfisxTrajectoryLine. They are tracked here for ClearAllTrails().
        private readonly List<LineRenderer> _archivedTrails = new List<LineRenderer>();

        // ---------------------------------------------------------------
        // Public state
        // ---------------------------------------------------------------
        public bool IsSimFlying  => _isSimFlying;
        public DfisX.FlightStats LastFlightStats { get; private set; }
        public DfisX.ThrowContainer ActiveContainer => _sim?.Container;

        /// <summary>Fired when DfisX flight ends. Payload contains all throw statistics.</summary>
        public event System.Action<DfisX.FlightStats> OnSimFinished;
        /// <summary>Fired immediately after NewThrow() — container is initialised and ready.</summary>
        public event System.Action OnSimStarted;

        public IReadOnlyList<LineRenderer> GetArchivedTrails() => _archivedTrails;

        // ---------------------------------------------------------------
        // Unity lifecycle
        // ---------------------------------------------------------------
        void Awake()
        {
            if (trajectoryLine != null)
            {
                trajectoryLine.positionCount = 0;
                trajectoryLine.useWorldSpace = true;
            }

            if (dfisxTrajectoryLine != null)
            {
                // Auto-create a vertex-color-capable material if none is assigned.
                // A null material renders as pink/magenta in URP and ignores startColor/endColor.
                if (dfisxTrajectoryLine.sharedMaterial == null)
                {
                    Shader shader = Shader.Find("Universal Render Pipeline/Particles/Unlit")
                                 ?? Shader.Find("Sprites/Default");
                    if (shader != null)
                        dfisxTrajectoryLine.material = new Material(shader) { color = Color.white };
                }

                dfisxTrajectoryLine.positionCount = 0;
                dfisxTrajectoryLine.useWorldSpace = true;
                dfisxTrajectoryLine.startWidth    = dfisxTrailWidth;
                dfisxTrajectoryLine.endWidth      = dfisxTrailWidth;
                dfisxTrajectoryLine.startColor    = dfisxTrailColor;
                dfisxTrajectoryLine.endColor      = dfisxTrailColor;
            }

            if (discRigidbody != null)
                discRigidbody.isKinematic = true;
        }

        void Update()
        {
            if (!_isSimFlying || _sim == null) return;

            _sim.StepForUnityFrame(Time.deltaTime);

            if (!_sim.IsFinished)
            {
                DfisX.DiscState ds = _sim.Container.currentDiscState;
                ApplySimStateToTransform(ds);
                AppendDfisxTrailPoint(DiscFlightSimulator.ToUnitySpace(ds.discLocation) * sceneScale);
            }
        }

        void OnDestroy()
        {
            _sim?.Container?.Dispose();
        }

        // ---------------------------------------------------------------
        // Called by DiscSimulator.onNewState  (existing — unchanged)
        // ---------------------------------------------------------------
        public void OnStateUpdate(KFState state)
        {
            if (state == null) return;

            Vector3 pos = RemapPosition(state.LinearPosition) * sceneScale;
            Quaternion rot = RemapRotation(state.DiscOrientation);

            if (discTransform != null)
            {
                discTransform.position = pos;
                discTransform.rotation = rot;
            }

            AppendTrailPoint(pos, state.LinearSpeedKph);
            UpdateHUD(state, null);
        }

        // ---------------------------------------------------------------
        // Called by DiscSimulator.onIdealStateReady  (existing — unchanged)
        // ---------------------------------------------------------------
        public void OnIdealState(KFState state)
        {
            if (state == null) return;

            Vector3 pos = RemapPosition(state.LinearPosition) * sceneScale;

            if (idealMarker != null)
                idealMarker.position = pos;

            UpdateHUD(state, "IDEAL");
        }

        // ---------------------------------------------------------------
        // Called by DiscSimulator.onThrowComplete  (extended)
        // ---------------------------------------------------------------
        public void OnThrowComplete(DiscInitState initState)
        {
            _lastIdealState = initState;
            ShowThrowSummary(initState);
            LaunchDfisX(initState);
        }

        // ---------------------------------------------------------------
        // DfisX launch — PUBLIC so DiscThrowDebugger can call it directly
        // ---------------------------------------------------------------
        public void LaunchDfisX(DiscInitState initState)
        {
            if (discModelLibrary == null)
            {
                Debug.LogWarning("[DiscVisualizer] discModelLibrary not assigned — DfisX flight skipped.");
                return;
            }

            // Convert DiscInitState → ThrowParameters
            // Both use Z-up frame — no axis remapping needed
            var p = new DfisX.ThrowParameters
            {
                position    = new Unity.Mathematics.float3(
                    initState.linearPositionM.x,
                    initState.linearPositionM.y,
                    initState.linearPositionM.z),
                velocity    = new Unity.Mathematics.float3(
                    initState.linearVelocityMs.x,
                    initState.linearVelocityMs.y,
                    initState.linearVelocityMs.z),
                hyzer       = initState.angularPositionRad.x,
                pitch       = initState.angularPositionRad.y,
                spinRate    = initState.angularVelocityRad.z,
                wobble      = initState.wobble,
                discIndex   = (DfisX.DiscLayoutIndex)(int)initState.discMold,
                directModel = initState.directModel
            };

            var env = DfisX.DiscEnvironment.Default;
            env.windVectorXYZ = new Unity.Mathematics.float3(
                windVectorZUp.x, windVectorZUp.y, windVectorZUp.z);
            env.airDensity = airDensity;

            discVisualController?.SetDiscModel(initState.directModel);
            _StartDfisxFlight(p, env, default);
        }

        /// <summary>
        /// LaunchDfisX overload that also applies custom AeroDebugSettings.
        /// Called by ThrowParameterPanelController when aero sliders are modified.
        /// </summary>
        public void LaunchDfisX(DiscInitState initState, DfisX.AeroDebugSettings aeroDebug)
        {
            if (discModelLibrary == null)
            {
                Debug.LogWarning("[DiscVisualizer] discModelLibrary not assigned — DfisX flight skipped.");
                return;
            }

            var p = new DfisX.ThrowParameters
            {
                position    = new Unity.Mathematics.float3(
                    initState.linearPositionM.x,
                    initState.linearPositionM.y,
                    initState.linearPositionM.z),
                velocity    = new Unity.Mathematics.float3(
                    initState.linearVelocityMs.x,
                    initState.linearVelocityMs.y,
                    initState.linearVelocityMs.z),
                hyzer       = initState.angularPositionRad.x,
                pitch       = initState.angularPositionRad.y,
                spinRate    = initState.angularVelocityRad.z,
                wobble      = initState.wobble,
                discIndex   = (DfisX.DiscLayoutIndex)(int)initState.discMold,
                directModel = initState.directModel
            };

            var env = DfisX.DiscEnvironment.Default;
            env.windVectorXYZ = new Unity.Mathematics.float3(
                windVectorZUp.x, windVectorZUp.y, windVectorZUp.z);
            env.airDensity = airDensity;

            discVisualController?.SetDiscModel(initState.directModel);
            _StartDfisxFlight(p, env, aeroDebug);
        }

        /// <summary>
        /// Called by LiveDiscTracker.onThrowComplete — marks the throw as coming from the live
        /// camera so the trail is rendered in the live-throw style.
        /// Wire up: LiveDiscTracker.onThrowComplete → DiscVisualizer.LaunchDfisXFromLive
        /// </summary>
        public void LaunchDfisXFromLive(DiscInitState initState)
        {
            if (discModelLibrary == null)
            {
                Debug.LogWarning("[DiscVisualizer] discModelLibrary not assigned — DfisX flight skipped.");
                return;
            }

            var p = new DfisX.ThrowParameters
            {
                position  = new Unity.Mathematics.float3(
                    initState.linearPositionM.x,
                    initState.linearPositionM.y,
                    initState.linearPositionM.z),
                velocity  = new Unity.Mathematics.float3(
                    initState.linearVelocityMs.x,
                    initState.linearVelocityMs.y,
                    initState.linearVelocityMs.z),
                hyzer     = initState.angularPositionRad.x,
                pitch     = initState.angularPositionRad.y,
                spinRate  = initState.angularVelocityRad.z,
                wobble    = initState.wobble,
                discIndex = (DfisX.DiscLayoutIndex)(int)initState.discMold
            };

            var env = DfisX.DiscEnvironment.Default;
            env.windVectorXYZ = new Unity.Mathematics.float3(
                windVectorZUp.x, windVectorZUp.y, windVectorZUp.z);
            env.airDensity = airDensity;

            _StartDfisxFlight(p, env, default, ThrowSource.Live);
        }

        // Internal common launch path used by both LaunchDfisX() overloads.
        private void _StartDfisxFlight(DfisX.ThrowParameters p, DfisX.DiscEnvironment env,
                                        DfisX.AeroDebugSettings aeroDebug = default,
                                        ThrowSource source = ThrowSource.Manual)
        {
            if (_sim == null)
                _sim = new DiscFlightSimulator(discModelLibrary);

            _sim.OnThrowFinished -= HandleSimFinished;
            _sim.OnThrowFinished += HandleSimFinished;

            if (discRigidbody != null)
                discRigidbody.isKinematic = true;

            // Handle trail archiving / clearing
            PrepareNewDfisxTrail(p.discIndex, p.directModel, source);

            _sim.NewThrow(p, env, aeroDebug);
            OnSimStarted?.Invoke();
            _isSimFlying = true;

            ApplySimStateToTransform(_sim.Container.currentDiscState);
        }

        // ---------------------------------------------------------------
        // DfisX trail management
        // ---------------------------------------------------------------
        private void PrepareNewDfisxTrail(DfisX.DiscLayoutIndex discLayout, DfisX.DiscModel directModel, ThrowSource source)
        {
            _dfisxTrailPoints.Clear();

            if (dfisxTrajectoryLine == null) return;

            // Determine color/width for the *incoming* throw.
            // Manual throws use the per-mold palette colour; live throws keep their
            // distinct orange so they're always identifiable in keepAllThrows mode.
            Color newColor;
            float newWidth;
            if (source == ThrowSource.Live)
            {
                newColor = liveThrowTrailColor;
                newWidth = liveThrowTrailWidth;
            }
            else
            {
                // Use the disc-model palette if a directModel is available;
                // fall back to the legacy discTypeTrailStyles / dfisxTrailColor otherwise.
                if (directModel != null)
                {
                    newColor = DiscVisionDeluxe.Visualization.DiscColorPalette.TrailForModel(directModel);
                    newWidth = dfisxTrailWidth;
                }
                else if (keepAllThrows)
                {
                    GetDiscTypeStyle((DiscIndex)(int)discLayout, out newColor, out newWidth);
                }
                else
                {
                    newColor = dfisxTrailColor;
                    newWidth = dfisxTrailWidth;
                }
            }

            if (keepAllThrows)
            {
                // Archive the current LineRenderer's content by spawning a
                // new GameObject with a copy that preserves the previous throw's style,
                // then reset the main one with the new throw's style.
                if (dfisxTrajectoryLine.positionCount > 0)
                {
                    var archived = new GameObject("DfisxTrail_archived");
                    archived.transform.SetParent(transform.parent);
                    var lr = archived.AddComponent<LineRenderer>();
                    lr.useWorldSpace = true;
                    // Preserve the style that was applied to the active line
                    lr.startWidth    = dfisxTrajectoryLine.startWidth;
                    lr.endWidth      = dfisxTrajectoryLine.endWidth;
                    lr.startColor    = dfisxTrajectoryLine.startColor;
                    lr.endColor      = dfisxTrajectoryLine.endColor;
                    lr.sharedMaterial = dfisxTrajectoryLine.sharedMaterial;

                    int count = dfisxTrajectoryLine.positionCount;
                    lr.positionCount = count;
                    Vector3[] pts = new Vector3[count];
                    dfisxTrajectoryLine.GetPositions(pts);
                    lr.SetPositions(pts);

                    _archivedTrails.Add(lr);
                }
            }

            // Apply the new throw's style and clear the active line
            dfisxTrajectoryLine.startColor    = newColor;
            dfisxTrajectoryLine.endColor      = newColor;
            dfisxTrajectoryLine.startWidth    = newWidth;
            dfisxTrajectoryLine.endWidth      = newWidth;
            dfisxTrajectoryLine.positionCount = 0;
        }

        /// <summary>Returns the configured trail color and width for a disc type, or the
        /// dfisxTrailColor/dfisxTrailWidth defaults if no entry is configured for that type.</summary>
        private void GetDiscTypeStyle(DiscIndex discType, out Color color, out float width)
        {
            foreach (var style in discTypeTrailStyles)
            {
                if (style.discType == discType)
                {
                    color = style.color;
                    width = style.width;
                    return;
                }
            }
            color = dfisxTrailColor;
            width = dfisxTrailWidth;
        }

        private void AppendDfisxTrailPoint(Vector3 worldPos)
        {
            _dfisxTrailPoints.Add(worldPos);
            if (dfisxTrajectoryLine == null) return;
            dfisxTrajectoryLine.positionCount = _dfisxTrailPoints.Count;
            dfisxTrajectoryLine.SetPositions(_dfisxTrailPoints.ToArray());
        }

        /// <summary>
        /// Clear the active DfisX trail and all archived trails.
        /// </summary>
        public void ClearAllDfisxTrails()
        {
            _dfisxTrailPoints.Clear();
            if (dfisxTrajectoryLine != null)
                dfisxTrajectoryLine.positionCount = 0;

            foreach (var lr in _archivedTrails)
                if (lr != null) Destroy(lr.gameObject);
            _archivedTrails.Clear();
        }

        // ---------------------------------------------------------------
        // DfisX per-frame transform update
        // ---------------------------------------------------------------
        private void ApplySimStateToTransform(DfisX.DiscState ds)
        {
            if (discTransform != null)
            {
                discTransform.position = DiscFlightSimulator.ToUnitySpace(ds.discLocation) * sceneScale;
                discTransform.rotation = DiscFlightSimulator.ToUnityRotation(ds.discOrientZVect, ds.discOrientXVect);
            }
        }

        // ---------------------------------------------------------------
        // DfisX landing handoff
        // ---------------------------------------------------------------
        private void HandleSimFinished(DfisX.DiscState finalState, DfisX.FlightStats stats)
        {
            _isSimFlying  = false;
            LastFlightStats = stats;

            ApplySimStateToTransform(finalState);

            if (discRigidbody != null)
                HandoffToRigidbody(finalState);

            if (showLandingMarker && stats.distanceM > 0f)
            {
                Vector3 landingPos = DiscFlightSimulator.ToUnitySpace(stats.landingLocation) * sceneScale;
                landingPos.y += landingMarkerHeightM;
                LandingMarker.Spawn(landingPos, stats.distanceM);
            }

            OnSimFinished?.Invoke(stats);
            Debug.Log($"[DiscVisualizer] Flight complete. {stats}");
        }

        private void HandoffToRigidbody(DfisX.DiscState ds)
        {
            Vector3 vel        = DiscFlightSimulator.ToUnitySpace(ds.discVelocity) * sceneScale;
            Vector3 discNormal = DiscFlightSimulator.ToUnitySpace(ds.discOrientZVect).normalized;
            Vector3 discX      = DiscFlightSimulator.ToUnitySpace(ds.discOrientXVect).normalized;
            Vector3 discY      = DiscFlightSimulator.ToUnitySpace(ds.discOrientYVect).normalized;

            Vector3 angVel = discNormal * (ds.discRotationVel * spinTransferFactor)
                           + discX      * ds.discRollingVel
                           + discY      * ds.discPitchingVel;

            // Sync Rigidbody transform to the disc's landed position before going non-kinematic,
            // in case the Rigidbody parent and discTransform are different objects.
            discRigidbody.transform.position = discTransform != null
                ? discTransform.position
                : DiscFlightSimulator.ToUnitySpace(ds.discLocation) * sceneScale;

            discRigidbody.isKinematic           = false;
            discRigidbody.detectCollisions       = true;
            discRigidbody.collisionDetectionMode = CollisionDetectionMode.Continuous;
            discRigidbody.linearDamping          = landingLinearDrag;
            discRigidbody.angularDamping         = landingAngularDrag;
            discRigidbody.linearVelocity         = vel;
            discRigidbody.angularVelocity        = angVel;
        }

        // ---------------------------------------------------------------
        // KF trail (existing — unchanged)
        // ---------------------------------------------------------------
        public void ClearTrail()
        {
            _trailPoints.Clear();
            if (trajectoryLine != null)
                trajectoryLine.positionCount = 0;
        }

        private void AppendTrailPoint(Vector3 worldPos, float speedKph)
        {
            _trailPoints.Add(worldPos);
            if (_trailPoints.Count > maxTrailPoints)
                _trailPoints.RemoveAt(0);

            if (trajectoryLine == null) return;

            trajectoryLine.positionCount = _trailPoints.Count;
            trajectoryLine.SetPositions(_trailPoints.ToArray());

            if (trailColorBySpeed != null && _trailPoints.Count > 1)
            {
                float t = Mathf.Clamp01(speedKph / maxSpeedKph);
                trajectoryLine.startColor = trailColorBySpeed.Evaluate(t);
                trajectoryLine.endColor   = trailColorBySpeed.Evaluate(t);
            }
        }

        // ---------------------------------------------------------------
        // Coordinate frame remap (existing — unchanged)
        // ---------------------------------------------------------------
        private Vector3 RemapPosition(Vector3 dvd)
        {
            if (!remapCoordinateFrame) return dvd;
            return new Vector3(dvd.y, dvd.z, dvd.x);
        }

        private Quaternion RemapRotation(Quaternion dvd)
        {
            if (!remapCoordinateFrame) return dvd;
            Vector3 e = dvd.eulerAngles;
            return Quaternion.Euler(e.y, e.z, e.x);
        }

        // ---------------------------------------------------------------
        // HUD (existing — unchanged)
        // ---------------------------------------------------------------
        private void UpdateHUD(KFState state, string prefix)
        {
            if (hudLabel == null) return;

            string label = prefix != null ? $"[{prefix}]\n" : "";
            label += $"Speed:  {state.LinearSpeedKph:F1} kph\n" +
                     $"Hyzer:  {Mathf.Rad2Deg * (float)state.angHyzer.pos:F1} deg\n" +
                     $"Pitch:  {Mathf.Rad2Deg * (float)state.angPitch.pos:F1} deg\n" +
                     $"Spin:   {(float)state.angSpin.vel:F1} rad/s\n" +
                     $"Wobble: {state.wobbleMag:P0}\n" +
                     $"Disc:   {state.discIndex}";

            hudLabel.text = label;
        }

        private void ShowThrowSummary(DiscInitState s)
        {
            if (hudLabel == null) return;

            float spinRadS = s.angularVelocityRad.z;
            hudLabel.text =
                $"--- THROW COMPLETE ---\n" +
                $"Speed:   {s.linearVelocityMs.magnitude * 3.6f:F1} kph\n" +
                $"Hyzer:   {Mathf.Rad2Deg * s.angularPositionRad.x:F1} deg\n" +
                $"Pitch:   {Mathf.Rad2Deg * s.angularPositionRad.y:F1} deg\n" +
                $"Spin:    {Mathf.Abs(spinRadS):F1} rad/s {(spinRadS > 0 ? "CCW" : "CW")}\n" +
                $"Wobble:  {s.wobble:P0}\n" +
                $"Disc:    {s.discMold}";
        }
    }
}
