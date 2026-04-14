using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe
{
    /// <summary>
    /// Inspector-driven disc throw debugger.
    /// Lets you manually set throw parameters, hit Launch, and watch the disc fly.
    /// Bypasses the KF pipeline — directly calls DiscVisualizer.LaunchDfisX().
    ///
    /// Usage:
    ///   1. Add this component to any GameObject in your debug scene.
    ///   2. Assign the discVisualizer reference in the Inspector.
    ///   3. Tweak throw parameters in the Inspector.
    ///   4. Right-click this component → Launch (in Editor), or press the
    ///      on-screen button in Play mode.
    /// </summary>
    public class DiscThrowDebugger : MonoBehaviour
    {
        // ---------------------------------------------------------------
        // Inspector — References
        // ---------------------------------------------------------------
        [Header("References")]
        [Tooltip("The DiscVisualizer that will run and render the flight.")]
        public DiscVisualizer discVisualizer;

        // ---------------------------------------------------------------
        // Inspector — Disc Selection
        // ---------------------------------------------------------------
        [Header("Disc")]
        [Tooltip("Which disc mold to throw.")]
        public DiscLayoutIndex discIndex = DiscLayoutIndex.DRIVER;

        // ---------------------------------------------------------------
        // Inspector — Throw Parameters
        // ---------------------------------------------------------------
        [Header("Throw Parameters")]
        [Tooltip("Release position in DfisX Z-up world frame (metres). X=forward, Y=right, Z=up.")]
        public Vector3 releasePositionM = new Vector3(0f, 0f, 1.5f);

        [Tooltip("Speed at release (m/s). Typical range: 10-30.")]
        [Range(5f, 40f)]
        public float speedMps = 22f;

        [Tooltip("Horizontal heading in degrees. 0 = straight along X axis.")]
        [Range(-180f, 180f)]
        public float headingDeg = 0f;

        [Tooltip("Vertical launch angle in degrees. 0 = flat, positive = upward.")]
        [Range(-30f, 45f)]
        public float launchAngleDeg = 3f;

        [Tooltip("Hyzer angle in degrees. Positive = disc tilted right for RHBH (anhyzer).")]
        [Range(-45f, 45f)]
        public float hyzerDeg = 0f;

        [Tooltip("Nose/pitch angle in degrees. Positive = nose up.")]
        [Range(-30f, 30f)]
        public float pitchDeg = 0f;

        [Tooltip("Spin rate in RPM. Negative = RHBH (clockwise from above). Typical: -600 to -900.")]
        [Range(-1200f, 1200f)]
        public float spinRpm = -700f;

        [Tooltip("Wobble magnitude [0, 1]. 0 = no wobble.")]
        [Range(0f, 1f)]
        public float wobble = 0f;

        // ---------------------------------------------------------------
        // Inspector — Environment
        // ---------------------------------------------------------------
        [Header("Environment")]
        [Tooltip("Wind in DfisX Z-up frame (m/s). X=forward, Y=right, Z=up.")]
        public Vector3 windVectorZUp = Vector3.zero;

        [Tooltip("Air density (kg/m3). ISA sea-level = 1.225.")]
        public float airDensity = 1.225f;

        // ---------------------------------------------------------------
        // Inspector — Trail
        // ---------------------------------------------------------------
        [Header("Trail")]
        [Tooltip("When true, each throw adds a new arc. When false, clears previous arc.")]
        public bool keepAllThrows = false;

        // ---------------------------------------------------------------
        // Private
        // ---------------------------------------------------------------
        private DiscInitState _lastParams;

        // ---------------------------------------------------------------
        // Launch — callable from Inspector via right-click context menu,
        // and from the runtime OnGUI button.
        // ---------------------------------------------------------------

        /// <summary>
        /// Build a DiscInitState from the current Inspector parameters and launch.
        /// Right-click this component in the Inspector → Launch to call in Editor.
        /// </summary>
        [ContextMenu("Launch")]
        public void Launch()
        {
            if (discVisualizer == null)
            {
                Debug.LogError("[DiscThrowDebugger] discVisualizer is not assigned.");
                return;
            }

            // Sync keepAllThrows to DiscVisualizer
            discVisualizer.keepAllThrows = keepAllThrows;

            _lastParams = BuildDiscInitState();
            discVisualizer.LaunchDfisX(_lastParams);

            Debug.Log($"[DiscThrowDebugger] Launched — {_lastParams.linearVelocityMs.magnitude * 3.6f:F1} kph, " +
                      $"hyzer {hyzerDeg:F1}deg, pitch {pitchDeg:F1}deg, spin {spinRpm:F0}rpm, disc {discIndex}");
        }

        /// <summary>
        /// Re-throw with identical parameters to the last launch.
        /// Useful for comparing physics changes or aero tuning.
        /// </summary>
        [ContextMenu("Re-throw (same params)")]
        public void Rethrow()
        {
            if (_lastParams == null)
            {
                Debug.LogWarning("[DiscThrowDebugger] No previous throw to repeat — calling Launch() instead.");
                Launch();
                return;
            }

            discVisualizer.keepAllThrows = keepAllThrows;
            discVisualizer.LaunchDfisX(_lastParams);
            Debug.Log("[DiscThrowDebugger] Re-threw with same parameters.");
        }

        /// <summary>Clear all DfisX trajectory arcs in the scene.</summary>
        [ContextMenu("Clear All Trails")]
        public void ClearAllTrails()
        {
            if (discVisualizer != null)
                discVisualizer.ClearAllDfisxTrails();
        }

        // ---------------------------------------------------------------
        // Runtime UI — simple OnGUI buttons visible in Play mode Game view.
        // No Canvas or UI setup needed.
        // ---------------------------------------------------------------
        void OnGUI()
        {
            float btnW = 180f;
            float btnH = 36f;
            float pad  = 10f;
            float x    = pad;
            float y    = pad;

            GUI.skin.button.fontSize  = 14;
            GUI.skin.label.fontSize   = 12;
            GUI.skin.label.normal.textColor = Color.white;

            if (GUI.Button(new Rect(x, y, btnW, btnH), "Launch"))
                Launch();

            y += btnH + pad;
            if (GUI.Button(new Rect(x, y, btnW, btnH), "Re-throw"))
                Rethrow();

            y += btnH + pad;
            if (GUI.Button(new Rect(x, y, btnW, btnH), "Clear Trails"))
                ClearAllTrails();

            // Live readout of key params below the buttons
            y += btnH + pad;
            GUI.Label(new Rect(x, y, 300f, 22f), $"Speed: {speedMps:F1} m/s ({speedMps * 3.6f:F1} kph)");
            y += 20f;
            GUI.Label(new Rect(x, y, 300f, 22f), $"Heading: {headingDeg:F1} deg  Launch: {launchAngleDeg:F1} deg");
            y += 20f;
            GUI.Label(new Rect(x, y, 300f, 22f), $"Hyzer: {hyzerDeg:F1} deg  Pitch: {pitchDeg:F1} deg");
            y += 20f;
            GUI.Label(new Rect(x, y, 300f, 22f), $"Spin: {spinRpm:F0} RPM  Disc: {discIndex}");
            y += 20f;

            if (discVisualizer != null && discVisualizer.IsSimFlying)
            {
                GUI.Label(new Rect(x, y, 300f, 22f), ">> FLYING <<");
            }
            else if (discVisualizer != null && !discVisualizer.IsSimFlying
                     && discVisualizer.LastFlightStats.timeAloftS > 0f)
            {
                var s = discVisualizer.LastFlightStats;
                y += 20f;
                GUI.Label(new Rect(x, y, 300f, 22f), $"Landed: {s.distanceM:F1}m ({s.distanceM * 3.28f:F0}ft)");
                y += 20f;
                GUI.Label(new Rect(x, y, 300f, 22f), $"Time: {s.timeAloftS:F2}s  Steps: {s.stepCount}");
            }
        }

        // ---------------------------------------------------------------
        // Build DiscInitState from Inspector params
        // ---------------------------------------------------------------
        private DiscInitState BuildDiscInitState()
        {
            // Convert heading + launch angle + speed to velocity vector (Z-up frame)
            float headingRad = Mathf.Deg2Rad * headingDeg;
            float launchRad  = Mathf.Deg2Rad * launchAngleDeg;

            // X=forward, Y=right, Z=up
            float vHoriz = speedMps * Mathf.Cos(launchRad);
            Vector3 velocity = new Vector3(
                vHoriz * Mathf.Cos(headingRad),   // X forward component
                vHoriz * Mathf.Sin(headingRad),   // Y right component
                speedMps * Mathf.Sin(launchRad)); // Z up component

            // Spin: RPM → rad/s
            float spinRadS = spinRpm * Mathf.PI * 2f / 60f;

            return new DiscInitState
            {
                linearPositionM    = releasePositionM,
                linearVelocityMs   = velocity,
                angularPositionRad = new Vector3(
                    Mathf.Deg2Rad * hyzerDeg,
                    Mathf.Deg2Rad * pitchDeg,
                    0f),
                angularVelocityRad = new Vector3(0f, 0f, spinRadS),
                wobble             = wobble,
                discMold           = (DiscIndex)(int)discIndex
            };
        }
    }
}
