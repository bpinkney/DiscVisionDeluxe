using Unity.Mathematics;
using static Unity.Mathematics.math;
using UnityEngine;
using float3 = Unity.Mathematics.float3;
using float3x3 = Unity.Mathematics.float3x3;

// ============================================================================
//  DiscFlightSimulator.cs
//  Port of DfisX.cpp — main simulation loop, throw initialisation, and
//  per-frame stepping.
//
//  Coordinate system: Z-up (matches C++).
//  Gravity vector in Dpropagate is (0, 0, -9.8*mass).
//  When handing position/orientation to Unity Transforms (Y-up), use the
//  helper ToUnitySpace() below to convert.
//
//  Porting notes:
//  - consume_Dcollision() is the messiest function in the C++ codebase.
//    The active (non-commented-out) path is the impulse-torque branch
//    (lines 320-376 of DfisX.cpp). The large overwrite_states block
//    (lines 59-308) is guarded by `overwrite_states_ignore_forces_torques = false`
//    and is entirely inert. It is omitted here. If collisions are re-enabled
//    in a future session, that block should be ported at that time.
//  - The disc mold selection switch in new_throw() (C++ lines 499-619)
//    contained many dev-only overrides and commented-out random selectors.
//    Replaced with a clean DiscModelLibrary.FindByDiscIndex() lookup.
//  - simulate_throw() (batch/offline mode) runs the full flight synchronously.
//    StepSimulation() (realtime mode) steps one dt per call, matching the
//    Unreal Tick() sub-stepping pattern from DiscThrow.cpp.
//  - SIM_DT_S = 0.001 (1kHz) matches the C++ constant.
//  - Max step guard: 10000 steps (matches C++ — ~10s at 1kHz).
//  - finish_throw() computes distance/speed stats and fires OnThrowFinished.
// ============================================================================

namespace DfisX
{
    public class DiscFlightSimulator
    {
        // --------------------------------------------------------------------
        // Constants
        // --------------------------------------------------------------------

        /// <summary>Default simulation timestep (1kHz). Matches SIM_DT_S in C++.</summary>
        public const float SIM_DT_S = 0.001f;

        /// <summary>Safety cutoff — abort if sim exceeds this many steps (~30s at 1kHz).</summary>
        public const int MAX_STEPS = 30000;

        // --------------------------------------------------------------------
        // State
        // --------------------------------------------------------------------

        public ThrowContainer Container { get; private set; }

        /// <summary>True once the disc has landed or the step limit was reached.</summary>
        public bool IsFinished => Container?.currentDiscState.simState == SimState.Stopped;

        // --------------------------------------------------------------------
        // Events
        // --------------------------------------------------------------------

        /// <summary>
        /// Fired when the throw ends (disc hits ground or step limit).
        /// Payload: final DiscState at landing, flight statistics.
        /// </summary>
        public System.Action<DiscState, FlightStats> OnThrowFinished;

        // --------------------------------------------------------------------
        // Dependencies
        // --------------------------------------------------------------------

        readonly DiscModelLibrary _library;

        public DiscFlightSimulator(DiscModelLibrary library)
        {
            _library = library;
        }

        // --------------------------------------------------------------------
        // NewThrow  (mirrors new_throw in DfisX.cpp)
        // --------------------------------------------------------------------

        /// <summary>
        /// Initialise a new throw from KF output parameters.
        /// Disposes any previous ThrowContainer.
        /// </summary>
        public void NewThrow(ThrowParameters p, DiscEnvironment env, AeroDebugSettings dbg = default)
        {
            Container?.Dispose();
            Container = new ThrowContainer();

            ref DiscState ds = ref Container.currentDiscState;

            // ---- Disc state ----
            ds.discLocation      = p.position;
            ds.discVelocity      = p.velocity;
            ds.discOrientZVect   = p.DiscNormal();
            ds.discRotationVel   = p.spinRate;
            ds.discRotation      = 0f;
            ds.simState          = SimState.Started;
            ds.forcesState       = default;

            // Seed X/Y orientation vectors to something orthonormal to Z.
            // Daero will recompute them on the first step.
            float3 arbitraryX    = abs(ds.discOrientZVect.x) < 0.9f
                                   ? new float3(1f, 0f, 0f)
                                   : new float3(0f, 1f, 0f);
            ds.discOrientYVect   = normalize(cross(ds.discOrientZVect, arbitraryX));
            ds.discOrientXVect   = normalize(cross(ds.discOrientYVect, ds.discOrientZVect));

            Container.previousDiscState = ds;

            // ---- Environment ----
            Container.discEnvironment = env;

            // ---- Aero debug ----
            Container.aeroDebug = dbg.cdEdge > 0f ? dbg : AeroDebugSettings.Default;

            // ---- Disc model ----
            // Prefer directModel (set by UI when user picks from dropdown) over
            // the enum→library lookup, which requires the library fields to be populated.
            DiscModel so = p.directModel ?? _library.FindByDiscIndex(p.discIndex);
            Container.discObject = so != null
                ? so.ToBlittable()
                : DiscModelPresets.Destroyer().ToBlittable();

            // ---- Statistics origin ----
            Container.discStartLocation = p.position;

            // Run one priming step so Daero populates unit vectors before
            // the first external call to StepSimulation() — mirrors the
            // step_simulation() call at the end of new_throw() in C++.
            StepSimulation(SIM_DT_S);
        }

        // --------------------------------------------------------------------
        // StepSimulation  (mirrors step_simulation in DfisX.cpp)
        // Call from MonoBehaviour.Update() with sub-stepping for large dt.
        // --------------------------------------------------------------------

        /// <summary>
        /// Advance simulation by one dt.
        /// Returns false if the throw has already finished.
        /// </summary>
        public bool StepSimulation(float dt)
        {
            if (Container == null || IsFinished) return false;

            ConsumeDcollision(dt);
            Daero.StepDaero(Container, dt);
            Dgyro.StepDgyro(Container, dt);
            Dpropagate.Propagate(Container, dt);

            // Ground detection (Z-up: z <= 0 means ground)
            if (Container.currentDiscState.discLocation.z <= 0f)
            {
                FinishThrow(dt);
                return false;
            }

            // Step-count safety cutoff
            if (Container.currentDiscState.forcesState.stepCount > MAX_STEPS)
            {
                Debug.LogWarning("[DfisX] Throw aborted: exceeded MAX_STEPS.");
                Container.currentDiscState.simState = SimState.Stopped;
                return false;
            }

            return true;
        }

        /// <summary>
        /// Helper for MonoBehaviour.Update(): sub-steps to keep sim at SIM_DT_S
        /// regardless of Unity frame rate. Matches the Tick() sub-stepping in
        /// DiscThrow.cpp.
        /// </summary>
        public void StepForUnityFrame(float unityDeltaTime)
        {
            if (IsFinished) return;

            if (unityDeltaTime > SIM_DT_S)
            {
                int substeps = Mathf.CeilToInt(unityDeltaTime / SIM_DT_S);
                float subDt  = unityDeltaTime / substeps;
                for (int i = 0; i < substeps && !IsFinished; i++)
                    StepSimulation(subDt);
            }
            else
            {
                StepSimulation(unityDeltaTime);
            }
        }

        // --------------------------------------------------------------------
        // SimulateThrow  (mirrors simulate_throw — offline/batch mode)
        // --------------------------------------------------------------------

        /// <summary>
        /// Run the full throw to completion synchronously.
        /// Use for trajectory preview, AI training, or landing-zone prediction.
        /// </summary>
        public FlightStats SimulateThrow(float dt = SIM_DT_S)
        {
            while (!IsFinished)
                StepSimulation(dt);

            return BuildStats(dt);
        }

        // --------------------------------------------------------------------
        // ConsumeDcollision  (mirrors consume_Dcollision in DfisX.cpp)
        // Only the active (impulse-torque) path is ported. The inert
        // overwrite_states block is omitted — see porting notes.
        // --------------------------------------------------------------------

        void ConsumeDcollision(float dt)
        {
            const float CLOSE_TO_ZERO = 0.000001f;

            // Clear collision outputs each step
            ref ForcesState f = ref Container.currentDiscState.forcesState;
            f.collisionForce       = float3.zero;
            f.collisionTorqueXYZ   = float3.zero;

            ref CollisionInput ci = ref Container.collisionInput;

            int framesNeeded = (int)math.round(ci.deltaTimeS / math.max(dt, CLOSE_TO_ZERO));
            framesNeeded     = math.clamp(framesNeeded, 0, 10);

            if (ci.consumedInput < framesNeeded && ci.deltaTimeS > CLOSE_TO_ZERO)
            {
                // Active (non-overwrite) branch: apply impulse forces and torques
                f.collisionForce     = new float3(
                    ci.linForceFromImpulsesN.x,
                    ci.linForceFromImpulsesN.y,
                    ci.linForceFromImpulsesN.z);

                f.collisionTorqueXYZ = new float3(
                    ci.angTorqueFromImpulsesNm.x,
                    ci.angTorqueFromImpulsesNm.y,
                    ci.angTorqueFromImpulsesNm.z);

                ci.consumedInput++;
            }
        }

        // --------------------------------------------------------------------
        // FinishThrow  (mirrors finish_throw in DfisX.cpp)
        // --------------------------------------------------------------------

        void FinishThrow(float dt)
        {
            Container.currentDiscState.simState  = SimState.Stopped;
            Container.discFinishLocation          = Container.currentDiscState.discLocation;

            FlightStats stats = BuildStats(dt);
            OnThrowFinished?.Invoke(Container.currentDiscState, stats);
        }

        FlightStats BuildStats(float dt)
        {
            if (Container.discStateCount < 2)
                return default;

            float3 startLoc   = Container.discStateArray[0].discLocation;
            float3 finishLoc  = Container.discStateArray[Container.discStateCount - 1].discLocation;
            float3 startVel   = Container.discStateArray[0].discVelocity;

            return new FlightStats
            {
                timeAloftS          = Container.discStateCount * dt,
                distanceM           = length(finishLoc - startLoc),
                maxSpeedMps         = length(startVel),
                stepCount           = Container.currentDiscState.forcesState.stepCount,
                startLocation       = startLoc,
                landingLocation     = finishLoc
            };
        }

        // --------------------------------------------------------------------
        // Coordinate conversion  (Z-up sim → Unity Y-up world)
        // --------------------------------------------------------------------

        /// <summary>
        /// Convert a DfisX Z-up position to Unity Y-up world space.
        /// Swaps Y and Z, scales metres → Unity units (1:1 by default).
        /// </summary>
        public static Vector3 ToUnitySpace(float3 pos)
            => new Vector3(pos.x, pos.z, pos.y);

        /// <summary>
        /// Convert a DfisX disc orientation (Z-up disc normal) to a Unity
        /// Quaternion suitable for assigning to a Transform.
        /// </summary>
        public static Quaternion ToUnityRotation(float3 discOrientZ, float3 discOrientX)
        {
            // Remap axes: DfisX (x,y,z) → Unity (x,z,y)
            Vector3 up      = new Vector3(discOrientZ.x, discOrientZ.z, discOrientZ.y);
            Vector3 forward = new Vector3(discOrientX.x, discOrientX.z, discOrientX.y);
            if (up == Vector3.zero || forward == Vector3.zero)
                return Quaternion.identity;
            return Quaternion.LookRotation(forward, up);
        }
    }

    // ------------------------------------------------------------------------
    // FlightStats — mirrors the console output of finish_throw()
    // ------------------------------------------------------------------------

    public struct FlightStats
    {
        public float  timeAloftS;
        public float  distanceM;
        public float  maxSpeedMps;
        public int    stepCount;
        public float3 startLocation;
        public float3 landingLocation;

        public override string ToString() =>
            $"Time: {timeAloftS:F2}s  Distance: {distanceM:F1}m ({distanceM * 3.28f:F0}ft)" +
            $"  MaxSpeed: {maxSpeedMps:F1}m/s ({maxSpeedMps * 2.237f:F0}mph)" +
            $"  Steps: {stepCount}";
    }
}
