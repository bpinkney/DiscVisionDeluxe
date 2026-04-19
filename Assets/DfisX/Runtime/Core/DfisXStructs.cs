using Unity.Mathematics;
using Unity.Collections;

// Unity.Mathematics float3 is used instead of UnityEngine.Vector3 because:
//   - it is blittable (no managed header)
//   - Burst understands it natively and auto-vectorises operations on it
//   - math.* functions (cross, dot, normalize, lengthsq) are SIMD-friendly

namespace DfisX
{
    // -------------------------------------------------------------------------
    // Simulation State
    // -------------------------------------------------------------------------

    public enum SimState : int
    {
        Stopped              = 0,
        Started              = 1,
        FlyingHighSpeedTurn  = 2,
        FlyingTurn           = 3,
        Flying               = 4,
        FlyingFade           = 5,
        Skipping             = 6,
        TreeHit              = 7,
        Rolling              = 8,
        Sliding              = 9
    }

    public enum GustFactor : int
    {
        ZeroDeadDiddly        = 0,
        OneDullDraft          = 1,
        TwoCalmChinook        = 2,
        ThreeBrusqueBreeze    = 3,
        FourRobustGust        = 4,
        FiveZealousZephyr     = 5,
        SixGalledGale         = 6,
        SevenFuriousFlurry    = 7,
        EightTerribleTempest  = 8,
        NinePsychoticCyclone  = 9,
        TenHomicidalHurricane = 10
    }

    // -------------------------------------------------------------------------
    // DiscEnvironment — blittable
    // -------------------------------------------------------------------------

    public struct DiscEnvironment
    {
        /// <summary>Static wind velocity in world frame (m/s)</summary>
        public float3 windVectorXYZ;

        public GustFactor gustFactor;

        /// <summary>
        /// Air density (kg/m3). ISA sea-level = 1.225.
        /// Decrease for altitude — roughly 1.007 at 500m, 0.905 at 2000m.
        /// </summary>
        public float airDensity;

        public static DiscEnvironment Default => new DiscEnvironment
        {
            windVectorXYZ = float3.zero,
            gustFactor    = GustFactor.ZeroDeadDiddly,
            airDensity    = 1.225f
        };
    }

    // -------------------------------------------------------------------------
    // DiscModelData — blittable mirror of DiscModel ScriptableObject.
    // Stored in NativeArray so jobs can read disc parameters without
    // touching managed memory.
    // FromScriptableObject() lives in DiscModel.cs (DfisX.Unity assembly)
    // as DiscModel.ToBlittable() to avoid a dependency on UnityEngine here.
    // -------------------------------------------------------------------------

    public struct DiscModelData
    {
        public float mass;
        public float radius;
        public float rimWidth;
        public float thickness;
        public float rimDepth;
        public float rimCamberHeight;
        public float domeHeight;

        /// <summary>0 = Flat, 1 = Concave, 2 = Convex</summary>
        public int rimCamberShapeInt;

        // ---- Computed convenience properties ----

        /// <summary>Blunt edge height = thickness - rimCamberHeight - domeHeight, >= 0</summary>
        public float EdgeHeight => math.max(0f, thickness - rimCamberHeight - domeHeight);

        /// <summary>Moment of inertia about spin (Z) axis: Iz = 0.5 m r^2</summary>
        public float Iz => 0.5f * mass * radius * radius;

        /// <summary>Moment of inertia about pitch/roll axes: Ix = Iy = 0.25 m r^2</summary>
        public float Ixy => 0.25f * mass * radius * radius;
    }

    // -------------------------------------------------------------------------
    // ForcesState — blittable
    // Per-step aerodynamic calculation results embedded inside DiscState.
    // -------------------------------------------------------------------------

    public struct ForcesState
    {
        // --- gyroscopic ---
        public float3 gyroOrientationDelta;

        // --- net forces / torques ---
        public float3 netForce;
        public float  netTorqueX;
        public float  netTorqueY;
        public float  netTorqueZ;

        // --- aero components ---
        public float3 aeroForce;
        public float  gyroTorqueX;
        public float  gyroTorqueY;
        public float  aeroTorqueX;
        public float  aeroTorqueY;
        public float  aeroTorqueZ;

        // --- collision ---
        public float3 collisionForce;       // world frame (N)
        public float3 collisionTorqueXYZ;   // body axes  (Nm)

        public int stepCount;

        // --- gust ---
        public float  gustTimeS;
        public float3 gustVectorXYZ;

        // --- aero intermediates (written by Daero, read by Dgyro/Dpropagate) ---
        public float3 discLiftUnitVector;
        public float3 discVelocityUnitVector;
        public float3 liftForceVector;
        public float3 dragForceVector;

        public float aoar;               // angle of attack (radians)
        public float velocityMagnitude;
        public float v2;                 // velocity squared

        // --- pitching moments ---
        public float liftInducedPitchingMoment;

        public float linDragForcePlateN;
        public float linDragForceEdgeN;
        public float linDragForceCavityEdgeN;
        public float linDragForceFrontRimCamberN;
        public float linDragForceBackRimCamberN;
        public float linDragForceFrontDomeCamberN;
        public float linDragForceBackDomeCamberN;
        public float linDragForceSkinN;

        public float liftForceCavityEdgeN;
        public float liftForceCamberN;

        public float rotTorquePlateOffsetNm;
        public float rotTorqueRimCamberOffsetNm;
        public float rotTorqueCavityEdgeOffsetNm;
        public float rotTorqueCamberOffsetNm;

        public float rotDragTorqueXNm;
        public float rotDragTorqueYNm;
        public float rotDragTorqueZNm;
    }

    // -------------------------------------------------------------------------
    // DiscState — blittable
    // Complete state of one disc at one point in time.
    // -------------------------------------------------------------------------

    public struct DiscState
    {
        // linear
        public float3 discLocation;
        public float3 discVelocity;
        public float3 discAcceleration;

        // orientation — three orthonormal unit vectors
        /// <summary>Normal pointing out of the top of the disc</summary>
        public float3 discOrientZVect;
        /// <summary>Points forward along the airspeed direction (in disc plane)</summary>
        public float3 discOrientXVect;
        /// <summary>Points right in the disc plane, orthogonal to X and Z</summary>
        public float3 discOrientYVect;

        // angular
        public float discPitchingVel;
        public float discPitchingAccel;
        public float discRollingVel;
        public float discRollingAccel;
        public float discRotation;      // spin position (radians)
        public float discRotationVel;   // spin rate     (rad/s)
        public float discRotationAccel;

        public SimState    simState;
        public ForcesState forcesState;

        /// <summary>dt used during the last propagation step (needed by collision model)</summary>
        public float lastDt;

        // ---- Factory ----

        public static DiscState Default => new DiscState
        {
            discOrientZVect = new float3(0f, 1f, 0f),
            discOrientXVect = new float3(1f, 0f, 0f),
            discOrientYVect = new float3(0f, 0f, 1f),
            simState        = SimState.Stopped
        };
    }

    // -------------------------------------------------------------------------
    // CollisionInput — blittable
    // -------------------------------------------------------------------------

    public struct CollisionInput
    {
        public float3 linPosM;
        public float3 linVelMps;
        public float3 linForceFromImpulsesN;
        public float3 linForceFromDeltaVelN;

        public float3 discRotation;

        public float3 angVelRadps;
        public float3 angVelDeltaRadps;
        public float3 angTorqueFromDeltaVelNm;
        public float3 angVelFromImpulsesNm;
        public float3 angTorqueFromImpulsesNm;

        public float deltaTimeS;

        public int consumedInput;

        public static CollisionInput Invalid => new CollisionInput { consumedInput = 255 };
    }

    // -------------------------------------------------------------------------
    // FrictionInput — blittable
    // -------------------------------------------------------------------------

    public struct FrictionInput
    {
        public float3 linForceXYZ;
        public float3 angTorqueXYZ;
    }

    // -------------------------------------------------------------------------
    // AeroDebugSettings — blittable
    // -------------------------------------------------------------------------

    public struct AeroDebugSettings
    {
        public float cdEdge;
        public float clCavity;
        public float clCamber;
        public float cavityEdgeExposedAreaFactor;
        public float pitchingMomentCavityLiftOffset;
        public float pitchingMomentCamberLiftOffset;

        public static AeroDebugSettings Default => new AeroDebugSettings
        {
            cdEdge                         = 0.6f,
            clCavity                       = 45.0f,
            clCamber                       = 1.0f,
            cavityEdgeExposedAreaFactor    = 1.0f,
            pitchingMomentCavityLiftOffset = 0.042f,
            pitchingMomentCamberLiftOffset = 0.15f
        };
    }

    // -------------------------------------------------------------------------
    // ThrowContainer — managed wrapper (one per active disc flight)
    // -------------------------------------------------------------------------

    public class ThrowContainer : System.IDisposable
    {
        public DiscState       currentDiscState;
        public DiscState       previousDiscState;
        public DiscModelData   discObject;
        public DiscEnvironment discEnvironment = DiscEnvironment.Default;
        public CollisionInput  collisionInput  = CollisionInput.Invalid;
        public FrictionInput   frictionInput;
        public AeroDebugSettings aeroDebug     = AeroDebugSettings.Default;

        /// <summary>
        /// Per-throw Gaussian noise state for gust simulation.
        /// Replaces static locals in C++ gaussrand() — safe for multiple simultaneous throws.
        /// </summary>
        public GaussState gaussState = new GaussState();

        public NativeArray<DiscState> discStateArray;
        public int discStateCount;

        public float3 discStartLocation;
        public float3 discFinishLocation;
        public float  discCumulativeRoll;

        // Throw-input snapshot for BuildStats (avoids ThrowParameters dependency)
        public float  throwSpinRateRadS;
        public float  throwHyzerRad;
        public float  throwPitchRad;
        public string throwDiscName;

        public ThrowContainer(int initialCapacity = 5000)
        {
            discStateArray = new NativeArray<DiscState>(
                initialCapacity, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);
            discStateCount = 0;

            currentDiscState  = DiscState.Default;
            previousDiscState = DiscState.Default;
        }

        public void RecordState()
        {
            if (discStateCount >= discStateArray.Length)
                EnsureCapacity(discStateArray.Length * 2);

            discStateArray[discStateCount++] = currentDiscState;
        }

        public void EnsureCapacity(int required)
        {
            if (required <= discStateArray.Length) return;
            var next = new NativeArray<DiscState>(
                required, Allocator.Persistent,
                NativeArrayOptions.UninitializedMemory);
            NativeArray<DiscState>.Copy(discStateArray, next, discStateCount);
            discStateArray.Dispose();
            discStateArray = next;
        }

        public void Dispose()
        {
            if (discStateArray.IsCreated) discStateArray.Dispose();
        }
    }
}
