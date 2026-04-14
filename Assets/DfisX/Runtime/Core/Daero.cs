using Unity.Mathematics;
using static Unity.Mathematics.math;
using float3 = Unity.Mathematics.float3;
using float3x3 = Unity.Mathematics.float3x3;

// ============================================================================
//  Daero.cs
//  Port of Daero.cpp — aerodynamic forces and torques for disc flight.
//
//  Porting notes:
//  - Eigen::Vector3d → float3  (Unity.Mathematics)
//  - double          → float   (acceptable precision at 1kHz sim dt)
//  - static function-local state in gaussrand() → Gauss state on ThrowContainer
//    (the C++ statics broke multithreading; per-container state fixes that)
//  - #define macros → const float / static methods
//  - strcmp(rim_camber, "Flat") → switch on DiscModelData.rimCamberShapeInt
//  - BOUND_VARIABLE(v,lo,hi)   → math.clamp(v, lo, hi)
//  - LP_FILT(var,new,N)        → LpFilt() helper below
//  - ALLOW_AERO_UNREAL_DEBUG   → removed; use AeroDebugSettings directly
// ============================================================================

namespace DfisX
{
    public static class Daero
    {
        // --------------------------------------------------------------------
        // Model constants  (mirrors of the #define block in Daero.cpp)
        // --------------------------------------------------------------------

        const float Cm_BASE  = 0.05f;   // base z-axis rotational parasitic skin drag coeff
        const float Cd_PLATE = 1.17f;   // base form drag coeff for 'plate' section
        const float Cd_EDGE  = 0.6f;    // base form drag coeff for disc edge
        const float Cd_SKIN  = 0.01f;   // base linear parasitic skin drag coeff

        const float Cl_CAVITY = 45.0f;  // base lift coeff for cavity Bernoulli effect
        const float Cl_CAMBER = 1.0f;   // base lift coeff for dome camber Bernoulli effect

        const float CAVITY_EDGE_NORM_ROT_SPEED = 0.0f;
        const float CAVITY_EDGE_LIFT_EXP       = 1.0f;
        const float CAVITY_EDGE_LIFT_GAIN      = 1.0f;

        const float CAVITY_EDGE_EXPOSED_AREA_FACTOR = 1.0f;
        const float A_EDGE_EFFECTIVE_GAIN           = 0.25f;

        const float PITCHING_MOMENT_FORM_DRAG_PLATE_OFFSET = 0.0f;
        const float PITCHING_MOMENT_CAVITY_LIFT_OFFSET     = 0.042f;
        const float PITCHING_MOMENT_CAMBER_LIFT_OFFSET     = 0.15f;

        const float RIM_CAMBER_EXPOSURE = 1.0f;

        // SimState thresholds (AOA in radians)
        const float HS_TURN_CONST = -0.05f;
        const float TURN_CONST    =  0.05f;
        const float FADE_CONST    =  0.15f;

        const float CLOSE_TO_ZERO = 0.000001f;
        const float GRAV          = 9.80665f;

        // --------------------------------------------------------------------
        // Utility helpers
        // --------------------------------------------------------------------

        static float AngleBetweenVectors(float3 a, float3 b)
        {
            return atan2(length(cross(a, b)), dot(a, b));
        }

        static int Signum(float x) => x > 0f ? 1 : (x < 0f ? -1 : 0);

        /// <summary>
        /// First-order IIR low-pass filter.
        /// Mirrors LP_FILT(var, new_val, N): var = (var*N + new_val) / (N+1)
        /// </summary>
        static float LpFilt(float current, float newVal, float n)
            => (current * n + newVal) / (n + 1f);

        // --------------------------------------------------------------------
        // Gaussian noise  (replaces static-local gaussrand() in Daero.cpp)
        // State is stored per-ThrowContainer so multiple simultaneous throws
        // don't share state (was a threading hazard in C++).
        // --------------------------------------------------------------------

        /// <summary>
        /// Box-Muller gaussian noise generator, stateless API.
        /// Caller passes in ref GaussState and gets one sample each call.
        /// </summary>
        public static float Gaussrand(ref GaussState gs)
        {
            float x;
            if (gs.phase == 0)
            {
                float u1, u2;
                do
                {
                    // Unity.Mathematics.Random is not usable in jobs without
                    // per-thread seeding — use System.Random here for now.
                    // TODO: replace with Unity.Mathematics.Random for Burst.
                    u1 = (float)gs.rng.NextDouble();
                    u2 = (float)gs.rng.NextDouble();
                    gs.v1 = 2f * u1 - 1f;
                    gs.v2 = 2f * u2 - 1f;
                    gs.s  = gs.v1 * gs.v1 + gs.v2 * gs.v2;
                }
                while (gs.s >= 1f || gs.s == 0f);

                x = gs.v1 * sqrt(-2f * log(gs.s) / gs.s);
            }
            else
            {
                x = gs.v2 * sqrt(-2f * log(gs.s) / gs.s);
            }
            gs.phase = 1 - gs.phase;
            return x;
        }

        // --------------------------------------------------------------------
        // Gust computation  (mirrors Daero_compute_gusts)
        // --------------------------------------------------------------------

        static void ComputeGusts(ref ForcesState dForces,
                                 in  DiscEnvironment env,
                                 ref GaussState gs)
        {
            // 1st-order Butterworth LP filter coefficients  (0.2 Hz cutoff, 200 Hz sample)
            const float Fc = 0.2f;
            const float Fs = 200.0f;
            const float gustStddev   = 0.3f;
            float       N            = (1f / (Fc / (Fs / 2f))) * gustStddev;

            float3 rawNoise = new float3(
                Gaussrand(ref gs) * gustStddev,
                Gaussrand(ref gs) * gustStddev,
                Gaussrand(ref gs) * gustStddev * 0.25f);

            rawNoise = clamp(rawNoise, -1f, 1f);

            float gustAmp = (float)env.gustFactor;
            gustAmp = pow(gustAmp, 2.5f);
            rawNoise *= gustAmp;

            // LP filter each axis independently
            dForces.gustVectorXYZ.x = LpFilt(dForces.gustVectorXYZ.x, rawNoise.x, N);
            dForces.gustVectorXYZ.y = LpFilt(dForces.gustVectorXYZ.y, rawNoise.y, N);
            dForces.gustVectorXYZ.z = LpFilt(dForces.gustVectorXYZ.z, rawNoise.z, N);
        }

        // --------------------------------------------------------------------
        // Main entry point  (mirrors step_Daero)
        // --------------------------------------------------------------------

        /// <summary>
        /// Compute all aerodynamic forces and torques for one simulation step.
        /// Results are written into tc.currentDiscState.forcesState and
        /// tc.currentDiscState orientation vectors.
        ///
        /// Call order inside step_simulation():
        ///   1. ComputeGusts  (done inside here)
        ///   2. step_Daero    ← this method
        ///   3. step_Dgyro
        ///   4. propagate
        /// </summary>
        public static void StepDaero(ThrowContainer tc, float dt)
        {
            // Convenience aliases — mirrors the local refs in C++:
            //   Disc_State&  d_state  = throw_container->current_disc_state;
            //   Forces_State& d_forces = d_state.forces_state;
            //   Disc_Model&  d_object  = *throw_container->disc_object;

            ref DiscState      dState  = ref tc.currentDiscState;
            ref ForcesState    dForces = ref tc.currentDiscState.forcesState;
            ref DiscModelData  dObject = ref tc.discObject;
            ref DiscEnvironment env    = ref tc.discEnvironment;
            AeroDebugSettings  dbg    = tc.aeroDebug;

            // ---- Gusts ----
            ComputeGusts(ref dForces, in env, ref tc.gaussState);

            // ---- Effective airspeed (subtract wind + gusts) ----
            float3 discAirVelocity = dState.discVelocity
                                   - env.windVectorXYZ
                                   - dForces.gustVectorXYZ;

            // ---- Unit vectors ----
            float3 dOrientation = normalize(dState.discOrientZVect);    // disc normal

            float velLen = length(discAirVelocity);
            dForces.velocityMagnitude = velLen;
            dForces.v2                = velLen * velLen;

            if (velLen > CLOSE_TO_ZERO)
                dForces.discVelocityUnitVector = discAirVelocity / velLen;
            else
                dForces.discVelocityUnitVector = float3.zero;

            // Divide-by-zero guard: disc travelling perpendicular to airflow
            bool parallel = all(abs(dOrientation - dForces.discVelocityUnitVector) < CLOSE_TO_ZERO);
            if (!parallel)
            {
                dState.discOrientYVect        = normalize(cross(dForces.discVelocityUnitVector, dOrientation));
                dState.discOrientXVect        = normalize(cross(dState.discOrientYVect, dOrientation));
                dForces.discLiftUnitVector    = cross(dState.discOrientYVect, dForces.discVelocityUnitVector);
            }
            else
            {
                dState.discOrientXVect     = float3.zero;
                dState.discOrientYVect     = float3.zero;
                dForces.discLiftUnitVector = float3.zero;
            }

            // ---- Angle of Attack ----
            dForces.aoar = AngleBetweenVectors(dForces.discVelocityUnitVector, dOrientation) - PI * 0.5f;

            // ---- SimState ----
            if      (dForces.aoar < HS_TURN_CONST) dState.simState = SimState.FlyingHighSpeedTurn;
            else if (dForces.aoar < TURN_CONST)    dState.simState = SimState.FlyingTurn;
            else if (dForces.aoar < FADE_CONST)    dState.simState = SimState.Flying;
            else                                   dState.simState = SimState.FlyingFade;

            // ---- Precomputed geometry ----
            float r2         = dObject.radius * dObject.radius;
            float r5         = r2 * r2 * dObject.radius;
            float edgeHeight = dObject.EdgeHeight;

            // Edge force vector: projection of disc normal cross airspeed, along disc plane
            float3 edgeForceVector = normalize(cross(dOrientation, cross(dOrientation, dForces.discVelocityUnitVector)));

            float totalExposedSurfaceAreaM2 = 0f;

            // ----------------------------------------------------------------
            // ** Linear Form Drag **
            // ----------------------------------------------------------------
            float aPlate      = r2 * PI;
            float aEdge       = dObject.radius * 2f * edgeHeight * A_EDGE_EFFECTIVE_GAIN;
            float rhov2o2     = env.airDensity * dForces.v2 * 0.5f;

            // Fd_edge — blunt front edge
            dForces.linDragForceEdgeN = rhov2o2 * dbg.cdEdge * aEdge * abs(cos(dForces.aoar));

            // Fd_plate — under-cavity plate surface (positive AOA only)
            float exposedCavityPlateSurface = dForces.aoar > 0f ? aPlate * sin(dForces.aoar) : 0f;
            dForces.linDragForcePlateN      = rhov2o2 * Cd_PLATE * exposedCavityPlateSurface;
            totalExposedSurfaceAreaM2      += exposedCavityPlateSurface;

            // ----------------------------------------------------------------
            // ** Dome Camber Form Drag **
            // ----------------------------------------------------------------
            float effectiveDomeRadius      = max(0f, dObject.radius - 0.01f);
            float domeCamberNormAngle      = atan2(effectiveDomeRadius, dObject.domeHeight);

            float domeCamberSurfFront = max(0f, sin(-dForces.aoar + (PI * 0.5f - domeCamberNormAngle)));
            float domeCamberSurfBack  = max(0f, sin(-dForces.aoar + (domeCamberNormAngle - PI * 0.5f)));

            float domeCamberAreaFront = (aPlate * 0.5f) * domeCamberSurfFront;
            float domeCamberAreaBack  = (aPlate * 0.5f) * domeCamberSurfBack;

            dForces.linDragForceFrontDomeCamberN = rhov2o2 * Cd_PLATE * domeCamberAreaFront;
            dForces.linDragForceBackDomeCamberN  = rhov2o2 * Cd_PLATE * domeCamberAreaBack;
            totalExposedSurfaceAreaM2 += domeCamberAreaFront + domeCamberAreaBack;

            // ----------------------------------------------------------------
            // ** Cavity Edge Drag + Bernoulli Lift **
            // ----------------------------------------------------------------
            const float cavityExposedCircumference = 0.3f;
            float aEffLip = 2f * cavityExposedCircumference * PI
                          * (dObject.radius - dObject.rimWidth)
                          * dObject.rimDepth * dbg.cavityEdgeExposedAreaFactor;
            float aEffLipAtAoa = aEffLip * cos(dForces.aoar);

            float cavityLiftMaxAngle  = radians( 90f);
            float cavityLiftMinAngle  = radians(-15f);
            float cavityLiftPeakAngle = radians( 20f);

            float cavityEdgeEffMag = cos(dForces.aoar - cavityLiftPeakAngle)
                                   * (dForces.aoar <= cavityLiftMaxAngle && dForces.aoar >= cavityLiftMinAngle ? 1f : 0f);
            float aEffLipAtAoaBounded = aEffLipAtAoa * cavityEdgeEffMag;

            dForces.linDragForceCavityEdgeN   = rhov2o2 * dbg.cdEdge * aEffLipAtAoaBounded;
            totalExposedSurfaceAreaM2        += aEffLipAtAoaBounded;

            // Bernoulli cavity lift
            float liftFactor = 0f;
            if (aEffLip > 0f)
                liftFactor = (1f / aEffLip) * 0.8f * 0.00035302903145605f;

            dForces.liftForceCavityEdgeN = rhov2o2 * dbg.clCavity * liftFactor * aEffLipAtAoa * cavityEdgeEffMag;

            // ----------------------------------------------------------------
            // ** Dome Camber Bernoulli Lift **
            // ----------------------------------------------------------------
            float camberHeight               = dObject.domeHeight;
            float camberRectArcLength        = sqrt(dObject.radius * dObject.radius + camberHeight * camberHeight) * 2f;
            float camberArcToDiameterRatio   = camberRectArcLength / (dObject.radius * 2f) - 1f;
            float camberRectArcLengthRefScale = 1f / (sqrt(dObject.radius * dObject.radius + 0.02f * 0.02f)
                                                     * 2f / (dObject.radius * 2f) - 1f);

            float domeCamberLiftMaxAngle  = radians( 50f);
            float domeCamberLiftMinAngle  = radians(-30f);
            float domeCamberLiftPeakAngle = radians(  0f);

            float domeCamberEffMag = cos(dForces.aoar - domeCamberLiftPeakAngle)
                                   * (dForces.aoar <= domeCamberLiftMaxAngle && dForces.aoar >= domeCamberLiftMinAngle ? 1f : 0f);

            float camberLiftFactor = camberArcToDiameterRatio * camberRectArcLengthRefScale;
            dForces.liftForceCamberN = rhov2o2 * aPlate * dbg.clCamber * camberLiftFactor * domeCamberEffMag;

            // ----------------------------------------------------------------
            // ** Rim Camber Form Drag + Pitching Moment **
            // ----------------------------------------------------------------
            float rimCamberNormAngle = atan2(dObject.rimWidth, dObject.rimCamberHeight);

            float rimCamberArea = sqrt(dObject.rimWidth  * dObject.rimWidth
                                     + dObject.rimCamberHeight * dObject.rimCamberHeight)
                                * dObject.radius * 2f * RIM_CAMBER_EXPOSURE;

            // Shape multiplier: 0=Flat, 1=Concave, 2=Convex
            float rimCamberShapeMult = dObject.rimCamberShapeInt switch
            {
                1 => 1.25f,   // Concave
                2 => 0.75f,   // Convex
                _ => 1.0f     // Flat / NONE
            };

            float rimCamberSurfFront = max(0f, sin(dForces.aoar + (PI * 0.5f - rimCamberNormAngle))) * rimCamberShapeMult;
            float rimCamberSurfBack  = max(0f, sin(dForces.aoar + (rimCamberNormAngle - PI * 0.5f))) * rimCamberShapeMult;

            float rimCamberAreaFront = rimCamberArea * rimCamberSurfFront;
            float rimCamberAreaBack  = rimCamberArea * rimCamberSurfBack;

            dForces.linDragForceFrontRimCamberN = rhov2o2 * dbg.cdEdge * rimCamberAreaFront;
            dForces.linDragForceBackRimCamberN  = rhov2o2 * dbg.cdEdge * rimCamberAreaBack;
            totalExposedSurfaceAreaM2 += rimCamberAreaFront + rimCamberAreaBack;

            float rimCamberMomentArmLength = dObject.radius - (dObject.rimWidth * 0.5f);
            dForces.rotTorqueRimCamberOffsetNm = rimCamberMomentArmLength * sin(rimCamberNormAngle)
                * (dForces.linDragForceFrontRimCamberN - dForces.linDragForceBackRimCamberN);

            // ----------------------------------------------------------------
            // ** Pitching Moment Arms (cavity + camber Bernoulli) **
            // ----------------------------------------------------------------
            float flCavityEdgeMomentArmLength = dbg.pitchingMomentCavityLiftOffset * dObject.radius * 2f;
            dForces.rotTorqueCavityEdgeOffsetNm = -flCavityEdgeMomentArmLength * dForces.liftForceCavityEdgeN;

            float flDomeCamberMomentArmLength = dbg.pitchingMomentCamberLiftOffset * dObject.radius * 2f;
            dForces.rotTorqueCamberOffsetNm   = flDomeCamberMomentArmLength * dForces.liftForceCamberN;

            // plate pitching moment is disabled (set to 0) — matches C++ commented-out block
            dForces.rotTorquePlateOffsetNm = 0f;

            // ----------------------------------------------------------------
            // ** Rotational Drag (paddle + skin) **
            // ----------------------------------------------------------------
            const float xyRotFormDragLimit = 0.13412f;

            dForces.rotDragTorqueXNm =
                -Signum(dState.discRollingVel) *
                2f * xyRotFormDragLimit * Cd_PLATE * r5 *
                env.airDensity *
                (dState.discRollingVel * dState.discRollingVel);

            dForces.rotDragTorqueYNm =
                -Signum(dState.discPitchingVel) *
                2f * xyRotFormDragLimit * Cd_PLATE * r5 *
                env.airDensity *
                (dState.discPitchingVel * dState.discPitchingVel);

            // Parasitic spin drag (rotational skin drag about disc Z axis)
            float angDiscNormalToAirspeed = AngleBetweenVectors(discAirVelocity, dState.discOrientZVect);
            float airspeedVelMagDiscPlane  = sin(angDiscNormalToAirspeed) * dForces.velocityMagnitude;
            float reRot = (abs(dState.discRotationVel) * r2)
                        / max(airspeedVelMagDiscPlane, CLOSE_TO_ZERO);

            float cm = 0f;
            if (airspeedVelMagDiscPlane > CLOSE_TO_ZERO)
                cm = Cm_BASE * (1f / max(sqrt(reRot), CLOSE_TO_ZERO));

            dForces.rotDragTorqueZNm =
                -Signum(dState.discRotationVel) *
                0.5f * env.airDensity *
                (dState.discRotationVel * dState.discRotationVel) *
                r5 * cm;

            // ----------------------------------------------------------------
            // ** Parasitic Linear Skin Drag **
            // ----------------------------------------------------------------
            dForces.linDragForceSkinN = rhov2o2 * Cd_SKIN * totalExposedSurfaceAreaM2;

            // ----------------------------------------------------------------
            // ** Sum forces and torques **
            // ----------------------------------------------------------------
            dForces.aeroTorqueX = dForces.rotDragTorqueXNm;
            dForces.aeroTorqueY = dForces.rotDragTorqueYNm;
            dForces.aeroTorqueZ = dForces.rotDragTorqueZNm;

            // Drag force vector
            dForces.dragForceVector  = dForces.linDragForceEdgeN        * edgeForceVector;
            dForces.dragForceVector += dForces.linDragForcePlateN       * dOrientation;
            dForces.dragForceVector += dForces.linDragForceSkinN        * -dForces.discVelocityUnitVector;
            dForces.dragForceVector += dForces.linDragForceCavityEdgeN  * edgeForceVector;

            // Rim camber form drag — normal and edge components
            dForces.dragForceVector += dForces.linDragForceFrontRimCamberN * sin(rimCamberNormAngle) * dOrientation;
            dForces.dragForceVector += dForces.linDragForceBackRimCamberN  * sin(rimCamberNormAngle) * dOrientation;
            dForces.dragForceVector += dForces.linDragForceFrontRimCamberN * cos(rimCamberNormAngle) * edgeForceVector;
            dForces.dragForceVector -= dForces.linDragForceBackRimCamberN  * cos(rimCamberNormAngle) * edgeForceVector;

            // Dome camber form drag — normal and edge components
            dForces.dragForceVector -= dForces.linDragForceFrontDomeCamberN * sin(domeCamberNormAngle) * dOrientation;
            dForces.dragForceVector -= dForces.linDragForceBackDomeCamberN  * sin(domeCamberNormAngle) * dOrientation;
            dForces.dragForceVector += dForces.linDragForceFrontDomeCamberN * cos(domeCamberNormAngle) * edgeForceVector;
            dForces.dragForceVector -= dForces.linDragForceBackDomeCamberN  * cos(domeCamberNormAngle) * edgeForceVector;

            // Lift force vector
            dForces.liftForceVector  = dForces.liftForceCavityEdgeN * dOrientation;
            dForces.liftForceVector += dForces.liftForceCamberN     * dOrientation;

            // Pitching moment sum
            dForces.liftInducedPitchingMoment  = dForces.rotTorqueRimCamberOffsetNm;
            dForces.liftInducedPitchingMoment += dForces.rotTorquePlateOffsetNm;
            dForces.liftInducedPitchingMoment += dForces.rotTorqueCavityEdgeOffsetNm;
            dForces.liftInducedPitchingMoment += dForces.rotTorqueCamberOffsetNm;

            // Net aero force (gravity added in Dpropagate, not here)
            dForces.aeroForce = dForces.liftForceVector + dForces.dragForceVector;
        }
    }

    // ------------------------------------------------------------------------
    // GaussState  — per-ThrowContainer state for the Gaussian noise generator.
    // Replaces the static locals in C++ gaussrand().
    // ------------------------------------------------------------------------

    public class GaussState
    {
        public System.Random rng = new System.Random();
        public float v1, v2, s;
        public int   phase;

        public GaussState(int seed = 0)
        {
            rng   = seed == 0 ? new System.Random() : new System.Random(seed);
            phase = 0;
        }
    }
}
