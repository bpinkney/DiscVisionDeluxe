using Unity.Mathematics;
using static Unity.Mathematics.math;
using float3 = Unity.Mathematics.float3;
using float3x3 = Unity.Mathematics.float3x3;

// ============================================================================
//  Dgyro.cs
//  Port of Dgyro.cpp — gyroscopic precession for disc flight.
//
//  Porting notes:
//  - double → float  (consistent with Daero.cs decision)
//  - The `mode` bool parameter in C++ only enables a block that is entirely
//    commented-out dead code. It is dropped here.
//  - make_unit_vector(disc_orient_z_vect) at the end of C++ Dgyro is omitted:
//    Dpropagate re-normalises all orientation vectors after propagation,
//    making this redundant. The normalise was a safety measure in C++ for
//    the case where Dgyro ran without Dpropagate following it.
//  - friction_input.ang_torque_XYZ and collision_torque_xyz are included in
//    the net moment computation, matching the C++ exactly.
// ============================================================================

namespace DfisX
{
    public static class Dgyro
    {
        const float CLOSE_TO_ZERO = 0.000001f;

        /// <summary>
        /// Compute gyroscopic precession torques for one simulation step.
        /// Writes gyroTorqueX and gyroTorqueY into forcesState.
        ///
        /// Call order inside step_simulation():
        ///   1. StepDaero   (Daero.cs)
        ///   2. StepDgyro   ← this method
        ///   3. Propagate   (Dpropagate.cs)
        /// </summary>
        public static void StepDgyro(ThrowContainer tc, float dt)
        {
            ref DiscState     dState  = ref tc.currentDiscState;
            ref ForcesState   dForces = ref tc.currentDiscState.forcesState;
            ref DiscModelData dObject = ref tc.discObject;

            // Moments of inertia for a thin disc
            float Ix = 0.25f * dObject.mass * dObject.radius * dObject.radius;
            float Iy = 0.25f * dObject.mass * dObject.radius * dObject.radius;
            float Iz = 0.50f * dObject.mass * dObject.radius * dObject.radius;

            dForces.gyroTorqueY = 0f;
            dForces.gyroTorqueX = 0f;

            // Only compute precession when the disc has meaningful spin
            if (abs(dState.discRotationVel) > 0.1f)
            {
                // Net pitch and roll moments (aero + collision + friction)
                float newPitchMoment = dForces.liftInducedPitchingMoment
                                     + dForces.aeroTorqueY
                                     + dForces.collisionTorqueXYZ.y
                                     + tc.frictionInput.angTorqueXYZ.y;

                float newRollMoment  = 0f
                                     + dForces.aeroTorqueX
                                     + dForces.collisionTorqueXYZ.x
                                     + tc.frictionInput.angTorqueXYZ.x;

                // Gyroscopic precession rates
                // Wp = precession rate about pitch axis (rad/s)
                // Wq = precession rate about roll  axis (rad/s)
                float Wp = -(newRollMoment)  / (Iz * dState.discRotationVel);
                float Wq =  (newPitchMoment) / (Iz * dState.discRotationVel);

                // Angular accelerations needed to drive current vel → precession vel
                float dtSafe = max(dt, CLOSE_TO_ZERO);
                float WqD = (Wq - dState.discRollingVel)  / dtSafe;
                float WpD = (Wp - dState.discPitchingVel) / dtSafe;

                // Convert to torques
                dForces.gyroTorqueY = WpD * Iy;
                dForces.gyroTorqueX = WqD * Ix;
            }
        }
    }
}
