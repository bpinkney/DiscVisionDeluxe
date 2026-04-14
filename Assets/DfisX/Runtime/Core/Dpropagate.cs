using Unity.Mathematics;
using static Unity.Mathematics.math;
using float3 = Unity.Mathematics.float3;
using float3x3 = Unity.Mathematics.float3x3;

// ============================================================================
//  Dpropagate.cs
//  Port of Dpropagate.cpp — Euler integration of disc state.
//
//  Porting notes:
//  - Eigen::Matrix3d rotation matrices → hand-rolled float3x3 using math.*
//  - Eigen is column-major by default, and the C++ explicitly transposes Rx
//    and Ry after construction ("Note this is column major, so it is really
//    the transpose of what you see here"). The math below builds the matrices
//    in the correct (row-major, non-transposed) form directly so no transpose
//    is needed.
//  - Rdw (disc-to-world rotation matrix) is built from the three orientation
//    unit vectors as columns, which in row-major storage is the transpose.
//    The C++ transposes Eigen's column-major result to get the same thing.
//    The C# below constructs this correctly without an explicit transpose.
//  - disc_state_array.push_back(d_state) → tc.RecordState() which writes to
//    the pre-allocated NativeArray<DiscState> trajectory buffer.
//  - Gravity: added here as (0, 0, -GRAV * mass) in world frame, matching
//    the C++ exactly. Unity Y-up coordinate system note: if your scene uses
//    Unity's Y-up convention, swap the gravity vector to (0, -GRAV*mass, 0)
//    in DiscFlightSimulator.cs when constructing the ThrowContainer. Daero
//    and Dpropagate are coordinate-system agnostic.
// ============================================================================

namespace DfisX
{
    public static class Dpropagate
    {
        const float GRAV = 9.80665f;

        /// <summary>
        /// Euler-integrate one simulation step.
        /// Records the pre-propagation state into the trajectory buffer,
        /// then advances position, velocity, orientation, and spin.
        ///
        /// Call order inside step_simulation():
        ///   1. StepDaero     (Daero.cs)
        ///   2. StepDgyro     (Dgyro.cs)
        ///   3. Propagate     ← this method
        /// </summary>
        public static void Propagate(ThrowContainer tc, float dt)
        {
            ref DiscState     dState  = ref tc.currentDiscState;
            ref ForcesState   dForces = ref tc.currentDiscState.forcesState;
            ref DiscModelData dObject = ref tc.discObject;

            dState.lastDt = dt;

            float dt2 = dt * dt * 0.5f;

            // Moments of inertia for a thin disc
            float Ix = 0.25f * dObject.mass * dObject.radius * dObject.radius;
            float Iy = 0.25f * dObject.mass * dObject.radius * dObject.radius;
            float Iz = 0.50f * dObject.mass * dObject.radius * dObject.radius;

            // ----------------------------------------------------------------
            // Linear acceleration  (aero + collision + gravity + friction)
            // ----------------------------------------------------------------
            dForces.netForce =
                dForces.aeroForce
              + dForces.collisionForce
              + new float3(0f, 0f, -GRAV * dObject.mass)
              + tc.frictionInput.linForceXYZ;

            dState.discAcceleration = dForces.netForce / dObject.mass;

            // ----------------------------------------------------------------
            // Angular acceleration
            // ----------------------------------------------------------------

            // Pitch axis (Y): gyro + aero + collision + friction
            dForces.netTorqueX = dForces.gyroTorqueX
                               + dForces.aeroTorqueX
                               + dForces.collisionTorqueXYZ.x
                               + tc.frictionInput.angTorqueXYZ.x;

            dForces.netTorqueY = dForces.gyroTorqueY
                               + dForces.aeroTorqueY
                               + dForces.collisionTorqueXYZ.y
                               + tc.frictionInput.angTorqueXYZ.y;

            dState.discRollingAccel  = dForces.netTorqueX / Ix;
            dState.discPitchingAccel = dForces.netTorqueY / Iy;

            // Spin axis (Z)
            dForces.netTorqueZ = dForces.aeroTorqueZ
                               + dForces.collisionTorqueXYZ.z
                               + tc.frictionInput.angTorqueXYZ.z;

            dState.discRotationAccel = dForces.netTorqueZ / Iz;

            // ----------------------------------------------------------------
            // Record state BEFORE propagation (matches C++ push_back timing)
            // ----------------------------------------------------------------
            tc.RecordState();

            // ----------------------------------------------------------------
            // Propagate linear state
            // ----------------------------------------------------------------
            dState.discLocation += dState.discVelocity     * dt + dState.discAcceleration * dt2;
            dState.discVelocity += dState.discAcceleration * dt;

            // ----------------------------------------------------------------
            // Propagate orientation (Z unit vector rotation)
            //
            // Strategy (mirrors C++):
            //   1. Compute roll and pitch deltas in DISC frame
            //   2. Build rotation matrices Rx (about disc X) and Ry (about disc Y)
            //   3. Apply each to the canonical base_z (0,0,1) to get the delta
            //   4. Rotate that delta back to world frame via Rdw
            //   5. Add delta to disc_orient_z_vect and renormalise
            //   6. Recompute X and Y unit vectors orthonormal to new Z
            // ----------------------------------------------------------------
            float rollRad  = dState.discRollingVel  * dt + dState.discRollingAccel  * dt2;
            float pitchRad = dState.discPitchingVel * dt + dState.discPitchingAccel * dt2;

            // Rx — rotation about disc X axis (roll)
            // Row-major, directly in the correct form (no transpose needed)
            float cR = cos(rollRad);
            float sR = sin(rollRad);
            float3x3 Rx = new float3x3(
                new float3(1f,  0f,  0f),
                new float3(0f,  cR, -sR),
                new float3(0f,  sR,  cR));

            // Ry — rotation about disc Y axis (pitch)
            float cP = cos(pitchRad);
            float sP = sin(pitchRad);
            float3x3 Ry = new float3x3(
                new float3( cP, 0f, sP),
                new float3( 0f, 1f, 0f),
                new float3(-sP, 0f, cP));

            // Rdw — disc-to-world rotation matrix (columns = disc X, Y, Z axes)
            // In row-major storage this is the transpose, which is what we want
            // because we are rotating from disc-frame deltas to world-frame.
            float3x3 Rdw = new float3x3(
                new float3(dState.discOrientXVect.x, dState.discOrientYVect.x, dState.discOrientZVect.x),
                new float3(dState.discOrientXVect.y, dState.discOrientYVect.y, dState.discOrientZVect.y),
                new float3(dState.discOrientXVect.z, dState.discOrientYVect.z, dState.discOrientZVect.z));

            // Compute the Z unit vector delta in disc frame, then transform to world
            float3 baseZ    = new float3(0f, 0f, 1f);
            float3 zDelta   = (mul(Rx, baseZ) - baseZ)
                            + (mul(Ry, baseZ) - baseZ);
            zDelta = mul(Rdw, zDelta);

            // Apply delta and renormalise
            dState.discOrientZVect = normalize(dState.discOrientZVect + zDelta);

            // Recompute X and Y to stay orthonormal to the new Z
            dState.discOrientYVect = normalize(cross(dForces.discVelocityUnitVector, dState.discOrientZVect));
            dState.discOrientXVect = normalize(cross(dState.discOrientYVect,         dState.discOrientZVect));

            // ----------------------------------------------------------------
            // Propagate angular velocities
            // ----------------------------------------------------------------
            dState.discRollingVel  += dState.discRollingAccel  * dt;
            dState.discPitchingVel += dState.discPitchingAccel * dt;

            dState.discRotation    += dState.discRotationVel   * dt + dState.discRotationAccel * dt2;
            dState.discRotationVel += dState.discRotationAccel * dt;

            dForces.stepCount++;
        }
    }
}
