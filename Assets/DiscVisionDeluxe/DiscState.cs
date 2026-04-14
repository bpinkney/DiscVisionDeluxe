using UnityEngine;

namespace DiscVisionDeluxe
{
    /// <summary>
    /// Mirrors disc_layouts.hpp DiscIndex enum
    /// </summary>
    public enum DiscIndex
    {
        NONE = 0,
        GROUNDPLANE,
        GROUNDPLANE_BIG,
        PUTTER,
        PUTTER_OS,
        PUTTER_US,
        MIDRANGE,
        MIDRANGE_OS,
        MIDRANGE_US,
        FAIRWAY,
        FAIRWAY_OS,
        FAIRWAY_US,
        DRIVER,
        DRIVER_OS,
        DRIVER_US,
        SPECIAL
    }

    /// <summary>
    /// Mirrors pos_vel_var_state_t from dvd_DvisEst_estimate.hpp
    /// Holds position, velocity, and 2x2 covariance for a single axis.
    /// Covariance layout is row-major: [p00, p01, p10, p11]
    /// </summary>
    [System.Serializable]
    public struct PosVelVarState
    {
        public double pos;
        public double vel;
        // 2x2 covariance matrix, row-major
        public double p00, p01, p10, p11;

        public double PosVariance => p00;
        public double VelVariance => p11;
    }

    /// <summary>
    /// Mirrors dvd_DvisEst_kf_state_t from dvd_DvisEst_estimate.hpp
    /// Full Kalman Filter state for position, velocity, and covariance on all 6 axes.
    ///
    /// Coordinate frame (from dvd_DvisEst_estimate.hpp):
    ///   X = positive toward throw direction
    ///   Y = positive to the right
    ///   Z = positive up
    ///   Origin = ground plane point
    ///
    /// Angular frame (Hyzer, Pitch, Spin):
    ///   Hyzer = rotation about world X (forward) axis
    ///   Pitch = rotation about world Y (right) axis
    ///   Spin  = rotation in disc frame about disc normal
    /// </summary>
    [System.Serializable]
    public class KFState
    {
        public ulong timestampNs;

        // Linear XYZ states
        public PosVelVarState linX;
        public PosVelVarState linY;
        public PosVelVarState linZ;

        // Angular Hyzer, Pitch, Spin states
        public PosVelVarState angHyzer;
        public PosVelVarState angPitch;
        public PosVelVarState angSpin;

        public double wobbleMag;
        public DiscIndex discIndex;

        // Convenience accessors
        public Vector3 LinearPosition =>
            new Vector3((float)linX.pos, (float)linY.pos, (float)linZ.pos);

        public Vector3 LinearVelocity =>
            new Vector3((float)linX.vel, (float)linY.vel, (float)linZ.vel);

        public float LinearSpeedKph =>
            LinearVelocity.magnitude * 3.6f;

        /// <summary>
        /// Returns a Unity Quaternion from the Hyzer/Pitch/Spin angles.
        /// Hyzer = Z rotation (tilt left/right), Pitch = X rotation (nose up/down).
        /// Spin is disc-frame yaw and is handled separately for visualization.
        /// </summary>
        public Quaternion DiscOrientation =>
            Quaternion.Euler(
                Mathf.Rad2Deg * (float)angPitch.pos,
                Mathf.Rad2Deg * (float)angSpin.pos,
                Mathf.Rad2Deg * (float)angHyzer.pos
            );

        public KFState Clone()
        {
            return (KFState)MemberwiseClone();
        }
    }

    /// <summary>
    /// Mirrors dvd_DvisEst_kf_meas_t — a single raw measurement from AprilTag detection
    /// or CSV log replay, before KF processing.
    /// </summary>
    [System.Serializable]
    public class KFMeasurement
    {
        public ulong timestampNs;
        public uint frameId;

        // Linear XYZ position measurement (metres)
        public double linX, linY, linZ;

        // Angular Hyzer, Pitch, Spin measurement (radians)
        public double angHyzer, angPitch, angSpin;

        public DiscIndex discIndex;
        public byte player;

        public Vector3 LinearPosition =>
            new Vector3((float)linX, (float)linY, (float)linZ);
    }

    /// <summary>
    /// Mirrors disc_init_state_t — the final output state served to the physics sim.
    /// This is what gets handed off to dvd_DfisX equivalent.
    /// </summary>
    [System.Serializable]
    public class DiscInitState
    {
        public Vector3 linearPositionM;   // metres
        public Vector3 linearVelocityMs;  // m/s
        public Vector3 angularPositionRad; // [hyzer, pitch, spin] radians
        public Vector3 angularVelocityRad; // [hyzer_d, pitch_d, spin_d] rad/s
        public float wobble;               // [0,1]
        public DiscIndex discMold;

        /// <summary>
        /// Optional direct disc model reference — bypasses the enum→library lookup in DfisX.
        /// Set by ThrowParameterPanelController so the exact dropdown-selected disc is used.
        /// Not serialized — runtime only.
        /// </summary>
        [System.NonSerialized]
        public DfisX.DiscModel directModel;

        public static DiscInitState FromKFState(KFState state)
        {
            return new DiscInitState
            {
                linearPositionM   = state.LinearPosition,
                linearVelocityMs  = state.LinearVelocity,
                angularPositionRad = new Vector3(
                    (float)state.angHyzer.pos,
                    (float)state.angPitch.pos,
                    (float)state.angSpin.pos),
                angularVelocityRad = new Vector3(
                    (float)state.angHyzer.vel,
                    (float)state.angPitch.vel,
                    (float)state.angSpin.vel),
                wobble    = (float)state.wobbleMag,
                discMold  = state.discIndex
            };
        }
    }
}
