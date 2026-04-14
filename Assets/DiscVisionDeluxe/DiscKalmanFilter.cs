using System;
using System.Collections.Generic;
using UnityEngine;

namespace DiscVisionDeluxe
{
    /// <summary>
    /// C# port of the Kalman Filter from dvd_DvisEst_estimate.cpp.
    ///
    /// Implements a decoupled 2-state (position + velocity) KF on each of 6 axes:
    ///   Linear:  X, Y, Z
    ///   Angular: Hyzer, Pitch, Spin
    ///
    /// Key design choices preserved from original:
    ///   - Constant-velocity prediction model
    ///   - Separate covariance matrix per axis (not a full 12x12)
    ///   - Measurement priming queue with velocity variance check
    ///   - Minimum-variance ideal state tracking (anti-wobble mean for H/P)
    /// </summary>
    public class DiscKalmanFilter
    {
        // ---------------------------------------------------------------
        // Tunable parameters — expose via Inspector through DiscSimulator
        // ---------------------------------------------------------------
        [Serializable]
        public class KFParams
        {
            // Prediction step rate
            public double predDtS = 1.0 / 1000.0; // 1000 Hz

            // Measurement noise (variance)
            public double linPosMeasVar = 0.01;  // m^2
            public double angPosMeasVar = 0.01;  // rad^2

            // Process noise (scaled by dt inside prediction step)
            public double linPosProcVar = 0.5;   // m^2 / s
            public double linVelProcVar = 20.0;  // (m/s)^2 / s
            public double angPosProcVar = 0.5;
            public double angVelProcVar = 20.0;

            // Initial covariance
            public double linPosVarInit = 0.05;
            public double linVelVarInit = 10.0;
            public double angPosVarInit = 0.05;
            public double angVelVarInit = 10.0;

            // Priming queue config (mirrors MEAS_PRIME_QUEUE_* defines)
            // Lower maxEntries means filter primes sooner on short logs
            public int   primeMaxEntries  = 8;
            public int   primeCount       = 5;
            public double primeMinVar     = 5.0; // (m/s)^2 — relaxed for synthetic data
        }

        // ---------------------------------------------------------------
        // State
        // ---------------------------------------------------------------
        public enum Stage { MeasCollect, Ready, Prime, Active, Complete }
        public Stage CurrentStage { get; private set; } = Stage.MeasCollect;

        public KFState  CurrentState  { get; private set; } = new KFState();
        public KFState  IdealState    { get; private set; } = new KFState();
        public bool     HasIdealState { get; private set; } = false;

        private KFParams _p;

        // Priming queue — mirrors meas_prime_queue deque
        private readonly Queue<KFMeasurement> _primeQueue = new Queue<KFMeasurement>();
        private int _primeIdx = 0;

        // Ideal state tracking
        private readonly Queue<KFState> _idealCheckQueue = new Queue<KFState>();
        private const int IdealCheckQueueSize = 5;
        private double _minVarianceSum = double.MaxValue;

        // Anti-wobble running mean for Hyzer and Pitch
        private double _angHMean, _angPMean;
        private double _angHMax = -1e9, _angPMax = -1e9;
        private double _angHMin =  1e9, _angPMin =  1e9;
        private int    _angMeanCount;

        // ---------------------------------------------------------------
        public DiscKalmanFilter(KFParams parameters = null)
        {
            _p = parameters ?? new KFParams();
        }

        public void Reset()
        {
            CurrentStage   = Stage.MeasCollect;
            CurrentState   = new KFState();
            IdealState     = new KFState();
            HasIdealState  = false;
            _primeQueue.Clear();
            _primeIdx      = 0;
            _idealCheckQueue.Clear();
            _minVarianceSum = double.MaxValue;
            _angHMean = _angPMean = 0;
            _angHMax = _angPMax = -1e9;
            _angHMin = _angPMin =  1e9;
            _angMeanCount = 0;
        }

        /// <summary>
        /// Feed a new measurement into the filter.
        /// Call this for every measurement coming from CSV or AprilTag.
        /// </summary>
        public void AddMeasurement(KFMeasurement meas)
        {
            if (CurrentStage >= Stage.Active) return;

            _primeQueue.Enqueue(meas);

            if (CurrentStage < Stage.Ready)
                TryPrimeCheck();
        }

        /// <summary>
        /// Call once after measurements stop arriving to trigger priming.
        /// Mirrors the apriltag timeout logic in process_filter_thread.
        /// </summary>
        public void SignalDetectionLost()
        {
            if (CurrentStage == Stage.Ready)
                CurrentStage = Stage.Prime;
        }

        /// <summary>
        /// Step the filter forward. Call at predDtS intervals (or faster, it self-limits).
        /// Mirrors the main loop in process_filter_thread.
        /// Returns true if a new ideal state was set this step.
        /// </summary>
        public bool Step(ulong nowNs)
        {
            if (CurrentStage == Stage.Prime)
                PrimeFilter();

            if (CurrentStage < Stage.Active)
                return false;

            bool newIdeal = false;

            // Consume measurements older than current state timestamp
            while (_primeQueue.Count > 0 &&
                   CurrentState.timestampNs > _primeQueue.Peek().timestampNs)
            {
                var meas = _primeQueue.Dequeue();
                MeasUpdateStep(meas);
                newIdeal |= CheckIdealState();
            }

            // Prediction step
            PredictionStep();
            CurrentState.timestampNs += (ulong)(KFParams_PredDtNs());

            if (_primeQueue.Count == 0 && CurrentStage < Stage.Complete)
                CurrentStage = Stage.Complete;

            return newIdeal;
        }

        // ---------------------------------------------------------------
        // Priming logic — mirrors kf_meas_update_step Stage < Active block
        // ---------------------------------------------------------------
        private void TryPrimeCheck()
        {
            int qSize = _primeQueue.Count;
            if (qSize < _p.primeCount) return;

            var measArray = _primeQueue.ToArray(); // non-destructive peek

            // Check velocity variance of last primeCount entries
            double[] velVar = new double[3];
            double[] velMean = new double[3];
            int n = _p.primeCount;

            for (int i = 0; i < n - 1; i++)
            {
                double dt = Math.Max(1e-9,
                    NsToS(measArray[qSize - 1 - i].timestampNs - measArray[qSize - 2 - i].timestampNs));
                velMean[0] += (measArray[qSize - 1 - i].linX - measArray[qSize - 2 - i].linX) / dt;
                velMean[1] += (measArray[qSize - 1 - i].linY - measArray[qSize - 2 - i].linY) / dt;
                velMean[2] += (measArray[qSize - 1 - i].linZ - measArray[qSize - 2 - i].linZ) / dt;
            }
            for (int k = 0; k < 3; k++) velMean[k] /= (n - 1);

            bool varMet = true;
            for (int i = 0; i < n - 1; i++)
            {
                double dt = Math.Max(1e-9,
                    NsToS(measArray[qSize - 1 - i].timestampNs - measArray[qSize - 2 - i].timestampNs));
                double[] v = {
                    (measArray[qSize - 1 - i].linX - measArray[qSize - 2 - i].linX) / dt,
                    (measArray[qSize - 1 - i].linY - measArray[qSize - 2 - i].linY) / dt,
                    (measArray[qSize - 1 - i].linZ - measArray[qSize - 2 - i].linZ) / dt
                };
                for (int k = 0; k < 3; k++)
                {
                    double diff = v[k] - velMean[k];
                    if (diff * diff > _p.primeMinVar) { varMet = false; break; }
                }
                if (!varMet) break;
            }

            if (varMet)
            {
                _primeIdx = qSize - 1;
                CurrentStage = Stage.Ready;
            }

            if (qSize >= _p.primeMaxEntries && CurrentStage == Stage.MeasCollect)
            {
                _primeIdx = qSize - 1;
                CurrentStage = Stage.Ready;
            }
        }

        private void PrimeFilter()
        {
            var measArray = _primeQueue.ToArray();
            int n = _p.primeCount;
            int end = Math.Min(_primeIdx, measArray.Length - 1);
            int start = Math.Max(0, end - n + 1);

            // Build dt series for linear fit
            double[] dtSeries = new double[n];
            double[] linXSeries = new double[n];
            double[] linYSeries = new double[n];
            double[] linZSeries = new double[n];

            for (int i = 0; i < n; i++)
            {
                int idx = Math.Min(start + i, measArray.Length - 1);
                dtSeries[i]  = NsToS(measArray[idx].timestampNs - measArray[start].timestampNs);
                linXSeries[i] = measArray[idx].linX;
                linYSeries[i] = measArray[idx].linY;
                linZSeries[i] = measArray[idx].linZ;
            }

            int primeIdx = Math.Min(end, measArray.Length - 1);
            var primeMeas = measArray[primeIdx];

            // Set initial positions from latest measurement
            CurrentState.linX.pos = primeMeas.linX;
            CurrentState.linY.pos = primeMeas.linY;
            CurrentState.linZ.pos = primeMeas.linZ;
            CurrentState.angHyzer.pos = primeMeas.angHyzer;
            CurrentState.angPitch.pos = primeMeas.angPitch;
            CurrentState.angSpin.pos  = primeMeas.angSpin;

            // Linear fit for velocities
            CurrentState.linX.vel = LinearFitSlope(dtSeries, linXSeries, n);
            CurrentState.linY.vel = LinearFitSlope(dtSeries, linYSeries, n);
            CurrentState.linZ.vel = LinearFitSlope(dtSeries, linZSeries, n);

            // Mean angular velocity
            double[] angHVel = new double[n - 1], angPVel = new double[n - 1], angSVel = new double[n - 1];
            for (int i = 0; i < n - 1; i++)
            {
                int idx  = Math.Min(start + i + 1, measArray.Length - 1);
                int idx0 = Math.Min(start + i,     measArray.Length - 1);
                double dt = Math.Max(1e-9, NsToS(measArray[idx].timestampNs - measArray[idx0].timestampNs));
                angHVel[i] = Wrap2Pi(measArray[idx].angHyzer - measArray[idx0].angHyzer) / dt;
                angPVel[i] = Wrap2Pi(measArray[idx].angPitch - measArray[idx0].angPitch) / dt;
                angSVel[i] = Wrap2Pi(measArray[idx].angSpin  - measArray[idx0].angSpin)  / dt;
            }
            CurrentState.angHyzer.vel = Mean(angHVel);
            CurrentState.angPitch.vel = Mean(angPVel);
            CurrentState.angSpin.vel  = Mean(angSVel);

            // Initial covariances
            SetInitialCovariance(ref CurrentState.linX,    _p.linPosVarInit, _p.linVelVarInit);
            SetInitialCovariance(ref CurrentState.linY,    _p.linPosVarInit, _p.linVelVarInit);
            SetInitialCovariance(ref CurrentState.linZ,    _p.linPosVarInit, _p.linVelVarInit);
            SetInitialCovariance(ref CurrentState.angHyzer, _p.angPosVarInit, _p.angVelVarInit);
            SetInitialCovariance(ref CurrentState.angPitch, _p.angPosVarInit, _p.angVelVarInit);
            SetInitialCovariance(ref CurrentState.angSpin,  _p.angPosVarInit, _p.angVelVarInit);

            CurrentState.timestampNs = primeMeas.timestampNs;
            CurrentState.discIndex   = primeMeas.discIndex;

            // Discard measurements used for priming
            for (int i = 0; i <= primeIdx; i++)
                if (_primeQueue.Count > 0) _primeQueue.Dequeue();

            CurrentStage = Stage.Active;
        }

        // ---------------------------------------------------------------
        // Measurement update — mirrors kf_meas_update_step Active block
        // ---------------------------------------------------------------
        private void MeasUpdateStep(KFMeasurement meas)
        {
            MeasUpdateAxis(ref CurrentState.linX,    meas.linX,     _p.linPosMeasVar, false);
            MeasUpdateAxis(ref CurrentState.linY,    meas.linY,     _p.linPosMeasVar, false);
            MeasUpdateAxis(ref CurrentState.linZ,    meas.linZ,     _p.linPosMeasVar, false);
            MeasUpdateAxis(ref CurrentState.angHyzer, meas.angHyzer, _p.angPosMeasVar, true);
            MeasUpdateAxis(ref CurrentState.angPitch, meas.angPitch, _p.angPosMeasVar, true);
            MeasUpdateAxis(ref CurrentState.angSpin,  meas.angSpin,  _p.angPosMeasVar, true);
        }

        private static void MeasUpdateAxis(ref PosVelVarState s, double measPos,
                                           double measVar, bool wrapAngle)
        {
            double innovation = measPos - s.pos;
            if (wrapAngle) innovation = Wrap2Pi(innovation);

            // Kalman gain: K = P[0,0] / (S2dm + P[0,0])
            double denom = Math.Max(1e-9, measVar + s.p00);
            double k0 = s.p00 / denom;
            double k1 = s.p10 / denom;

            s.pos += k0 * innovation;
            s.vel += k1 * innovation;
            if (wrapAngle) s.pos = Wrap2Pi(s.pos);

            // Covariance update
            double p00 = s.p00, p01 = s.p01;
            s.p00 += -k0 * p00;
            s.p01 += -k0 * p01;
            s.p10 += -k1 * p00;
            s.p11 += -k1 * p01;
        }

        // ---------------------------------------------------------------
        // Prediction step — mirrors kf_prediction_step
        // ---------------------------------------------------------------
        private void PredictionStep()
        {
            double dt = _p.predDtS;
            PredictAxis(ref CurrentState.linX,    dt, _p.linPosProcVar * dt, _p.linVelProcVar * dt, false);
            PredictAxis(ref CurrentState.linY,    dt, _p.linPosProcVar * dt, _p.linVelProcVar * dt, false);
            PredictAxis(ref CurrentState.linZ,    dt, _p.linPosProcVar * dt, _p.linVelProcVar * dt, false);
            PredictAxis(ref CurrentState.angHyzer, dt, _p.angPosProcVar * dt, _p.angVelProcVar * dt, true);
            PredictAxis(ref CurrentState.angPitch, dt, _p.angPosProcVar * dt, _p.angVelProcVar * dt, true);
            PredictAxis(ref CurrentState.angSpin,  dt, _p.angPosProcVar * dt, _p.angVelProcVar * dt, true);
        }

        private static void PredictAxis(ref PosVelVarState s, double dt,
                                        double s2dp, double s2vp, bool wrapAngle)
        {
            s.pos += s.vel * dt;
            if (wrapAngle) s.pos = Wrap2Pi(s.pos);

            double p12 = s.p01, p21 = s.p10, p22 = s.p11;
            s.p00 += dt * dt * p22 + dt * (p12 + p21) + s2dp;
            s.p01 += dt * p22;
            s.p10 += dt * p22;
            s.p11 += s2vp;
        }

        // ---------------------------------------------------------------
        // Ideal state tracking — mirrors kf_check_for_ideal_output_state
        // ---------------------------------------------------------------
        private bool CheckIdealState()
        {
            if (CurrentStage < Stage.Active) return false;

            _idealCheckQueue.Enqueue(CurrentState.Clone());
            while (_idealCheckQueue.Count > IdealCheckQueueSize)
                _idealCheckQueue.Dequeue();

            if (_idealCheckQueue.Count < IdealCheckQueueSize) return false;

            var states = new List<KFState>(_idealCheckQueue);
            double[] velMean = new double[3];
            foreach (var s in states)
            {
                velMean[0] += s.linX.vel;
                velMean[1] += s.linY.vel;
                velMean[2] += s.linZ.vel;
            }
            for (int k = 0; k < 3; k++) velMean[k] /= IdealCheckQueueSize;

            double varSum = 0;
            foreach (var s in states)
            {
                double dx = s.linX.vel - velMean[0];
                double dy = s.linY.vel - velMean[1];
                double dz = s.linZ.vel - velMean[2];
                varSum += dx * dx + dy * dy + dz * dz;
            }

            // Update anti-wobble running mean
            if (varSum < _minVarianceSum * 4.0 || varSum < 1.0)
            {
                _angHMean += CurrentState.angHyzer.pos;
                _angPMean += CurrentState.angPitch.pos;
                _angMeanCount++;
                _angHMax = Math.Max(_angHMax, CurrentState.angHyzer.pos);
                _angPMax = Math.Max(_angPMax, CurrentState.angPitch.pos);
                _angHMin = Math.Min(_angHMin, CurrentState.angHyzer.pos);
                _angPMin = Math.Min(_angPMin, CurrentState.angPitch.pos);
            }

            bool newIdeal = varSum < _minVarianceSum;
            if (newIdeal)
            {
                _minVarianceSum = varSum;
                IdealState = CurrentState.Clone();
            }

            // Apply anti-wobble mean to ideal state H/P
            if (_angMeanCount > 0)
            {
                IdealState.angHyzer.pos = _angHMean / _angMeanCount;
                IdealState.angPitch.pos = _angPMean / _angMeanCount;
            }

            double angleDelta = ((_angHMax - _angHMin) + (_angPMax - _angPMin)) * 0.5;
            IdealState.wobbleMag = angleDelta / (Math.PI * 0.5);
            IdealState.discIndex = CurrentState.discIndex;

            HasIdealState = IdealState.timestampNs > 0;
            return newIdeal;
        }

        // ---------------------------------------------------------------
        // Helpers
        // ---------------------------------------------------------------
        private static void SetInitialCovariance(ref PosVelVarState s, double posVar, double velVar)
        {
            s.p00 = posVar; s.p01 = 0;
            s.p10 = 0;      s.p11 = velVar;
        }

        private static double LinearFitSlope(double[] x, double[] y, int n)
        {
            if (n < 2) return 0;
            double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
            for (int i = 0; i < n; i++)
            {
                sumX += x[i]; sumY += y[i];
                sumXY += x[i] * y[i]; sumX2 += x[i] * x[i];
            }
            double xMean = sumX / n;
            double yMean = sumY / n;
            double denom = sumX2 - sumX * xMean;
            return Math.Abs(denom) < 1e-7 ? 0 : (sumXY - sumX * yMean) / denom;
        }

        private static double Mean(double[] arr)
        {
            double s = 0;
            foreach (var v in arr) s += v;
            return s / Math.Max(1, arr.Length);
        }

        private static double Wrap2Pi(double rad)
        {
            while (rad >  Math.PI) rad -= 2 * Math.PI;
            while (rad < -Math.PI) rad += 2 * Math.PI;
            return rad;
        }

        private static double NsToS(ulong ns) => ns * 1e-9;
        private ulong KFParams_PredDtNs() => (ulong)(_p.predDtS * 1e9);
    }
}
