using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

namespace DiscVisionDeluxe
{
    /// <summary>
    /// Drives the Kalman Filter from CSV log data.
    /// Feeds measurements in, steps the filter, and fires events when states are available.
    ///
    /// Usage:
    ///   1. Assign a CSV log file path or TextAsset in the Inspector.
    ///   2. Hook up OnNewState and OnIdealStateReady UnityEvents to your visualizer.
    ///   3. Call StartSimulation() or enable AutoStartOnPlay.
    /// </summary>
    public class DiscSimulator : MonoBehaviour
    {
        // ---------------------------------------------------------------
        // Inspector
        // ---------------------------------------------------------------
        [Header("Data Source")]
        [Tooltip("Path to a csvlog.csv file on disk (leave empty to use TextAsset below)")]
        public string csvFilePath = "";

        [Tooltip("CSV log bundled as a TextAsset in Resources (used if csvFilePath is empty)")]
        public TextAsset csvTextAsset;

        [Header("Playback")]
        [Tooltip("Play back at this multiple of real-time speed. 1 = real-time, 0 = instant (process all at once)")]
        [Range(0f, 10f)]
        public float playbackSpeed = 1f;

        [Tooltip("Start simulation automatically on Play")]
        public bool autoStartOnPlay = true;

        [Header("Kalman Filter Parameters")]
        public DiscKalmanFilter.KFParams kfParams = new DiscKalmanFilter.KFParams();

        // ---------------------------------------------------------------
        // Events
        // ---------------------------------------------------------------
        [Header("Events")]
        [Tooltip("Fired every KF step while the filter is active")]
        public UnityEvent<KFState> onNewState;

        [Tooltip("Fired each time a new minimum-variance ideal state is found")]
        public UnityEvent<KFState> onIdealStateReady;

        [Tooltip("Fired once when the filter reaches Complete stage with the final ideal output")]
        public UnityEvent<DiscInitState> onThrowComplete;

        // ---------------------------------------------------------------
        // Public state
        // ---------------------------------------------------------------
        public DiscKalmanFilter.Stage FilterStage => _kf?.CurrentStage ?? DiscKalmanFilter.Stage.MeasCollect;
        public KFState CurrentState  => _kf?.CurrentState;
        public KFState IdealState    => _kf?.IdealState;

        // ---------------------------------------------------------------
        // Private
        // ---------------------------------------------------------------
        private DiscKalmanFilter _kf;
        private List<KFMeasurement> _measurements;
        private Coroutine _simCoroutine;
        private bool _completeFired;

        // ---------------------------------------------------------------
        void Start()
        {
            if (autoStartOnPlay)
                StartSimulation();
        }

        // ---------------------------------------------------------------
        public void StartSimulation()
        {
            if (_simCoroutine != null)
                StopCoroutine(_simCoroutine);

            _kf = new DiscKalmanFilter(kfParams);
            _completeFired = false;

            // Load measurements
            if (!string.IsNullOrEmpty(csvFilePath))
                _measurements = CsvLogReader.LoadFromFile(csvFilePath);
            else if (csvTextAsset != null)
                _measurements = CsvLogReader.LoadFromTextAsset(csvTextAsset);
            else
            {
                Debug.LogError("[DiscSimulator] No CSV source assigned.");
                return;
            }

            if (_measurements == null || _measurements.Count == 0)
            {
                Debug.LogError("[DiscSimulator] No measurements loaded.");
                return;
            }

            // Normalise timestamps to start at zero
            ulong t0 = _measurements[0].timestampNs;
            foreach (var m in _measurements)
                m.timestampNs -= t0;

            _simCoroutine = StartCoroutine(
                playbackSpeed <= 0f ? RunInstant() : RunRealtime());
        }

        public void StopSimulation()
        {
            if (_simCoroutine != null)
            {
                StopCoroutine(_simCoroutine);
                _simCoroutine = null;
            }
        }

        public void ResetSimulation()
        {
            StopSimulation();
            _kf?.Reset();
        }

        // ---------------------------------------------------------------
        // Instant mode — process all measurements in one frame, then step
        // ---------------------------------------------------------------
        private IEnumerator RunInstant()
        {
            Debug.Log("[DiscSimulator] Running in instant mode.");

            foreach (var meas in _measurements)
                _kf.AddMeasurement(meas);

            _kf.SignalDetectionLost();

            ulong nowNs = _measurements[_measurements.Count - 1].timestampNs;
            while (_kf.CurrentStage != DiscKalmanFilter.Stage.Complete)
            {
                bool newIdeal = _kf.Step(nowNs);
                nowNs += (ulong)(_kf.CurrentState != null ? 1_000_000UL : 1_000_000UL);

                onNewState?.Invoke(_kf.CurrentState);
                if (newIdeal)
                    onIdealStateReady?.Invoke(_kf.IdealState);
            }

            FireComplete();
            yield return null;
        }

        // ---------------------------------------------------------------
        // Realtime mode — drip-feed measurements according to timestamps
        // ---------------------------------------------------------------
        private IEnumerator RunRealtime()
        {
            Debug.Log($"[DiscSimulator] Running at {playbackSpeed}x speed.");

            int measIdx = 0;
            ulong simNs = 0;
            double stepNs = kfParams.predDtS * 1e9;
            float realDtAccum = 0f;

            while (_kf.CurrentStage != DiscKalmanFilter.Stage.Complete)
            {
                realDtAccum += Time.deltaTime * playbackSpeed;
                ulong advanceNs = (ulong)(realDtAccum * 1e9);
                realDtAccum -= (float)(advanceNs * 1e-9);

                ulong targetNs = simNs + advanceNs;

                // Feed measurements that fall within this time window
                while (measIdx < _measurements.Count &&
                       _measurements[measIdx].timestampNs <= targetNs)
                {
                    _kf.AddMeasurement(_measurements[measIdx]);
                    measIdx++;
                }

                // Signal detection lost once all measurements fed
                if (measIdx >= _measurements.Count &&
                    _kf.CurrentStage == DiscKalmanFilter.Stage.Ready)
                    _kf.SignalDetectionLost();

                // Step filter up to targetNs
                while (simNs < targetNs)
                {
                    bool newIdeal = _kf.Step(simNs);
                    simNs += (ulong)stepNs;

                    onNewState?.Invoke(_kf.CurrentState);
                    if (newIdeal)
                        onIdealStateReady?.Invoke(_kf.IdealState);
                }

                if (_kf.CurrentStage == DiscKalmanFilter.Stage.Complete && !_completeFired)
                    FireComplete();

                yield return null;
            }
        }

        private void FireComplete()
        {
            if (_completeFired) return;
            _completeFired = true;

            if (_kf.HasIdealState)
            {
                var initState = DiscInitState.FromKFState(_kf.IdealState);
                Debug.Log($"[DiscSimulator] Throw complete. Speed: {initState.linearVelocityMs.magnitude * 3.6f:F1} kph");
                onThrowComplete?.Invoke(initState);
            }
            else
            {
                Debug.LogWarning("[DiscSimulator] Simulation complete but no ideal state found.");
            }
        }
    }
}
