using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using Unity.Mathematics;
using static Unity.Mathematics.math;
using DfisX;

namespace DiscVisionDeluxe.UI
{
    [RequireComponent(typeof(UIDocument))]
    public class ThrowResultPanelController : MonoBehaviour
    {
        [Header("References")]
        public DiscVisualizer               discVisualizer;
        public ThrowParameterPanelController throwParamPanel;
        public DiscPreviewController         discPreview;

        const int MaxHistory = 5;

        readonly List<FlightStats> _history = new List<FlightStats>();

        // Cached throw-direction vectors for real-time lateral computation
        float2 _throwDir2D;
        float2 _perpDir2D;
        float3 _startLoc;

        DropdownField _historyDropdown;
        VisualElement _topBar;
        VisualElement _statsPanel;

        Label _discName;
        Label _speed, _spinRpm, _spinFactor, _hyzer, _noseAngle, _elevation, _azimuth;
        Label _distance, _lateral, _timeAloft;
        Label _turn, _fade;

        VisualElement _discPreviewEl;

        void Start()
        {
            var root = GetComponent<UIDocument>().rootVisualElement;

            _topBar     = root.Q("top-bar");
            _statsPanel = root.Q("stats-panel");

            _historyDropdown = root.Q<DropdownField>("history-dropdown");
            if (_historyDropdown != null)
                _historyDropdown.RegisterValueChangedCallback(_ => OnDropdownChanged());

            _discName   = root.Q<Label>("stat-disc-name");
            _speed      = root.Q<Label>("stat-speed");
            _spinRpm    = root.Q<Label>("stat-spin-rpm");
            _spinFactor = root.Q<Label>("stat-spin-factor");
            _hyzer      = root.Q<Label>("stat-hyzer");
            _noseAngle  = root.Q<Label>("stat-nose");
            _elevation  = root.Q<Label>("stat-elevation");
            _azimuth    = root.Q<Label>("stat-azimuth");
            _distance   = root.Q<Label>("stat-distance");
            _lateral    = root.Q<Label>("stat-lateral");
            _timeAloft  = root.Q<Label>("stat-time");
            _turn       = root.Q<Label>("stat-turn");
            _fade       = root.Q<Label>("stat-fade");

            _discPreviewEl = root.Q("disc-preview");
            if (_discPreviewEl != null && discPreview != null)
                StartCoroutine(BindPreviewRT());

            HidePanel();
            if (_topBar != null) _topBar.style.display = DisplayStyle.None;

            if (discVisualizer != null)
            {
                discVisualizer.OnSimStarted  += HandleSimStarted;
                discVisualizer.OnSimFinished += HandleThrowFinished;
            }
        }

        void OnDestroy()
        {
            if (discVisualizer != null)
            {
                discVisualizer.OnSimStarted  -= HandleSimStarted;
                discVisualizer.OnSimFinished -= HandleThrowFinished;
            }
        }

        IEnumerator BindPreviewRT()
        {
            yield return null; // wait one frame for DiscPreviewController.Awake to create RT
            if (_discPreviewEl != null && discPreview?.PreviewRT != null)
                _discPreviewEl.style.backgroundImage = Background.FromRenderTexture(discPreview.PreviewRT);
        }

        // ── Real-time update during flight ───────────────────────────────────

        void Update()
        {
            if (discVisualizer == null || !discVisualizer.IsSimFlying) return;

            var container = discVisualizer.ActiveContainer;
            if (container == null || container.discStateCount < 2) return;

            float3 cur    = container.currentDiscState.discLocation;
            float2 relH   = new float2(cur.x - _startLoc.x, cur.y - _startLoc.y);
            float  dist   = length(relH);
            float  lat    = dot(relH, _perpDir2D);
            float  tAloft = container.discStateCount * DiscFlightSimulator.SIM_DT_S;

            if (_distance  != null) _distance.text  = $"{dist:F1} m  ({dist * 3.281f:F0} ft)";
            if (_lateral   != null) _lateral.text   = $"{Mathf.Abs((float)lat):F1} m";
            if (_timeAloft != null) _timeAloft.text = $"{tAloft:F2} s";
        }

        // ── Event handlers ───────────────────────────────────────────────────

        void HandleSimStarted()
        {
            var container = discVisualizer.ActiveContainer;
            if (container == null || container.discStateCount == 0) return;

            // Cache throw geometry for real-time lateral updates
            float3 startVel = container.discStateArray[0].discVelocity;
            float2 velH     = new float2(startVel.x, startVel.y);
            float  hSpeed   = length(velH);
            _throwDir2D = hSpeed > 0.001f ? velH / hSpeed : new float2(1f, 0f);
            _perpDir2D  = new float2(-_throwDir2D.y, _throwDir2D.x);
            _startLoc   = container.discStateArray[0].discLocation;

            // Show panel immediately with throw-input stats
            ShowPanel();
            if (_topBar != null) _topBar.style.display = DisplayStyle.Flex;

            if (_discName != null)
                _discName.text = string.IsNullOrEmpty(container.throwDiscName)
                                 ? "Unknown" : container.throwDiscName;

            float speedMps = length(startVel);
            float rawSpin  = container.throwSpinRateRadS;
            float spinRpm  = -rawSpin * 60f / (2f * PI);
            float spinAbs  = Mathf.Abs((float)spinRpm);

            if (_speed      != null) _speed.text      = $"{speedMps:F1} m/s  ({speedMps * 2.237f:F0} mph)";
            if (_spinRpm    != null) _spinRpm.text    = $"{spinAbs:F0} rpm";
            if (_spinFactor != null) _spinFactor.text = $"{(speedMps > 0f ? spinAbs / speedMps : 0f):F1}";
            if (_hyzer      != null) _hyzer.text      = $"{degrees(container.throwHyzerRad):F1}°";
            if (_noseAngle  != null) _noseAngle.text  = $"{degrees(container.throwPitchRad):F1}°";
            if (_elevation  != null) _elevation.text  = $"{degrees(atan2(startVel.z, hSpeed)):F1}°";
            if (_azimuth    != null) _azimuth.text    = $"{degrees(atan2(startVel.y, startVel.x)):F1}°";

            if (_distance  != null) _distance.text  = "—";
            if (_lateral   != null) _lateral.text   = "—";
            if (_timeAloft != null) _timeAloft.text = "—";
            if (_turn      != null) _turn.text      = "—";
            if (_fade      != null) _fade.text      = "—";

            if (discPreview != null)
            {
                discPreview.MirrorDiscAppearance();
                discPreview.UpdateOrientation(
                    degrees(container.throwHyzerRad),
                    degrees(container.throwPitchRad));
            }
        }

        void HandleThrowFinished(FlightStats stats)
        {
            _history.Insert(0, stats);
            if (_history.Count > MaxHistory)
                _history.RemoveAt(_history.Count - 1);

            RebuildDropdown();
            if (_historyDropdown != null && _historyDropdown.choices.Count > 0)
                _historyDropdown.SetValueWithoutNotify(_historyDropdown.choices[0]);

            // Finalise the stats that couldn't be computed in real-time
            if (_turn != null) _turn.text = $"{stats.lateralTurnM:F1} m";
            if (_fade != null) _fade.text = $"{stats.lateralFadeM:F1} m";

            // Also finalise distance/lateral/time with the accurate BuildStats values
            if (_distance != null)
                _distance.text = $"{stats.horizontalDistM:F1} m  ({stats.horizontalDistM * 3.281f:F0} ft)";
            if (_lateral  != null)
                _lateral.text  = $"{Mathf.Abs(stats.lateralDistM):F1} m";
            if (_timeAloft != null)
                _timeAloft.text = $"{stats.timeAloftS:F2} s";
        }

        void OnDropdownChanged()
        {
            int idx = _historyDropdown?.index ?? 0;
            if (idx < 0 || idx >= _history.Count) return;

            var stats = _history[idx];
            ShowStatsFromHistory(stats);
            ShowPanel();
            throwParamPanel?.PopulateFromHistory(stats);
        }

        // ── Display helpers ──────────────────────────────────────────────────

        void ShowPanel()
        {
            if (_statsPanel == null) return;
            _statsPanel.style.display = DisplayStyle.Flex;
            _statsPanel.style.opacity = 1f;
        }

        void HidePanel()
        {
            if (_statsPanel == null) return;
            _statsPanel.style.opacity = 0f;
            _statsPanel.style.display = DisplayStyle.None;
        }

        void ShowStatsFromHistory(FlightStats s)
        {
            if (_discName != null)
                _discName.text = string.IsNullOrEmpty(s.discName) ? "Unknown" : s.discName;

            if (_speed != null)
                _speed.text = $"{s.throwSpeedMps:F1} m/s  ({s.throwSpeedMps * 2.237f:F0} mph)";

            if (_spinRpm    != null) _spinRpm.text    = $"{Mathf.Abs(s.spinRpm):F0} rpm";
            if (_spinFactor != null) _spinFactor.text = $"{s.spinFactor:F1}";
            if (_hyzer      != null) _hyzer.text      = $"{s.hyzerAngleDeg:F1}°";
            if (_noseAngle  != null) _noseAngle.text  = $"{s.noseAngleDeg:F1}°";
            if (_elevation  != null) _elevation.text  = $"{s.throwElevationDeg:F1}°";
            if (_azimuth    != null) _azimuth.text    = $"{s.throwAzimuthDeg:F1}°";

            if (_distance != null)
                _distance.text = $"{s.horizontalDistM:F1} m  ({s.horizontalDistM * 3.281f:F0} ft)";
            if (_lateral  != null)
                _lateral.text  = $"{Mathf.Abs(s.lateralDistM):F1} m";
            if (_timeAloft != null)
                _timeAloft.text = $"{s.timeAloftS:F2} s";

            if (_turn != null) _turn.text = $"{s.lateralTurnM:F1} m";
            if (_fade != null) _fade.text = $"{s.lateralFadeM:F1} m";

            discPreview?.UpdateOrientation(s.hyzerAngleDeg, s.noseAngleDeg);
        }

        void RebuildDropdown()
        {
            if (_historyDropdown == null) return;
            var choices = new List<string>();
            for (int i = 0; i < _history.Count; i++)
            {
                string name = string.IsNullOrEmpty(_history[i].discName) ? "?" : _history[i].discName;
                choices.Add($"#{_history.Count - i}  {name}  {_history[i].horizontalDistM:F0}m");
            }
            _historyDropdown.choices = choices;
        }
    }
}
