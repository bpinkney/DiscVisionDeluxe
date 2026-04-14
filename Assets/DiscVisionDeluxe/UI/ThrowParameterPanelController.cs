using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;
using DfisX;
using DiscVisionDeluxe.Camera;
using DiscVisionDeluxe.Visualization;

namespace DiscVisionDeluxe.UI
{
    /// <summary>
    /// Runtime controller for ThrowParameterPanel.uxml.
    ///
    /// Attach to a GameObject that also has a UIDocument component.
    /// Assign the UIDocument's Source Asset to ThrowParameterPanel.uxml.
    ///
    /// Inspector wiring:
    ///   discVisualizer    → the DiscVisualizer in your scene
    ///   discModelLibrary  → the Disc Model Library ScriptableObject asset
    ///
    /// This replaces the DiscThrowDebugger OnGUI buttons with a proper
    /// UI Toolkit panel. DiscThrowDebugger can remain in the scene for
    /// its context-menu and Inspector-based throw workflow.
    /// </summary>
    [RequireComponent(typeof(UIDocument))]
    public class ThrowParameterPanelController : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("References")]
        public DiscVisualizer    discVisualizer;
        public DiscModelLibrary  discModelLibrary;
        public FollowFlightCamera followFlightCamera;
        public ShotPreviewLine   shotPreviewLine;

        // ── Throw parameter state (mirrors DiscThrowDebugger fields) ────────

        float _speedMps       = 22f;
        float _headingDeg     = 0f;
        float _launchAngleDeg = 3f;
        float _hyzerDeg       = 0f;
        float _pitchDeg       = 0f;
        float _spinRpm        = -700f;
        float _wobble         = 0f;

        // ── Environment state ────────────────────────────────────────────────

        float      _windX       = 0f;
        float      _windY       = 0f;
        float      _windZ       = 0f;
        float      _airDensity  = 1.225f;
        int        _gustFactor  = 0;

        // ── Aero debug state ─────────────────────────────────────────────────

        float _cdEdge                         = 0.6f;
        float _clCavity                       = 45.0f;
        float _clCamber                       = 1.0f;
        float _cavityEdgeExposedAreaFactor    = 1.0f;
        float _pitchingMomentCavityLiftOffset = 0.042f;
        float _pitchingMomentCamberLiftOffset = 0.15f;

        // ── Disc selection ───────────────────────────────────────────────────

        // Authoritative selected disc — set by dropdown or recent-pill click.
        DiscModel        _selectedModel      = null;

        string           _filterType         = "All";
        string           _filterManufacturer = "All";
        string           _filterStability    = "All";
        string           _filterSearch       = "";
        List<DiscModel>  _filteredDiscs      = new List<DiscModel>();
        DropdownField    _discDropdown       = null;
        VisualElement    _recentsContainer   = null;

        // Static so the list survives re-binding (e.g. domain reload in Editor).
        static readonly List<DiscModel> _recentDiscs = new List<DiscModel>();
        const int MaxRecents = 5;

        // ── Trail ────────────────────────────────────────────────────────────

        bool _keepAllThrows = false;

        // ── Cached last throw (for Re-throw) ─────────────────────────────────

        DiscInitState _lastState;

        // ── Cached UI elements ───────────────────────────────────────────────

        Label _statsLabel;

        // ── Unity lifecycle ──────────────────────────────────────────────────

        void Start()
        {
            var root = GetComponent<UIDocument>().rootVisualElement;

            BindDisc(root);
            BindThrow(root);
            BindEnvironment(root);
            BindAeroDebug(root);
            BindTrail(root);
            BindCamera(root);
            BindButtons(root);

            _statsLabel = root.Q<Label>("stats-label");

            // Seed the ghost trajectory with the default parameter values
            RequestPreviewUpdate();
        }

        void Update()
        {
            if (_statsLabel == null || discVisualizer == null) return;

            if (discVisualizer.IsSimFlying)
            {
                _statsLabel.text = ">> FLYING <<";
            }
            else if (discVisualizer.LastFlightStats.timeAloftS > 0f)
            {
                var s = discVisualizer.LastFlightStats;
                _statsLabel.text =
                    $"Distance: {s.distanceM:F1} m  ({s.distanceM * 3.281f:F0} ft)\n" +
                    $"Time aloft: {s.timeAloftS:F2} s\n" +
                    $"Steps: {s.stepCount}";
            }
        }

        // ── Disc dropdown + filters ──────────────────────────────────────────

        void BindDisc(VisualElement root)
        {
            // ── Type filter ──────────────────────────────────────────────────
            var typeFilter = root.Q<DropdownField>("type-filter");
            if (typeFilter != null)
            {
                typeFilter.choices = new List<string> { "All", "Putter", "Midrange", "Fairway", "Driver" };
                typeFilter.index   = 0;
                typeFilter.RegisterValueChangedCallback(evt =>
                {
                    _filterType = evt.newValue;
                    RebuildDiscDropdown();
                });
            }

            // ── Stability filter ─────────────────────────────────────────────
            var stabFilter = root.Q<DropdownField>("stability-filter");
            if (stabFilter != null)
            {
                stabFilter.choices = new List<string> { "All", "Understable", "Stable", "Overstable" };
                stabFilter.index   = 0;
                stabFilter.RegisterValueChangedCallback(evt =>
                {
                    _filterStability = evt.newValue;
                    RebuildDiscDropdown();
                });
            }

            // ── Manufacturer filter ──────────────────────────────────────────
            var mfrFilter = root.Q<DropdownField>("manufacturer-filter");
            if (mfrFilter != null)
            {
                var mfrSet = new SortedSet<string>();
                if (discModelLibrary != null)
                    foreach (var d in discModelLibrary.discs)
                        if (d != null) mfrSet.Add(d.manufacturer);

                var mfrChoices = new List<string> { "All" };
                mfrChoices.AddRange(mfrSet);

                mfrFilter.choices = mfrChoices;
                mfrFilter.index   = 0;
                mfrFilter.RegisterValueChangedCallback(evt =>
                {
                    _filterManufacturer = evt.newValue;
                    RebuildDiscDropdown();
                });
            }

            // ── Search field ─────────────────────────────────────────────────
            var searchField = root.Q<TextField>("disc-search");
            if (searchField != null)
            {
                searchField.RegisterValueChangedCallback(evt =>
                {
                    _filterSearch = evt.newValue ?? "";
                    RebuildDiscDropdown();
                });
            }

            // ── Disc dropdown (filtered result) ──────────────────────────────
            _discDropdown = root.Q<DropdownField>("disc-dropdown");
            RebuildDiscDropdown();

            if (_discDropdown != null)
            {
                _discDropdown.RegisterValueChangedCallback(evt =>
                {
                    int idx = _discDropdown.index;
                    if (idx >= 0 && idx < _filteredDiscs.Count)
                    {
                        _selectedModel = _filteredDiscs[idx];
                        RefreshRecentsUI();       // update active highlight
                        RequestPreviewUpdate();
                    }
                });
            }

            // ── Recents container ────────────────────────────────────────────
            _recentsContainer = root.Q<VisualElement>("recents-container");
            RefreshRecentsUI();
        }

        void RebuildDiscDropdown()
        {
            _filteredDiscs.Clear();

            if (discModelLibrary != null)
            {
                string search = _filterSearch.Trim().ToLowerInvariant();

                foreach (var d in discModelLibrary.discs)
                {
                    if (d == null) continue;
                    if (_filterType         != "All" && d.discType     != _filterType)         continue;
                    if (_filterManufacturer != "All" && d.manufacturer != _filterManufacturer) continue;
                    if (_filterStability    != "All" && d.stability    != _filterStability)    continue;
                    if (search.Length > 0 &&
                        !d.moldName.ToLowerInvariant().Contains(search) &&
                        !d.manufacturer.ToLowerInvariant().Contains(search))                   continue;

                    _filteredDiscs.Add(d);
                }
            }

            var names = new List<string>();
            foreach (var d in _filteredDiscs)
                names.Add($"{d.manufacturer} - {d.moldName}");

            if (names.Count == 0) names.Add("(No matching discs)");

            if (_discDropdown != null)
            {
                _discDropdown.choices = names;
                _discDropdown.index   = 0;
            }

            // Keep _selectedModel pointing at whatever is now at index 0,
            // unless it's still present in the filtered list (preserve it).
            if (_filteredDiscs.Count > 0)
            {
                int keep = _filteredDiscs.IndexOf(_selectedModel);
                if (keep >= 0 && _discDropdown != null)
                    _discDropdown.index = keep;
                else
                    _selectedModel = _filteredDiscs[0];
            }
            else
            {
                _selectedModel = null;
            }

            RequestPreviewUpdate();
        }

        // ── Recents ──────────────────────────────────────────────────────────

        void AddToRecents(DiscModel disc)
        {
            if (disc == null) return;
            _recentDiscs.Remove(disc);          // de-dupe
            _recentDiscs.Insert(0, disc);
            while (_recentDiscs.Count > MaxRecents)
                _recentDiscs.RemoveAt(_recentDiscs.Count - 1);
        }

        void RefreshRecentsUI()
        {
            if (_recentsContainer == null) return;
            _recentsContainer.Clear();

            foreach (var disc in _recentDiscs)
            {
                var captured = disc;
                var btn = new Button(() => SelectRecent(captured))
                {
                    text = captured.moldName
                };
                btn.AddToClassList("recent-btn");
                if (captured == _selectedModel)
                    btn.AddToClassList("recent-btn--active");
                _recentsContainer.Add(btn);
            }
        }

        void SelectRecent(DiscModel disc)
        {
            _selectedModel = disc;

            // Sync the main dropdown if the disc is visible in current filter
            int idx = _filteredDiscs.IndexOf(disc);
            if (idx >= 0 && _discDropdown != null)
                _discDropdown.index = idx;

            RefreshRecentsUI();
            RequestPreviewUpdate();
        }

        // ── Throw sliders ────────────────────────────────────────────────────

        void BindThrow(VisualElement root)
        {
            BindSlider(root, "speed-slider",       "speed-val",       v => { _speedMps       = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "heading-slider",     "heading-val",     v => { _headingDeg     = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "launch-angle-slider","launch-angle-val",v => { _launchAngleDeg = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "hyzer-slider",       "hyzer-val",       v => { _hyzerDeg       = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "pitch-slider",       "pitch-val",       v => { _pitchDeg       = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "spin-slider",        "spin-val",        v => { _spinRpm        = v; RequestPreviewUpdate(); }, "F0");
            BindSlider(root, "wobble-slider",      "wobble-val",      v => { _wobble         = v; RequestPreviewUpdate(); }, "F2");
        }

        // ── Environment sliders ──────────────────────────────────────────────

        void BindEnvironment(VisualElement root)
        {
            BindSlider(root, "wind-x-slider",      "wind-x-val",      v => { _windX      = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "wind-y-slider",      "wind-y-val",      v => { _windY      = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "wind-z-slider",      "wind-z-val",      v => { _windZ      = v; RequestPreviewUpdate(); }, "F1");
            BindSlider(root, "air-density-slider", "air-density-val", v => { _airDensity = v; RequestPreviewUpdate(); }, "F3");

            var gustSlider = root.Q<SliderInt>("gust-slider");
            var gustVal    = root.Q<Label>("gust-val");
            if (gustSlider != null)
            {
                gustSlider.RegisterValueChangedCallback(evt =>
                {
                    _gustFactor = evt.newValue;
                    if (gustVal != null) gustVal.text = evt.newValue.ToString();
                });
            }
        }

        // ── Aero debug sliders ───────────────────────────────────────────────

        void BindAeroDebug(VisualElement root)
        {
            BindSlider(root, "cd-edge-slider",     "cd-edge-val",     v => _cdEdge                         = v, "F3");
            BindSlider(root, "cl-cavity-slider",   "cl-cavity-val",   v => _clCavity                       = v, "F1");
            BindSlider(root, "cl-camber-slider",   "cl-camber-val",   v => _clCamber                       = v, "F3");
            BindSlider(root, "cavity-area-slider", "cavity-area-val", v => _cavityEdgeExposedAreaFactor    = v, "F3");
            BindSlider(root, "pitch-cavity-slider","pitch-cavity-val",v => _pitchingMomentCavityLiftOffset = v, "F3");
            BindSlider(root, "pitch-camber-slider","pitch-camber-val",v => _pitchingMomentCamberLiftOffset = v, "F3");

            var resetBtn = root.Q<Button>("reset-aero-btn");
            if (resetBtn != null)
                resetBtn.clicked += ResetAeroDefaults;
        }

        void ResetAeroDefaults()
        {
            var defaults = AeroDebugSettings.Default;
            _cdEdge                         = defaults.cdEdge;
            _clCavity                       = defaults.clCavity;
            _clCamber                       = defaults.clCamber;
            _cavityEdgeExposedAreaFactor    = defaults.cavityEdgeExposedAreaFactor;
            _pitchingMomentCavityLiftOffset = defaults.pitchingMomentCavityLiftOffset;
            _pitchingMomentCamberLiftOffset = defaults.pitchingMomentCamberLiftOffset;

            // Sync sliders back to new values
            var root = GetComponent<UIDocument>().rootVisualElement;
            SetSliderValue(root, "cd-edge-slider",     "cd-edge-val",     _cdEdge,                         "F3");
            SetSliderValue(root, "cl-cavity-slider",   "cl-cavity-val",   _clCavity,                       "F1");
            SetSliderValue(root, "cl-camber-slider",   "cl-camber-val",   _clCamber,                       "F3");
            SetSliderValue(root, "cavity-area-slider", "cavity-area-val", _cavityEdgeExposedAreaFactor,    "F3");
            SetSliderValue(root, "pitch-cavity-slider","pitch-cavity-val",_pitchingMomentCavityLiftOffset, "F3");
            SetSliderValue(root, "pitch-camber-slider","pitch-camber-val",_pitchingMomentCamberLiftOffset, "F3");
        }

        // ── Trail toggle ─────────────────────────────────────────────────────

        void BindTrail(VisualElement root)
        {
            var toggle = root.Q<Toggle>("keep-throws-toggle");
            if (toggle != null)
                toggle.RegisterValueChangedCallback(evt => _keepAllThrows = evt.newValue);
        }

        // ── Camera mode dropdown ─────────────────────────────────────────────

        // Dropdown choice order: Follow=0, Side=1, Overhead=2, Overview=3
        // CameraMode enum order: Overview=0, Follow=1, Side=2, Overhead=3
        // These differ — always use ModeToIndex / IndexToMode, never cast directly.

        static int ModeToIndex(FollowFlightCamera.CameraMode mode) => mode switch
        {
            FollowFlightCamera.CameraMode.Follow   => 0,
            FollowFlightCamera.CameraMode.Side     => 1,
            FollowFlightCamera.CameraMode.Overhead => 2,
            FollowFlightCamera.CameraMode.Overview => 3,
            _                                      => 0
        };

        static FollowFlightCamera.CameraMode IndexToMode(int index) => index switch
        {
            0 => FollowFlightCamera.CameraMode.Follow,
            1 => FollowFlightCamera.CameraMode.Side,
            2 => FollowFlightCamera.CameraMode.Overhead,
            3 => FollowFlightCamera.CameraMode.Overview,
            _ => FollowFlightCamera.CameraMode.Follow
        };

        void BindCamera(VisualElement root)
        {
            var dropdown = root.Q<DropdownField>("camera-dropdown");
            if (dropdown == null) return;

            dropdown.choices = new List<string> { "Follow", "Side", "Overhead", "Overview" };

            // Sync dropdown to camera's current mode and apply it so both agree on startup
            var initialMode  = followFlightCamera != null
                ? followFlightCamera.currentMode
                : FollowFlightCamera.CameraMode.Follow;

            dropdown.index = ModeToIndex(initialMode);
            followFlightCamera?.SetMode(initialMode);

            dropdown.RegisterValueChangedCallback(evt =>
                followFlightCamera?.SetMode(IndexToMode(dropdown.index)));
        }

        // ── Buttons ──────────────────────────────────────────────────────────

        void BindButtons(VisualElement root)
        {
            var launchBtn  = root.Q<Button>("launch-btn");
            var rethrowBtn = root.Q<Button>("rethrow-btn");
            var clearBtn   = root.Q<Button>("clear-btn");

            if (launchBtn  != null) launchBtn.clicked  += Launch;
            if (rethrowBtn != null) rethrowBtn.clicked += Rethrow;
            if (clearBtn   != null) clearBtn.clicked   += ClearTrails;
        }

        // ── Launch logic ─────────────────────────────────────────────────────

        void Launch()
        {
            if (discVisualizer == null)
            {
                Debug.LogError("[ThrowParameterPanel] discVisualizer not assigned.");
                return;
            }

            discVisualizer.keepAllThrows = _keepAllThrows;

            _lastState = BuildDiscInitState();
            var aero   = BuildAeroDebugSettings();

            AddToRecents(_selectedModel);
            RefreshRecentsUI();

            shotPreviewLine?.HidePreview();
            discVisualizer.LaunchDfisX(_lastState, aero);

            Debug.Log($"[ThrowParameterPanel] Launched — " +
                      $"{_speedMps:F1} m/s  hyzer {_hyzerDeg:F1}°  spin {_spinRpm:F0} RPM");
        }

        void Rethrow()
        {
            if (discVisualizer == null) return;

            if (_lastState == null)
            {
                Launch();
                return;
            }

            discVisualizer.keepAllThrows = _keepAllThrows;
            discVisualizer.LaunchDfisX(_lastState, BuildAeroDebugSettings());
            Debug.Log("[ThrowParameterPanel] Re-threw with same parameters.");
        }

        void ClearTrails()
        {
            discVisualizer?.ClearAllDfisxTrails();
            if (_statsLabel != null) _statsLabel.text = "";
        }

        // ── Build structs from current state ─────────────────────────────────

        DiscInitState BuildDiscInitState()
        {
            // Matches DiscThrowDebugger.BuildDiscInitState() exactly.
            float headingRad = Mathf.Deg2Rad * _headingDeg;
            float launchRad  = Mathf.Deg2Rad * _launchAngleDeg;

            float vHoriz  = _speedMps * Mathf.Cos(launchRad);
            Vector3 velocity = new Vector3(
                vHoriz * Mathf.Cos(headingRad),
                vHoriz * Mathf.Sin(headingRad),
                _speedMps * Mathf.Sin(launchRad));

            float spinRadS = _spinRpm * Mathf.PI * 2f / 60f;

            var state = new DiscInitState
            {
                linearPositionM    = new Vector3(0f, 0f, 1.5f),
                linearVelocityMs   = velocity,
                angularPositionRad = new Vector3(
                    Mathf.Deg2Rad * _hyzerDeg,
                    Mathf.Deg2Rad * _pitchDeg,
                    0f),
                angularVelocityRad = new Vector3(0f, 0f, spinRadS),
                wobble             = _wobble,
                discMold           = DiscIndex.DRIVER  // display/KF use; physics uses directModel
            };

            if (_selectedModel != null)
                state.directModel = _selectedModel;

            return state;
        }

        AeroDebugSettings BuildAeroDebugSettings()
        {
            return new AeroDebugSettings
            {
                cdEdge                         = _cdEdge,
                clCavity                       = _clCavity,
                clCamber                       = _clCamber,
                cavityEdgeExposedAreaFactor    = _cavityEdgeExposedAreaFactor,
                pitchingMomentCavityLiftOffset = _pitchingMomentCavityLiftOffset,
                pitchingMomentCamberLiftOffset = _pitchingMomentCamberLiftOffset
            };
        }

        // ── Preview helpers ───────────────────────────────────────────────────

        void RequestPreviewUpdate()
        {
            if (shotPreviewLine == null) return;
            shotPreviewLine.RequestUpdate(BuildThrowParameters(), BuildEnvironment(),
                                          BuildAeroDebugSettings());
        }

        ThrowParameters BuildThrowParameters()
        {
            float headingRad = Mathf.Deg2Rad * _headingDeg;
            float launchRad  = Mathf.Deg2Rad * _launchAngleDeg;
            float vHoriz     = _speedMps * Mathf.Cos(launchRad);

            var p = new ThrowParameters
            {
                position  = new Vector3(0f, 0f, 1.5f),
                velocity  = new Vector3(
                    vHoriz * Mathf.Cos(headingRad),
                    vHoriz * Mathf.Sin(headingRad),
                    _speedMps * Mathf.Sin(launchRad)),
                hyzer     = Mathf.Deg2Rad * _hyzerDeg,
                pitch     = Mathf.Deg2Rad * _pitchDeg,
                spinRate  = _spinRpm * Mathf.PI * 2f / 60f,
                wobble    = _wobble,
            };

            // Inject the exact DiscModel the user selected so the sim uses real aero data
            if (_selectedModel != null)
                p.directModel = _selectedModel;

            return p;
        }

        DiscEnvironment BuildEnvironment()
        {
            var env = DiscEnvironment.Default;
            env.windVectorXYZ = new Unity.Mathematics.float3(_windX, _windY, _windZ);
            env.airDensity    = _airDensity;
            return env;
        }

        // ── Helpers ──────────────────────────────────────────────────────────

        static void BindSlider(VisualElement root, string sliderName, string labelName,
                                System.Action<float> setter, string fmt)
        {
            var slider = root.Q<Slider>(sliderName);
            var label  = root.Q<Label>(labelName);
            if (slider == null) return;

            // Sync label to initial value
            if (label != null) label.text = slider.value.ToString(fmt);

            slider.RegisterValueChangedCallback(evt =>
            {
                setter(evt.newValue);
                if (label != null) label.text = evt.newValue.ToString(fmt);
            });
        }

        static void SetSliderValue(VisualElement root, string sliderName, string labelName,
                                    float value, string fmt)
        {
            var slider = root.Q<Slider>(sliderName);
            var label  = root.Q<Label>(labelName);
            if (slider != null) slider.value = value;
            if (label  != null) label.text   = value.ToString(fmt);
        }
    }
}
