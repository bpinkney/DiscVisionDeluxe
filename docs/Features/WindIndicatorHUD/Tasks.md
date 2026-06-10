# Wind Indicator HUD — Tasks

## Active Session
- **Last action**: None
- **Next action**: Implement windsock HUD (POL-5)
- **Blocker**: None

## Current Sprint
- [ ] POL-5a: Extract wind simulation into `WindField` class (replaces inline `DiscEnvironment` construction in `DiscVisualizer` and `ThrowParameterPanelController`)
- [ ] POL-5b: Rework throw panel environment inputs — direction (0–359°), speed (kph), vertical component, `GustFactor` dropdown with exact enum names
- [ ] POL-5c: 3D windsock mesh + pole GameObject for HUD corner
- [ ] POL-5d: Sock orientation and droop driven by wind speed in knots (spring-damped)
- [ ] POL-5e: Gust ripple animation keyed to `GustFactor` int value (ambient/looping)
- [ ] POL-5f: UI Toolkit overlay — cardinal direction label (8-point) and speed in kph below sock

## Design Notes
See [Design.md](Design.md) for full spec.

Summary:
- `WindField` owns direction/speed/gust state and exposes `Sample(pos, time) → DiscEnvironment`
- Throw panel inputs: compass direction degrees, speed in kph, optional vertical m/s, `GustFactor` enum dropdown
- Windsock is a 3D mesh (not UI Toolkit); sock elevation from horizontal encodes speed in knots (~5 kn = 45°, ~15 kn = 80°, 30+ kn = fully horizontal)
- Ripple animation amplitude/frequency driven by `GustFactor` value
- Labels below sock: 8-point cardinal (S/SW/W/NW/N/NE/E/SE) and speed in kph (static wind only)

## Backlog
_(none)_

## Completed
_(none)_
