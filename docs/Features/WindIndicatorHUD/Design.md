# Wind Indicator HUD — Design

## Overview

A corner HUD overlay that visualizes current wind conditions using a 3D windsock rendered in world space (or as a world-space camera-facing object composited into the HUD). Shows wind direction, speed in kph, cardinal label, and animated gust behaviour.

---

## Wind Simulation — `WindField`

The existing `Daero.ComputeGusts()` inline logic will be extracted into a standalone `WindField` class responsible for sampling wind at any world position and time. This decouples wind state from individual throws and makes the HUD (and future features such as Replay or Course Holes) able to query wind independently.

### Responsibilities
- Owns `windVectorXYZ` (direction + magnitude, m/s) and `GustFactor` enum value
- Exposes a `Sample(Vector3 worldPos, float time)` method returning a `DiscEnvironment` struct for the given position/time
- Handles spatial variation if needed in future (e.g. terrain sheltering) — initial implementation can be uniform
- Replaces the direct `DiscEnvironment` construction in `DiscVisualizer` and `ThrowParameterPanelController`

### Location
`Assets/DiscVisionDeluxe/Environment/WindField.cs`

---

## Throw Panel Input Changes

The three raw X/Y/Z axis sliders are replaced with **direction + speed** controls that match how players think about wind, plus a named gust selector.

| Old control | New control |
|---|---|
| Wind X slider (m/s) | **Direction** — degrees, 0–359 (compass heading the wind blows *from*) |
| Wind Y slider (m/s) | *(removed — direction encodes both horizontal axes)* |
| Wind Z slider (m/s) | **Vertical component** — ±5 m/s slider (kept, optional, defaults 0) |
| Gust slider (0–10 int) | **Gust selector** — dropdown using exact `GustFactor` enum names (see below) |
| Air density slider | *(unchanged)* |

Speed control: slider, 0–60 kph, displayed in kph. Converted to m/s internally before building `DiscEnvironment`.

**`GustFactor` dropdown entries (exact enum names):**
- `ZeroDeadDiddly`
- `OneDullDraft`
- `TwoCalmChinook`
- `ThreeBrusqueBreeze`
- `FourRobustGust`
- `FiveZealousZephyr`
- `SixGalledGale`
- `SevenFuriousFlurry`
- `EightTerribleTempest`
- `NinePsychoticCyclone`
- `TenHomicidalHurricane`

Direction + speed are converted to `windVectorXYZ` by `WindField` using standard compass-to-vector math (wind *from* N = vector pointing S in world space).

---

## HUD Windsock — Visual Design

### Component type
3D GameObject rendered by a dedicated small camera (or a world-space Canvas camera) composited into a corner of the screen. Using a real 3D mesh (not a UI Toolkit element) gives natural cloth-like orientation and animation.

### Geometry
- Rigid pole (static)
- Tapered mesh cone/cylinder representing the sock body — 6–8 segments, UV-mapped for stripe texture
- Sock pivots at the pole attachment point; tip droops or extends based on wind

### Orientation and droop
- The sock's rest pose hangs straight down (no wind)
- At each frame, the target rotation of the sock is computed from `WindField.Sample()`:
  - Horizontal angle = wind direction (compass)
  - Droop angle (elevation from horizontal) = f(speed in knots): mapped so that ~5 kn = 45°, ~15 kn = 80° (nearly horizontal), ~30+ kn = fully extended
- Rotation is smoothed with a spring/damper so it doesn't snap

**Knot mapping reference (approx):**

| Speed | Droop from horizontal |
|---|---|
| 0 kn (calm) | −90° (straight down) |
| 5 kn (~9 kph) | −45° |
| 15 kn (~28 kph) | −10° |
| 30+ kn (~55+ kph) | 0° (fully extended) |

### Gust animation
- `GustFactor` enum value drives a looping procedural animation on the sock mesh vertices (ripple along the length)
- Ripple amplitude and frequency are keyed to the int value (0 = no ripple, 10 = violent flutter)
- Animation is continuous/ambient, not a one-shot event — it represents the turbulence level set before the throw

### Labels (UI Toolkit overlay on top of the 3D sock camera)
- **Cardinal direction** — 8-point compass label (S, SW, W, NW, N, NE, E, SE) derived from the wind-from direction
- **Speed** — kph value, no decimal, static wind speed only (gusts excluded from the label)
- Both labels sit below the sock in a small UI Toolkit overlay anchored to the same corner

---

## Data Flow

```
WindField (ScriptableObject or MonoBehaviour singleton)
    ├── windDirectionDeg   (float, compass degrees)
    ├── windSpeedKph       (float)
    ├── windVerticalMs     (float, optional)
    └── gustFactor         (GustFactor enum)

    ↓ Sample(pos, time) → DiscEnvironment
         ├── windVectorXYZ  (m/s, XYZ world)
         └── gustFactor

ThrowParameterPanelController  ──reads──►  WindField  ──►  DiscEnvironment  ──►  DiscFlightSimulator
WindIndicator (HUD)            ──reads──►  WindField
```

---

## Reference Axes

`DiscEnvironment.windVectorXYZ` uses the DfisX Z-up world frame (X=forward, Y=right, Z=up). Compass direction (wind *from*) maps to this as:

- N (0°/360°) → wind vector points in −X direction
- E (90°) → wind vector points in −Y direction
- S (180°) → wind vector points in +X direction
- W (270°) → wind vector points in +Y direction

---

## Files

| File | Notes |
|---|---|
| `Assets/DiscVisionDeluxe/Environment/WindField.cs` | New — wind simulation and sampling |
| `Assets/DiscVisionDeluxe/UI/WindIndicator.cs` | New — HUD controller, reads WindField |
| `Assets/DiscVisionDeluxe/UI/WindIndicatorHUD.uxml` | New — cardinal + speed labels overlay |
| `Assets/DiscVisionDeluxe/UI/WindIndicatorHUD.uss` | New — styling |
| `Assets/DiscVisionDeluxe/UI/ThrowParameterPanel.uxml` | Modify — replace X/Y/Z sliders + gust int slider |
| `Assets/DiscVisionDeluxe/UI/ThrowParameterPanelController.cs` | Modify — bind new direction/speed/gust controls, read from WindField |
