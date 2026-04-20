# Disc Visualization — Design

## Components

### DiscMeshBuilder (Visualization/)
Static class. `Build(DiscModel, segments, domeSegs)` → surface-of-revolution mesh from disc dimensions.

### DiscVisualController (Visualization/)
MonoBehaviour (RequireComponent MeshFilter, MeshRenderer). `SetDiscModel(DiscModel)` rebuilds mesh + resets scale to Vector3.one. Both `LaunchDfisX` overloads call `discVisualController?.SetDiscModel(initState.directModel)`.
- `fallbackModel`: used for live-camera throws (no directModel). Should eventually resolve from DiscIndex.
- Material: URP Lit, off-white (0.95, 0.95, 0.95), smoothness 0.55 — created at runtime, no asset on disk.

### DiscFoilStamp (Visualization/)
Created by `DiscVisualController.Awake()`. Sibling component.
- Font loaded at runtime from Windows OS fonts via `TMP_FontAsset.CreateFontAsset()`. Cached statically per name.
- Colour = complementary hue (h+0.5) to disc body. Vertex gradient + embossed outline.
- 5 decoration presets (LineRenderers, ZTest=Disabled, renderQueue=3000/3001).
- Font + decoration seeded from `manufacturer::moldName` hash — deterministic per mold.

### ShotPreviewLine (Visualization/)
Runs synchronous `SimulateThrow()` on each `RequestUpdate()`, throttled to `maxUpdatesPerSecond=5Hz` via `Time.unscaledTime`.
- Call `RequestUpdate(ThrowParameters, DiscEnvironment, AeroDebugSettings)` from every slider callback.
- Call `HidePreview()` immediately before `LaunchDfisX` fires.
- Seed initial values in `ThrowParameterPanelController.Start()`.

### LandingMarker (Visualization/)
Self-pooling world-space TMP label. `LandingMarker.Spawn(worldPos, distanceM)`.
- Floats upward at `FloatSpeedMps` (0.6 m/s). Holds opacity for first 20% of `FadeDurationS` (5s) then fades. Billboards to `Camera.main`.
- `DiscVisualizer` Inspector: `showLandingMarker` toggle, `landingMarkerHeightM` (default 1.5m).

### DfisX Trail System (DiscVisualizer)
`dfisxTrajectoryLine` LineRenderer. `keepAllThrows` toggle.
Methods: `PrepareNewDfisxTrail` / `AppendDfisxTrailPoint` / `ClearAllDfisxTrails`.
- `liveThrowTrailColor` / `liveThrowTrailWidth` — per-throw style overrides for live throws.
- Per-disc-type colors: `List<DiscTypeTrailStyle>` (DiscIndex → color + width) in `keepAllThrows` mode.
- `dfisxTrajectoryLine` must have vertex-color-capable material. `Awake()` auto-creates `URP/Particles/Unlit` if `sharedMaterial==null`.

## Pending: Spin Animation (POL-10)
Mesh + foil stamp complete. Spin animation (UV rotation / normal-map rotation) not yet implemented. Approach: rotate UV offset each frame based on `ThrowContainer.throwSpinRateRadS`.
