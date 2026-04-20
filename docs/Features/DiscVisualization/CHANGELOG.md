# Disc Visualization — Changelog

## [Unreleased]
- POL-10: Spin animation (UV/normal-map rotation)

## Session 12-13 — POL-10 First Pass
- DiscMeshBuilder.cs: surface-of-revolution mesh generation from DiscModel dimensions
- DiscVisualController.cs: SetDiscModel(), runtime URP Lit material (no asset on disk)
- DiscFoilStamp.cs: complementary hue, vertex gradient, embossed outline, 5 decoration presets, deterministic hash from manufacturer::moldName

## Session 8-9 — Polish
- ShotPreviewLine.cs: ghost trajectory preview, throttled to 5Hz
- LandingMarker.cs: self-pooling world-space TMP label, float + fade animation

## Session 6 — POL-0
- DfisX trail customization: liveThrowTrailColor, per-disc-type colors in keepAllThrows mode
