# MiniMap — Design

## Architecture
Software-drawn `Texture2D` — no extra camera or RenderTexture.

## Coordinate Mapping
- World X (flight direction) → pixel Y (tee at bottom)
- World Z (lateral) → pixel X (flipped)

## Grid
Horizontal lines at `gridIntervalFt` intervals (default 50ft). Labels at `labelIntervalFt` (default 50ft).

## Trail
Reads `dfisxTrajectoryLine` positions + `DiscVisualizer.GetArchivedTrails()`. Drawn at `trailThicknessPx` (default 3px).

## Auto-Find
`FindAnyObjectByType<DiscVisualizer>()` in `Start()` if not wired in Inspector.

## Files
- `Assets/DiscVisionDeluxe/UI/MiniMap.cs`
