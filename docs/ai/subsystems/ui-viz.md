# UI & Visualization — Reference

## UI Architecture
All new panels use UI Toolkit (UXML/USS). Keep OnGUI only in DiscThrowDebugger. CameraMode dropdown uses ModeToIndex()/IndexToMode() — never cast directly (enum order ≠ UI order).

## ThrowParameterPanelController (UI/)
Key fields:
- `_selectedModel` — single authoritative DiscModel ref for all physics builds. Set by dropdown OR recent-pill click.
- `RebuildDiscDropdown()` — applies 4 AND filters: type, stability, manufacturer, free-text search (case-insensitive, matches mold name or manufacturer).
- If _selectedModel still in filtered list after filter change: scroll to it (don't reset to 0).
- Manufacturer filter: dynamically populated from discModelLibrary.discs (SortedSet<string>).
- Filter dropdowns: `compact-filter` USS class, 26px row height (same as sliders).

## Disc Recents
`static List<DiscModel> _recentDiscs` (max 5). Static — survives re-binds within play session.
Populated on each Launch via AddToRecents(_selectedModel) — deduplicates, newest first.
Pill buttons (`recent-btn` USS class). Active pill: `recent-btn--active` highlight.
SelectRecent(): sets _selectedModel directly, syncs main dropdown if disc visible in current filter.

## Cinemachine 3 Cameras (FollowFlightCamera.cs)
VCam_Follow: manually positioned every frame (Position/Rotation Control = None). Offset: (-followDistance, followHeight, 0).
VCam_Side: CinemachineFollow + RotationComposer. Tracking Target = disc. Offset: (0, 4, 20).
VCam_Overhead: CinemachineFollow + RotationComposer. Offset: (0, 35, 0).
VCam_Overview: fully static.
Switching: `vcam.Priority = int`. Do NOT auto-switch on throw (was removed — was overriding user selection).

## Disc Visualization (Visualization/)
DiscMeshBuilder.cs: static class. Build(DiscModel, segments, domeSegs) → surface-of-revolution mesh.
DiscVisualController.cs: MonoBehaviour (RequireComponent MeshFilter, MeshRenderer). SetDiscModel(DiscModel) rebuilds mesh + resets scale to Vector3.one.
Both LaunchDfisX overloads call discVisualController?.SetDiscModel(initState.directModel).
Material: URP Lit, off-white (0.95,0.95,0.95), smoothness 0.55 — created at runtime, no asset on disk.
DiscVisualController.fallbackModel: used for live-camera throws (no directModel). Should eventually resolve from DiscIndex.

## DiscFoilStamp (Visualization/)
Created by DiscVisualController.Awake(). Sibling component.
Font loaded at runtime from Windows OS fonts via TMP_FontAsset.CreateFontAsset(). Cached statically per name.
Colour = complementary hue (h+0.5) to disc body. Vertex gradient + embossed outline.
5 decoration presets (LineRenderers, ZTest=Disabled, renderQueue=3000/3001).
Font + decoration seeded from manufacturer::moldName hash — deterministic per mold.

## ShotPreviewLine (Visualization/)
Runs synchronous SimulateThrow() on each RequestUpdate(), throttled to maxUpdatesPerSecond=5Hz via Time.unscaledTime.
Call RequestUpdate(ThrowParameters, DiscEnvironment, AeroDebugSettings) from every slider callback.
Call HidePreview() immediately before LaunchDfisX fires.
Seed initial values in ThrowParameterPanelController.Start().

## LandingMarker (Visualization/)
Self-pooling world-space TMP label. LandingMarker.Spawn(worldPos, distanceM).
Floats upward at FloatSpeedMps (0.6 m/s). Holds opacity for first 20% of FadeDurationS (5s) then fades. Billboards to Camera.main.
DiscVisualizer inspector: showLandingMarker toggle, landingMarkerHeightM (default 1.5m).

## MiniMap (UI/)
Software-drawn Texture2D — no extra camera or RenderTexture.
Coordinate mapping: world X (flight) → pixel Y (tee at bottom). World Z (lateral) → pixel X (flipped).
Grid: horizontal lines at gridIntervalFt intervals (default 50ft). Labels at labelIntervalFt (default 50ft).
Trail: reads dfisxTrajectoryLine positions + DiscVisualizer.GetArchivedTrails(). Drawn at trailThicknessPx (default 3px).
Auto-find: FindAnyObjectByType<DiscVisualizer>() in Start() if not wired.

## DistanceMarkerSpawner (Environment/)
Spawns TMP labels along +X axis at configurable intervals. groundOffsetY Inspector field (default 0).
Labels face camera with Quaternion.Euler(0, 90, 0). Ground lines: LineRenderer at groundOffsetY+0.01, running along Z axis. URP/Particles/Unlit vertex-color material.

## DfisX Trail System (DiscVisualizer)
dfisxTrajectoryLine LineRenderer. keepAllThrows toggle. PrepareNewDfisxTrail/AppendDfisxTrailPoint/ClearAllDfisxTrails.
liveThrowTrailColor/liveThrowTrailWidth — per-throw style overrides when throw is from LaunchDfisXFromLive().
Per-disc-type colors: List<DiscTypeTrailStyle> (DiscIndex → color + width) in keepAllThrows mode.
dfisxTrajectoryLine must have vertex-color-capable material. Awake() auto-creates URP/Particles/Unlit if sharedMaterial==null.

## Aero Debug Sliders
Global physics-model tuning constants — NOT per-disc parameters. Applied identically to every disc.
Sliders: cdEdge(0.6), clCavity(45.0), clCamber(1.0), cavityEdgeExposedAreaFactor(1.0), pitchingMomentCavityLiftOffset(0.042), pitchingMomentCamberLiftOffset(0.15).

## ThrowResultPanelController (UI/)
Files: ThrowResultPanel.uxml / ThrowResultPanel.uss / ThrowResultPanelController.cs / DiscPreviewController.cs

Stats shown: Disc Name, Speed, Spin RPM, Spin Factor, Hyzer, Nose Angle, Elevation, Azimuth, Distance, Lateral, Time Aloft, Turn, Fade.
- Panel appears immediately on throw (HandleSimStarted), Distance/Lateral/Time update in real-time via Update().
- HandleThrowFinished finalises Turn, Fade, and corrects Distance/Lateral/Time with BuildStats values.
- History dropdown (top-bar, left: 165px — right of MiniMap) stores last 5 throws. Selecting from dropdown calls PopulateFromHistory on ThrowParameterPanelController.

**Spin sign convention**: `spinRpm = -rawSpin * 60 / (2π)` — negated so RHBH (negative spinRate in physics) displays positive.
**Turn/Fade**: perpDir = 90° CCW from throwDir (points right); turnSide = ±1 from spin sign; fadeM = max(0, maxTurnLateral − signedLatLanding).
**Horizontal distance**: `length(relFinish.xy)` — 2D magnitude, NOT dot product (would undercount with lateral drift).

ThrowContainer primitive snapshot fields (DfisXStructs.cs — Core, no UnityEngine):
`throwSpinRateRadS`, `throwHyzerRad`, `throwPitchRad`, `throwDiscName` — set in DiscFlightSimulator.NewThrow().

## DiscPreviewController (UI/)
Renders a 3D disc preview into a 288×172 RenderTexture, displayed as a `backgroundImage` on the `disc-preview` VisualElement inside stats-panel.

**Isolation**: entire rig (camera + disc copy) placed at y=5000. Camera farClipPlane=3.0m — main scene never bleeds in, no custom layer needed.
**Camera**: local position (0, 0.4, 1.3) — from in front of disc, ~17° above horizontal. Shows hyzer as left/right tilt, nose angle as face pitch. FOV=14°, transparent clear (0,0,0,0).
**Disc copy**: MeshFilter + MeshRenderer child. MirrorDiscAppearance() copies sharedMesh + creates Material instances from sourceDisc (DiscVisualController).
**UpdateOrientation(hyzerDeg, noseAngleDeg)**: uses DiscNormal formula (same as ThrowParameters.DiscNormal): `nx=sin(-p)cos(h), ny=sin(h)cos(p), nz=cos(p)cos(h)`; DfisX→Unity axis swap (x,y,z)→(x,z,y); then `Quaternion.LookRotation(Vector3.forward, discNormalUnity)`.
**RT binding**: one-frame coroutine in ThrowResultPanelController.Start() — `Background.FromRenderTexture(discPreview.PreviewRT)`.
**Known issue**: if RT shows black box instead of transparency, URP camera output alpha may need `allowHDR=false` or a camera output action tweak.

## Pending Tasks
| ID | Status | Notes |
|---|---|---|
| POL-4 | Done | Code complete. Requires scene wiring (see scene-setup.md). |
| POL-5 | Pending | Wind indicator HUD (windsock style). Sock body droops in wind direction; gust indicator animates on gustFactor. |
| POL-6 | Pending | Practice range modes + leaderboard. Modes: Freestyle, DistanceChallenge, AccuracyChallenge. Persists leaderboard.json. |
| POL-7 | Pending | Replay system. discStateArray.ToArray() → raw bytes → file. NativeArray.CopyFrom() on load. Play/pause/scrub coroutine. |
| POL-8 | Pending | Multiple camera angles (basket cam, tee cam, auto-cut). |
| POL-9 | Stretch | Course holes ScriptableObjects (HoleDefinition, CourseDefinition). |
| POL-10 | First pass done | Mesh + foil stamp complete. Spin animation (UV rotation/normal-map) not yet implemented. |
| POL-11 | Pending | Practice range visuals: terrain, treeline, better ground material, HDRI sky. |
| POL-12 | Pending | Collision feedback: capture OnCollisionEnter impulse → counter-force/torque in DfisX body frame. May need externalForceDelta/externalTorqueDelta fields on ThrowContainer. |
