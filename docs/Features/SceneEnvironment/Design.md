# Scene Environment — Design

## Scenes
| Scene | Purpose |
|---|---|
| Assets/Scenes/DiscDebug.unity | Manual debug scene — DiscThrowDebugger + CSV KF |
| Assets/Scenes/LiveCapture.unity | FLIR camera + AprilTag live capture |
| Assets/Scenes/PracticeRange.unity | Full sim with terrain, distance markers, Cinemachine |
| Assets/Scenes/MainMenu.unity | Pending — INF-5 |

## Disc GameObject Hierarchy (DiscDebug / PracticeRange)
```
Disc (GameObject)
  DiscVisualController     — wire discVisualController on DiscVisualizer; fallbackModel = Destroyer
  MeshFilter + MeshRenderer  auto-created by DiscVisualController
  Rigidbody               — Is Kinematic = true initially
  CapsuleCollider         — Radius 0.106, Height 0.020, Direction Y-Axis, Center (0, 0.010, 0)
  LineRenderer            — KF trail (assign to DiscVisualizer.trajectoryLine)
  DiscVisualizer          — assign all refs
  DfisxTrail (child GameObject)
    LineRenderer          — DfisX trail (assign to DiscVisualizer.dfisxTrajectoryLine)
```

## DiscVisualizer Inspector Fields
- discTransform: Disc transform
- discRigidbody: Rigidbody on same object
- trajectoryLine: KF LineRenderer
- dfisxTrajectoryLine: DfisX LineRenderer
- discModelLibrary: DiscModelLibrary asset
- discVisualController: DiscVisualController component
- showLandingMarker: true
- landingMarkerHeightM: 1.5
- spinTransferFactor: 0.3

## DiscThrowDebugger Inspector (default debug values)
- discIndex: DRIVER
- speedMps: 22, headingDeg: 0, launchAngleDeg: 3
- hyzerDeg: 0, pitchDeg: 0, spinRpm: -700, wobble: 0

## Ground / Plane
Scale: (10, 1, 10). PhysicsMaterial: friction 0.4, bounciness 0.15.

## DistanceMarkerSpawner (Environment/)
Spawns TMP labels along +X axis at configurable intervals. `groundOffsetY` Inspector field (default 0).
Labels face camera with `Quaternion.Euler(0, 90, 0)`. Ground lines: LineRenderer at `groundOffsetY+0.01`, running along Z axis. URP/Particles/Unlit vertex-color material.

## LiveCapture Scene Additions
- SpinnakerCameraCapture component (separate GameObject or on Disc)
- RawImage UI element for camera preview (assign to `SpinnakerCameraCapture.previewTexture` target)
- LiveDiscTracker component:
  - Camera Capture → SpinnakerCameraCapture
  - Disc Tag Id → AprilTag ID physically on disc
  - Ground Plane Profile → "ground_plane" (or empty for fallback)
  - Detection Timeout Ms → **2000** (old scenes may have 500 — update manually)
  - On Throw Complete → DiscVisualizer.LaunchDfisX

## DiscModel Assets Location
`Assets/Resources/DiscModels/` — DiscModelLibrary.asset + 1,272 individual .asset files.
Named presets (original 11): Innova_Destroyer, etc.
