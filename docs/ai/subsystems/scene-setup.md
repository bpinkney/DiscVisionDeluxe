# Scene Setup Reference — Inspector Values & Wiring

## Scenes
| Scene | Purpose |
|---|---|
| Scenes/DiscDebug.unity | Original manual debug scene |
| Scenes/LiveCapture.unity | FLIR camera + AprilTag live capture |
| Scenes/PracticeRange.unity | Full sim with terrain, markers, Cinemachine |
| Scenes/MainMenu.unity | (Pending — INF-5) |

---

## DiscDebug / PracticeRange — Disc GameObject Hierarchy
```
Disc (GameObject)
  DiscVisualController     component — wire discVisualController on DiscVisualizer; fallbackModel = Destroyer
  MeshFilter + MeshRenderer  auto-created by DiscVisualController
  Rigidbody               — Is Kinematic = true initially
  CapsuleCollider         — Radius 0.106, Height 0.020, Direction Y-Axis, Center (0, 0.010, 0)
  LineRenderer            — KF trail (assign to DiscVisualizer.trajectoryLine)
  DiscVisualizer          — assign all refs (see below)
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
- speedMps: 22
- headingDeg: 0
- launchAngleDeg: 3
- hyzerDeg: 0
- pitchDeg: 0
- spinRpm: -700
- wobble: 0

## Main Camera
Position: (-20, 2, 0), Rotation: (0, 90, 0) — faces +X (disc flight direction)

## Plane / Ground
Scale: (10, 1, 10). PhysicsMaterial: friction 0.4, bounciness 0.15.

## Cinemachine VCam Inspector Values
VCam_Follow: Position/Rotation Control = None. Follow offset: (-followDistance, followHeight, 0). Typical: followDistance=6, followHeight=2.
VCam_Side: CinemachineFollow offset (0, 4, 20). RotationComposer. Tracking Target = Disc.
VCam_Overhead: CinemachineFollow offset (0, 35, 0). RotationComposer. Tracking Target = Disc.
VCam_Overview: Static. No tracking.

## LiveCapture Scene Additions
SpinnakerCameraCapture component (separate GameObject or on Disc).
RawImage UI element for camera preview (assign to SpinnakerCameraCapture.previewTexture target).
LiveDiscTracker component:
- Camera Capture → SpinnakerCameraCapture
- Disc Tag Id → AprilTag ID physically on disc (check [AprilTag] ID=X in console)
- Ground Plane Profile → "ground_plane" (or empty for fallback)
- Detection Timeout Ms → 2000 (update manually — old scenes may have 500)
- On Throw Complete → DiscVisualizer.LaunchDfisX

## Distance Marker Spawner (PracticeRange)
Markers along +X axis. groundOffsetY: raise above terrain. Labels face camera (Quaternion.Euler(0,90,0)).

## ThrowResultPanel (POL-4)
Two GameObjects required:

**ThrowResultPanel (GameObject)**
- UIDocument: Source = ThrowResultPanel.uxml, Sort Order = 2 (above ThrowParameterPanel)
- ThrowResultPanelController: assign discVisualizer, throwParamPanel, discPreview

**DiscPreviewRig (GameObject)**
- DiscPreviewController: assign sourceDisc = DiscVisualController on Disc GameObject
- No transform setup needed — Awake() moves it to y=5000 automatically

ThrowResultPanelController.discPreview → DiscPreviewRig's DiscPreviewController.

## DiscModel Assets Location
Assets/Resources/DiscModels/ — DiscModelLibrary.asset + 1,272 individual .asset files (post SIM-4).
Named presets (original 11): Innova_Destroyer, etc.
