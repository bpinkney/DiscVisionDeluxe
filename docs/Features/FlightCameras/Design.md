# Flight Cameras — Design

## Architecture
Cinemachine 3 (CM3) API. Unity 6 uses `CinemachineCamera` — never `CinemachineVirtualCamera` (CM2). `FollowFlightCamera.cs` owns all mode logic. Camera switching: `vcam.Priority = int`.

Do NOT auto-switch on throw — this was removed (was overriding user's camera selection). User selects mode via `ThrowParameterPanelController` camera dropdown.

## Camera Modes (FollowFlightCamera.cs)
| Mode | Setup | Offset |
|---|---|---|
| VCam_Follow | Position/Rotation Control = None. Manually positioned every frame. | (-followDistance, followHeight, 0). Typical: followDistance=6, followHeight=2 |
| VCam_Side | CinemachineFollow + RotationComposer. Tracking Target = Disc. | (0, 4, 20) |
| VCam_Overhead | CinemachineFollow + RotationComposer. Tracking Target = Disc. | (0, 35, 0) |
| VCam_Overview | Fully static. No tracking. | — |

## Camera Mode Dropdown
`CameraMode` enum order does NOT match UI choices order. Always use `ModeToIndex()` / `IndexToMode()` — never cast directly. See UIThrowPanel/Bugs.md.

## Static Main Camera
Position: (-20, 2, 0), Rotation: (0, 90, 0) — faces +X (disc flight direction).

## Files
- `Assets/DiscVisionDeluxe/FlightCameras/FollowFlightCamera.cs`
