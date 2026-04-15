# Unity Project Setup

## Requirements
- Unity 6000.4.0f1 (Unity 6)
- Visual Studio 2022 or JetBrains Rider

## Package Versions
| Package | Version | How to Install |
|---|---|---|
| Universal Render Pipeline | 17.4.0 | Package Manager UI (not manifest.json) |
| Cinemachine | 3.1.6 | Package Manager |
| Burst | 1.8.17 | Package Manager |
| Mathematics | (bundled) | via manifest.json |
| Collections | (bundled) | via manifest.json |

> Note: `com.unity.jobs` was removed due to a version conflict with Unity 6. It is not needed.

## URP Configuration (one-time, manual steps)
After opening the project for the first time:

1. `Edit > Project Settings > Graphics > Scriptable Render Pipeline Settings` — create a URP Asset and assign it
2. `Edit > Rendering > Render Pipeline Converter` — upgrade existing Built-in materials
3. Open the DiscDebug scene and confirm no pink materials before proceeding

## Scenes
| Scene | Purpose |
|---|---|
| Assets/Scenes/DiscDebug.unity | Manual throw debug (DiscThrowDebugger + CSV KF) |
| Assets/Scenes/LiveCapture.unity | Live FLIR camera capture + AprilTag detection |
| Assets/Scenes/PracticeRange.unity | Full practice environment with terrain and Cinemachine |
| Assets/Scenes/MainMenu.unity | Scene selection (pending — INF-5) |

## Assembly Definitions
The project uses four custom asmdefs. See [Assembly Reference](../architecture/assembly-reference.md) for the full dependency graph.

## Native Plugins
- `Assets/Plugins/Spinnaker/` — FLIR Spinnaker SDK (12 DLLs). See [Camera Hardware Setup](camera-hardware.md).
- `Assets/Plugins/EmguCV/` — Emgu CV native runtime (cvextern.dll + libusb-1.0.dll).

## Streaming Assets
- `Assets/StreamingAssets/CameraCalibration/` — lens calibration and ground plane calibration JSON files.
