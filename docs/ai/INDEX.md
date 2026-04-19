# DiscVisionDeluxe — AI Context Index
Last updated: Session 13 (2026-04-14)

## What This Project Is
Unity 6 disc golf flight simulator. DfisX C++ physics engine ported to C#.
Two input paths: (A) FLIR camera + AprilTag → KF → DfisX, (B) manual params from UI sliders → DfisX.
Single output: `DiscVisualizer.LaunchDfisX(DiscInitState)`. Never bypass this entrypoint.

## Project Tracks
- Track A (Priority 1): Hardware — FLIR Spinnaker + AprilTag → live throw detection → sim
- Track B (Priority 2/3): Sim experience — practice range, UI panels, disc database, polish

## Unity & Render Pipeline
Unity 6000.4.0f1. URP (com.unity.render-pipelines.universal 17.4.0). Cinemachine 3.1.6.

## Key Namespaces and Files
| Namespace | Key Files |
|---|---|
| DfisX (Core, noEngineRef=true) | DfisXStructs.cs, Daero.cs, Dgyro.cs, Dpropagate.cs |
| DfisX (Unity, noEngineRef=false) | DiscFlightSimulator.cs, DiscModel.cs, DiscModelLibrary.cs, DiscModelPresets.cs, ThrowParameters.cs |
| DiscVisionDeluxe | DiscState.cs, DiscKalmanFilter.cs, DiscSimulator.cs, DiscVisualizer.cs, CsvLogReader.cs, DiscThrowDebugger.cs |
| DiscVisionDeluxe.Camera | SpinnakerCAPI.cs, SpinnakerCameraCapture.cs, AprilTagDetector.cs, CameraCalibration.cs, GroundPlaneCalibration.cs, GroundPlaneCalibrator.cs, LiveDiscTracker.cs, FollowFlightCamera.cs |
| DiscVisionDeluxe.UI | ThrowParameterPanelController.cs, MiniMap.cs, WindIndicator.cs, ThrowResultPanelController.cs, LeaderboardPanelController.cs |
| DiscVisionDeluxe.Visualization | DiscMeshBuilder.cs, DiscVisualController.cs, ShotPreviewLine.cs, LandingMarker.cs |

## Flight Pipeline
CSV/Camera → DiscSimulator/LiveDiscTracker → DiscKalmanFilter
→ onThrowComplete(DiscInitState) → DiscVisualizer.LaunchDfisX()
→ DiscFlightSimulator.NewThrow() → StepForUnityFrame() → OnThrowFinished → Rigidbody

## Coordinate System
DfisX internal: Z-up (X forward, Y right). Unity Y-up conversion at render time via ToUnitySpace().
Disc flies along Unity +X. Camera must face +X. All scene layout uses +X as flight direction.

## Assembly Rules
- DfisX.Runtime (Core/): noEngineReferences=true — no UnityEngine. Only Unity.Mathematics float3.
- DfisX.Unity (Runtime/): noEngineReferences=false — full UnityEngine access.
- DiscVisionDeluxe → may reference DfisX.Unity + DfisX.Runtime. Reverse is forbidden.
- DiscVisionDeluxe.Camera: Editor + WindowsStandalone64 platforms only (Spinnaker SDK requirement).
- DiscVisionDeluxe.UI: refs DiscVisionDeluxe, DiscVisionDeluxe.Camera, DfisX.Unity, DfisX.Runtime.

## Active Tasks (summary — see tasks/active.md for details)
| ID | Task | Track |
|---|---|---|
| INF-4 | Burst migration (System.Random → Unity.Mathematics.Random, [BurstCompile]) | Infra |
| INF-5 | Main menu scene + AppSettings.cs persistence | Infra |
| CAM-ISSUE-1 | Redo camera calibration (current RMS 3.4274 — target <1.0) | Track A |
| CAM-ISSUE-3 | Real throws not detected without establishment | Track A |
| POL-4 | Throw result statistics panel | Track B |
| POL-5 | Wind indicator HUD (windsock style) | Track B |
| POL-6 | Practice range modes + leaderboard | Track B |
| POL-7 | Replay system (discStateArray serialization) | Track B |
| POL-8 | Multiple camera angles (basket cam, tee cam) | Track B |
| POL-10 | Disc render — spin animation pending (mesh + foil done) | Track B |
| POL-11 | Practice range visual improvements | Track B |
| POL-12 | Collision feedback into flight model | Track B |

## Context Files
| File | When to Read |
|---|---|
| subsystems/dfisX.md | DfisX physics, blittable structs, asmdef, Burst work |
| subsystems/kf-pipeline.md | Kalman filter, CSV input, DiscSimulator, KF params |
| subsystems/camera.md | FLIR, AprilTag, calibration, LiveDiscTracker, open issues |
| subsystems/ui-viz.md | UI Toolkit panels, Cinemachine cameras, trails, minimap, disc viz |
| subsystems/scene-setup.md | Scene hierarchy, Inspector values, wiring |
| decisions/architecture.md | Before any architectural decision or when checking constraints |
| decisions/gotchas.md | Debugging native plugins, Emgu CV, or Unity quirks |
| tasks/active.md | Task details, mid-session state, design notes for pending work |
| tasks/archive.md | Completed task history and full session log |

## Session Log
| Session | Work Done |
|---|---|
| 1-6 | DfisX port complete (Phases 1-5). All physics engine C# files working. Disc flying confirmed. |
| 7 | INF-1/2/3/6: asmdef, manifest.json (URP/CM/Burst), created next_steps.md. |
| 8 | SIM-1 UI panel (UXML/USS). SIM-2 distance markers (+X). SIM-3 Cinemachine 3 cameras. |
| 9 | Camera auto-switch fix. Ground collision fix. Grass plane URP material. |
| 10 | CAM-1: P/Invoke Spinnaker, 12 DLLs, RGB24 preview fix. Camera feed confirmed. |
| 11 | CAM-2: Emgu CV 4.12 AprilTag. CAM-3: calibration EditorWindow. |
| 12 | CAM-4: ground plane calibration. CAM-5 PoC: LiveDiscTracker, end-to-end throw detected. |
| 13 | SIM-4: 1,272 discs imported, multi-filter disc selection UI, recents list. |
