# Development Guidelines

## Non-Negotiable Invariants

1. **Every input pathway terminates at `DiscVisualizer.LaunchDfisX(DiscInitState)`.** New throw sources (replay, network, AI opponent) must produce a `DiscInitState` and call this method. Never call `DiscFlightSimulator.NewThrow()` directly.

2. **No Unity API calls from background threads.** The only cross-thread mechanism is `ConcurrentQueue`. All `UnityEvent` callbacks fire on the main thread.

3. **Camera pipeline must output DfisX Z-up frame** (X forward, Y right, Z up). Reference `CsvLogReader.ParseLine()` for the angular extraction formulas.

4. **New physics-facing structs must be blittable.** No managed references, no strings. Required for `NativeArray` and future Burst compilation.

5. **`DiscVisionDeluxe` scripts may reference `DfisX.Unity` and `DfisX.Runtime`. The reverse is forbidden.**

6. **Search existing files before creating a new one.** Check the file maps in the feature docs and assembly reference. Extend existing files where possible.

## Before Creating a New .cs File

1. Check [AssemblyReference.md](AssemblyReference.md) — which assembly should it live in?
2. Check the relevant feature's Design.md — does something similar already exist?
3. Is the type physics-facing? If yes, it must be blittable (no class fields, no string fields).
4. Does it need Unity APIs? If yes, it belongs in `DfisX.Unity` or `DiscVisionDeluxe.*`, not `DfisX.Runtime`.

## Wiring a New Throw Input

1. Produce a `DiscInitState` with all fields populated (position, velocity, orientation, spin, disc model)
2. Set `directModel` on `DiscInitState` to the selected `DiscModel` asset (bypasses index lookup)
3. Call `discVisualizer.LaunchDfisX(initState)` — this is the only valid entrypoint

## UI

All new panels and HUDs use UI Toolkit (UXML/USS). Do not use uGUI Canvas for new UI. The exception is `DiscThrowDebugger`, which uses `OnGUI` and should not be changed.

## Disc Selection

`ThrowParameterPanelController._selectedModel` is the single authoritative `DiscModel` reference. It must flow through to `DiscInitState.directModel`. Never rely solely on `DiscIndex` — the index only maps to 12 type/stability slots; the full 1,272-disc library requires direct model references.

## Aero Debug Sliders

The six aero debug sliders (`cdEdge`, `clCavity`, etc.) are **global physics-model constants** — they apply to every disc equally. Do not save them per-disc or expose them as disc properties.

## Adding a New Feature

Follow these steps in order when starting any new feature (pending or new).

**1. Pick the right assembly**
Use [AssemblyReference.md](AssemblyReference.md) to decide where your scripts live:
- Pure math, no Unity API → `DfisX.Runtime`
- Unity types, physics-facing → `DfisX.Unity`
- General sim/game logic, Cinemachine → `DiscVisionDeluxe` (default for most new features)
- UI Toolkit panels/HUDs → `DiscVisionDeluxe.UI`
- FLIR/AprilTag/camera hardware → `DiscVisionDeluxe.Camera` (Editor + Win64 only)
- Editor tooling → `DfisX.Editor` or `DiscVisionDeluxe.Camera.Editor`

**2. Create the feature doc folder**
```
docs/Features/<FeatureName>/
  Entry.md      ← required; status, blocker, key script paths, links to Design/Tasks
  Design.md     ← required before writing non-trivial code; architecture, data flow, gotchas
  Tasks.md      ← active and backlog tasks
  CHANGELOG.md  ← append a line when the feature ships or a significant change lands
```
Add a row to the table in [docs/README.md](../README.md) with status and Entry link.

**3. Write Design.md before any code**
Minimum content: what the feature does, which existing scripts it touches or extends, how data flows in/out, and any known constraints or gotchas. One page is enough for small features.

**4. Search before creating files**
Run a project-wide search for the concept before creating a new `.cs` file. Many features have stub scripts already (e.g. `WindIndicator.cs`). Check the relevant feature's Design.md file map too.

**5. Wire the throw input correctly (if applicable)**
Any new throw source must produce a `DiscInitState` and call `discVisualizer.LaunchDfisX(initState)`. See [Wiring a New Throw Input](#wiring-a-new-throw-input) below.

**6. UI: always UI Toolkit**
New panels and HUDs use UXML + USS. No uGUI Canvas. Place UXML/USS under `Assets/UI/`. Controller scripts go in `Assets/DiscVisionDeluxe/UI/` and compile into `DiscVisionDeluxe.UI`.

**7. Update Entry.md when status changes**
Flip the status emoji (`⏳ Pending` → `🔧 In Progress` → `✅ Complete`) and clear the active task line when done.

---

## Adding a New Camera Mode

Use `vcam.Priority = int` to switch active cameras. Use Cinemachine 3 (`CinemachineCamera`), not CM2 `CinemachineVirtualCamera`. `FollowFlightCamera.cs` owns all mode logic; extend it rather than creating a new camera controller.
