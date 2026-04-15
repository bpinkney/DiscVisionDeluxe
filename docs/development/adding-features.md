# Adding Features — Conventions and Rules

## Non-Negotiable Invariants

These rules must not be violated regardless of how a feature is structured:

1. **Every input pathway terminates at `DiscVisualizer.LaunchDfisX(DiscInitState)`.** New throw sources (replay, network, AI opponent) must produce a `DiscInitState` and call this method. Never call `DiscFlightSimulator.NewThrow()` directly.

2. **No Unity API calls from background threads.** The only cross-thread mechanism is `ConcurrentQueue`. All `UnityEvent` callbacks fire on the main thread.

3. **Camera pipeline must output DfisX Z-up frame** (X forward, Y right, Z up). Reference `CsvLogReader.ParseLine()` for the angular extraction formulas.

4. **New physics-facing structs must be blittable.** No managed references, no strings. This is required for `NativeArray` and future Burst compilation.

5. **`DiscVisionDeluxe` scripts may reference `DfisX.Unity` and `DfisX.Runtime`. The reverse is forbidden.**

6. **Search existing files before creating a new one.** See the file maps in the architecture docs. Extend existing files where possible.

## Before Creating a New .cs File

1. Check the [Assembly Reference](../architecture/assembly-reference.md) — which assembly should it live in?
2. Check the key files list in the relevant architecture doc — does something similar already exist?
3. Is the type physics-facing? If yes, it must be blittable (no class fields, no string fields).
4. Does it need Unity APIs? If yes, it belongs in `DfisX.Unity` or `DiscVisionDeluxe.*`, not `DfisX.Runtime`.

## Wiring a New Throw Input

To add a new throw source:
1. Produce a `DiscInitState` with all fields populated (position, velocity, orientation, spin, disc model)
2. Set `directModel` on `DiscInitState` to the selected `DiscModel` asset (bypasses index lookup)
3. Call `discVisualizer.LaunchDfisX(initState)` — this is the only valid entrypoint

## UI Toolkit vs uGUI

All new panels and HUDs use UI Toolkit (UXML/USS). Do not use uGUI Canvas for new UI. The exception is `DiscThrowDebugger`, which uses `OnGUI` and should not be changed.

## Disc Selection

`ThrowParameterPanelController._selectedModel` is the single authoritative `DiscModel` reference. It must flow through to `DiscInitState.directModel`. Never rely solely on `DiscIndex` — the index only maps to 12 type/stability slots; the full 1,272-disc library requires direct model references.

## Aero Debug Sliders

The six aero debug sliders (`cdEdge`, `clCavity`, etc.) are **global physics-model constants** — they apply to every disc equally and should never be described as per-disc parameters. Do not save them per-disc or expose them as disc properties.

## Adding a New Camera Mode (Cinemachine)

Use `vcam.Priority = int` to switch active cameras. Cinemachine 3 (`CinemachineCamera`) must be used — not CM2 `CinemachineVirtualCamera`. `FollowFlightCamera.cs` owns all mode logic; extend it rather than creating a new camera controller.
