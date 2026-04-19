# DfisX Physics Engine

DfisX is a disc golf flight physics engine originally written in C++. It was ported to C# for Unity in Sessions 1–6.

## Assembly Split

The engine is split across two assemblies to maintain platform portability and enable Burst compilation:

| Assembly | Location | noEngineReferences | Purpose |
|---|---|---|---|
| DfisX.Runtime | Assets/DfisX/Runtime/Core/ | true | Pure math — no UnityEngine dependency |
| DfisX.Unity | Assets/DfisX/Runtime/ | false | Unity-facing code (MonoBehaviours, ScriptableObjects) |

The split was introduced to prevent ambiguity between `UnityEngine.Vector3` and `Unity.Mathematics.float3` in the math files.

## Why float3 Over Vector3
`float3` (Unity.Mathematics) is blittable and can be stored in `NativeArray`. `Vector3` (UnityEngine) is not. Blittable structs are required for the future Burst/Jobs parallelism path — running many simultaneous disc flights for prediction.

## Key Types
- **DiscState** — blittable struct: position, velocity, orientation (float3x3), angular velocity (float3)
- **ThrowContainer** — managed class: owns `NativeArray<DiscState>`, `AeroDebugSettings`, `GaussState`, `FlightStats`
- **DiscModelData** — blittable mirror of `DiscModel` ScriptableObject; converted via `DiscModel.ToBlittable()` at throw start
- **AeroDebugSettings** — six global tuning constants (not per-disc); see [adding-features.md](../development/adding-features.md)

## Coordinate System
The simulation runs entirely in **Z-up** (X forward, Y right, Z up), matching the original C++.
Gravity: `(0, 0, -9.8 * mass)`.
At render time, `DiscFlightSimulator.ToUnitySpace()` and `ToUnityRotation()` convert to Unity's Y-up space.
The disc flies along Unity's +X axis.

## Simulation Loop
`DiscFlightSimulator.StepForUnityFrame(dt)` advances the simulation by Unity's elapsed delta time, sub-stepping at 1kHz internally (`dtS = 0.001`). MAX_STEPS = 30,000 (~30 seconds at 1kHz). Each sub-step calls:
1. `Daero` — aerodynamic lift, drag, and pitching moment forces
2. `Dgyro` — gyroscopic precession torque
3. `Dpropagate` — Euler integration; renormalises orientation

## Disc Models
`DiscModel` is a ScriptableObject (Inspector-facing). Physical dimensions drive all aerodynamic calculations in `Daero.cs` — there are no per-disc "turn" or "fade" constants as in manufacturers' ratings. Aero behaviour emerges purely from geometry.

1,272 disc models are imported from PDGA measurement data via `DiscParamsImporter.cs`.

## Flight Statistics
`FlightStats` on `ThrowContainer` is populated by `DiscFlightSimulator.BuildStats()` after each throw:
- distanceM, maxHeightM (pending POL-4), lateralDriftM (pending POL-4)

## Pending: Burst Migration (INF-4)
Replace `System.Random` in `GaussState` (Daero.cs) with `Unity.Mathematics.Random`. Add `[BurstCompile]` attributes. Add `allowUnsafeCode: true` to DfisX.Runtime.asmdef.
