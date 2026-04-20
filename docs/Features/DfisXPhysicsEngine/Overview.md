# DfisX Physics Engine — Overview

## Purpose
DfisX is a disc golf flight physics engine originally written in C++, ported to C# for Unity in Sessions 1–6. It calculates aerodynamic forces, gyroscopic precession, and Euler integration at 1kHz sub-steps to produce realistic disc flight trajectories.

## Requirements
- Simulate full disc golf flight from initial throw state to ground contact
- Support 1,272 real PDGA disc models with physically-measured dimensions
- Enable future parallelism: many simultaneous flights for prediction/replay
- Run on main thread today; Burst/Jobs-ready by design

## Architecture
The engine splits into two assemblies:

| Assembly | Location | noEngineReferences | Purpose |
|---|---|---|---|
| DfisX.Runtime | Assets/DfisX/Runtime/Core/ | true | Pure math — no UnityEngine dependency |
| DfisX.Unity | Assets/DfisX/Runtime/ | false | Unity-facing code (MonoBehaviours, ScriptableObjects) |

The split prevents ambiguity between `UnityEngine.Vector3` and `Unity.Mathematics.float3` in math files, and is required for Burst compilation.

All aerodynamic behaviour emerges from disc geometry — there are no per-disc "turn", "fade", or "glide" constants as in manufacturer ratings. `Daero.cs` derives all forces from physical dimensions in `DiscModelData`.

## Place in the Project
DfisX is the foundation the entire project rests on. Every input path (camera, UI sliders, CSV, future replay) eventually calls `DiscFlightSimulator.NewThrow()` via the `DiscVisualizer.LaunchDfisX()` entrypoint. No other code may call `NewThrow()` directly.
