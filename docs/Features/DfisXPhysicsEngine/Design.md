# DfisX Physics Engine — Design

## Assembly Structure
```
Assets/DfisX/Runtime/Core/      → DfisX.Runtime.asmdef (noEngineReferences: true)
Assets/DfisX/Runtime/           → DfisX.Unity.asmdef  (noEngineReferences: false)
Assets/DfisX/Editor/            → Editor-only (no asmdef needed)
```

## File Map

### Core (DfisX.Runtime, noEngineReferences=true)
| File | Purpose |
|---|---|
| DfisXStructs.cs | DiscState, ForcesState, ThrowContainer, DiscModelData, AeroDebugSettings. GaussState field on ThrowContainer. |
| Daero.cs | Aero forces + gust model. GaussState class at bottom. TODO: swap System.Random → Unity.Mathematics.Random for Burst (INF-4). |
| Dgyro.cs | Gyroscopic precession. Dead mode==true branch dropped. |
| Dpropagate.cs | Euler integration + gravity. Renormalises orientation. |

### Unity-facing (DfisX.Unity)
| File | Purpose |
|---|---|
| DiscFlightSimulator.cs | Main simulation loop. NewThrow(), StepForUnityFrame(), SimulateThrow(). ToUnitySpace()/ToUnityRotation() for Y-up conversion. MAX_STEPS=30000. |
| DiscModel.cs | ScriptableObject. ToBlittable() → DiscModelData. |
| DiscModelLibrary.cs | ScriptableObject; holds disc assets + DiscLayoutIndex mapping; discs List. |
| DiscModelPresets.cs | 11 reference disc factory methods. |
| ThrowParameters.cs | KF→DfisX handoff. [NonSerialized] directModel field. Parse() reads stdout format. DiscNormal() converts hyzer+pitch→Z unit vector. |

### Editor
| File | Purpose |
|---|---|
| DiscModelPresetsEditor.cs | Creates all 11 preset disc assets in one click. Auto-populates DiscModelLibrary. |
| DiscParamsImporter.cs | DfisX → Import CSV Disc Database. Reads disc_params_pdga_final.csv. InvariantCulture float parsing. Creates one DiscModel .asset per row. |

## Key Types

### DiscState (blittable)
Position, velocity, orientation (float3x3), angular velocity (float3). Lives in `NativeArray<DiscState>`.

### ThrowContainer
Managed class. Owns `NativeArray<DiscState> discStateArray` (resized to MAX_STEPS). Holds `AeroDebugSettings` and `GaussState`. `FlightStats` populated by `BuildStats()` in DiscFlightSimulator. Snapshot fields for ThrowResultPanel: `throwSpinRateRadS`, `throwHyzerRad`, `throwPitchRad`, `throwDiscName` — set in `NewThrow()`.

### DiscModelData (blittable)
Mirror of DiscModel ScriptableObject. Fields: mass, radius, rimWidth, thickness, rimDepth, rimCamberHeight, domeHeight, rimCamberShape (int: 0=Flat, 1=Concave, 2=Convex). NONE from CSV → 0 (Flat degradation).

### AeroDebugSettings (blittable)
cdEdge (0.6), clCavity (45.0), clCamber (1.0), cavityEdgeExposedAreaFactor (1.0), pitchingMomentCavityLiftOffset (0.042), pitchingMomentCamberLiftOffset (0.15). Global tuning constants — NOT per-disc.

## Coordinate System
Z-up internally (X forward, Y right, Z up). Gravity = `(0, 0, -9.8 * mass)`.
Unity Y-up conversion at render time via `ToUnitySpace()` / `ToUnityRotation()`.

## Simulation Step
`StepForUnityFrame()` advances sim by elapsed Unity delta time. Internally sub-steps at 1kHz (dtS = 0.001). MAX_STEPS = 30,000 (~30s). Each sub-step: Daero → Dgyro → Dpropagate.

## Pending: Burst Migration (INF-4)
1. Replace `System.Random` in `GaussState` (Daero.cs) with `Unity.Mathematics.Random`
2. Add `[BurstCompile]` to Dpropagate, Dgyro, Daero step methods
3. Add `allowUnsafeCode: true` to DfisX.Runtime.asmdef
4. Verify simulation output unchanged vs pre-Burst
