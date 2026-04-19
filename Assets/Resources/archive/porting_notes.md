# DfisX Unity Port — Porting Notes

## !! IMPORTANT — CHECK PROJECT FILES FIRST !!
Before generating any new C# file, always search project knowledge for existing
.cs files in the same namespace or with similar names. Never shadow or duplicate
an existing file. Extend existing files where possible.

Key existing C# files in namespace DiscVisionDeluxe:
- DiscState.cs        — KFState, KFMeasurement, DiscInitState, DiscIndex, PosVelVarState
- DiscVisualizer.cs   — KF visualiser + DfisX flight driver + Rigidbody handoff
- DiscSimulator.cs    — drives KF from CSV; fires onThrowComplete(DiscInitState)
- DiscKalmanFilter.cs — full KF implementation
- CsvLogReader.cs     — reads csvlog.csv
- DiscThrowDebugger.cs — manual throw debug tool (added this project)

Key C# files in namespace DfisX:
- DfisXStructs.cs, Daero.cs, Dgyro.cs, Dpropagate.cs — physics engine
- DiscFlightSimulator.cs, DiscModel.cs, DiscModelLibrary.cs, DiscModelPresets.cs, ThrowParameters.cs

---

## Project Overview
Porting DfisX (C++ disc golf flight physics) from Unreal to Unity.

Full pipeline:
```
csvlog.csv
  → DiscSimulator → DiscKalmanFilter
  → onThrowComplete(DiscInitState)
  → DiscVisualizer.LaunchDfisX()       ← called by KF pipeline OR DiscThrowDebugger
  → DiscFlightSimulator.NewThrow()
  → StepForUnityFrame() each Update()
  → OnThrowFinished → Rigidbody handoff
```

---

## Folder Structure
```
Assets/
  DfisX/
    Editor/
      DiscModelPresetsEditor.cs    ← Editor-only, no asmdef needed
    Runtime/
      DfisX.Unity.asmdef           ← noEngineReferences: false, refs DfisX.Runtime
      DiscFlightSimulator.cs
      DiscModel.cs
      DiscModelLibrary.cs
      DiscModelPresets.cs
      ThrowParameters.cs
      Core/
        DfisX.Runtime.asmdef       ← noEngineReferences: true
        DfisXStructs.cs
        Daero.cs
        Dgyro.cs
        Dpropagate.cs
  DiscVisionDeluxe/
    DiscState.cs
    DiscKalmanFilter.cs
    DiscSimulator.cs
    DiscVisualizer.cs
    DiscThrowDebugger.cs
    CsvLogReader.cs
  Resources/
    DiscModels/
      DiscModelLibrary.asset
      Innova_Destroyer.asset
      ... (11 preset discs)
  Scenes/
    DiscDebug.unity
```

## asmdef Configuration

### DfisX.Runtime.asmdef (Assets/DfisX/Runtime/Core/)
```json
{
    "name": "DfisX.Runtime",
    "references": ["Unity.Mathematics", "Unity.Collections"],
    "autoReferenced": false,
    "noEngineReferences": true
}
```

### DfisX.Unity.asmdef (Assets/DfisX/Runtime/)
```json
{
    "name": "DfisX.Unity",
    "references": ["DfisX.Runtime", "Unity.Mathematics", "Unity.Collections"],
    "autoReferenced": true,
    "noEngineReferences": false
}
```

Note: DiscVisionDeluxe scripts have no asmdef — they compile against everything by default
and see both DfisX.Runtime and DfisX.Unity automatically.

---

## Architecture Decision Log

### Why C# port over native plugin (.dll)
- Daero.cpp is pure math — ports line-for-line.
- Native plugin loses Unity debugger, P/Invoke marshalling complexity, gaussrand() static state breaks multithreaded simulation.
- C# allows Unity Jobs parallelism later.

### Why blittable structs over classes for DiscState / ForcesState
- Target: many simultaneous disc flights.
- Blittable structs → NativeArray<DiscState> → IJobParallelFor → Burst SIMD.
- ThrowContainer stays managed class — owns NativeArray and handles lifetime/resize.

### Why Unity.Mathematics (float3) over UnityEngine.Vector3
- float3 is blittable; Vector3 is not safe inside NativeArray.
- Burst understands float3 natively and auto-vectorises.
- noEngineReferences: true on DfisX.Runtime.asmdef prevents Vector3/float3 ambiguity in math files.

### DiscModel as ScriptableObject + DiscModelData blittable mirror
- DiscModel ScriptableObject = authored/Inspector-facing.
- DiscModelData struct = blittable mirror, converted once at throw-start via DiscModel.ToBlittable().
- ToBlittable() inlines the conversion directly — no FromScriptableObject() dependency.

### AeroDebugSettings
- Mirrors gv_aero_debug0..5 from Daero.cpp.
- Blittable struct on ThrowContainer; expose via [SerializeField] floats on a MonoBehaviour.

### disc_params.hpp → DiscModelPresets.cs + DiscModelLibrary
- 11 reference discs in DiscModelPresets.cs as factory methods.
- DiscModelLibrary ScriptableObject holds disc assets + DiscLayoutIndex mapping.
- DiscModelPresetsEditor.cs adds menu items to auto-create all preset disc assets in one click.

### ThrowParameters.cs
- Handoff struct from dvd_DvisEst KF output.
- Parse() reads posx:0.115,posy:-0.516,... stdout format.
- DiscNormal() converts hyzer + pitch → Z unit vector.

### gaussrand() → GaussState class (Phase 2)
- C++ static locals were a threading hazard. GaussState managed class, one per ThrowContainer.
- Future Burst path: replace System.Random with Unity.Mathematics.Random (TODO in Daero.cs).

### double → float throughout (Phase 2)
- Acceptable precision at 1kHz sim step. Required for Burst.

### Coordinate system: Z-up internally (Phase 3/4)
- Sim runs Z-up matching C++. Gravity is (0, 0, -9.8*mass).
- Unity Y-up conversion at render time via DiscFlightSimulator.ToUnitySpace() / ToUnityRotation().
- DiscInitState and DfisX ThrowParameters use the same Z-up frame — no remapping needed at handoff.
- Camera should face along positive X axis to see disc fly correctly.

### Eigen rotation matrix transpose (Phase 3)
- C# float3x3 is row-major. Matrices built directly in correct form — no transpose needed.

### Dgyro dead code dropped (Phase 3)
- mode == true branch is entirely commented-out. Dropped.
- Redundant make_unit_vector at end of step_Dgyro dropped (Dpropagate renormalises).

### consume_Dcollision inert block omitted (Phase 4)
- overwrite_states_ignore_forces_torques block guarded by = false — entirely inert.
- Only active impulse-torque branch (lines 320-376) ported.

### Landing handoff: Rigidbody (Phase 5)
- DfisX owns flight phase. At z<=0, OnThrowFinished fires, DiscVisualizer hands off to Rigidbody.
- spinTransferFactor (default 0.3) scales spin → Rigidbody angular velocity. Tune against real footage.
- PhysX handles bounce/roll/settle via PhysicsMaterial on terrain.

### Phase 5: extend existing DiscVisualizer.cs, not a new file
- LaunchDfisX() made public so DiscThrowDebugger can call it directly.
- DfisX trail system added (separate from KF trail): dfisxTrajectoryLine, keepAllThrows toggle,
  PrepareNewDfisxTrail(), AppendDfisxTrailPoint(), ClearAllDfisxTrails().

### asmdef split to resolve Vector3/float3 ambiguity (compilation)
- DfisX.Runtime (Core/) has noEngineReferences: true — pure math, no UnityEngine.
- DfisX.Unity (Runtime/) has noEngineReferences: false — Unity-facing files with UnityEngine access.
- DfisXStructs.cs has no 'using UnityEngine' — all types are float3/Unity.Mathematics.
- DiscModel.cs, DiscModelLibrary.cs, DiscModelPresets.cs, DiscFlightSimulator.cs live in DfisX.Unity.
- DiscFlightSimulator.cs needs 'using float3 = Unity.Mathematics.float3' alias at top.

---

## Required Unity Packages
| Package | Name |
|---|---|
| Mathematics | com.unity.mathematics |
| Collections | com.unity.collections |
| Burst | com.unity.burst |

---

## File Map

### DfisX.Runtime assembly — Assets/DfisX/Runtime/Core/
| File | Source | Notes |
|---|---|---|
| DfisXStructs.cs | DfisX.hpp | No UnityEngine. No FromScriptableObject. GaussState field on ThrowContainer. |
| Daero.cs | Daero.cpp | Aero forces + gust model. GaussState class at bottom. |
| Dgyro.cs | Dgyro.cpp | Gyroscopic precession. |
| Dpropagate.cs | Dpropagate.cpp | Euler integration + gravity. |

### DfisX.Unity assembly — Assets/DfisX/Runtime/
| File | Source | Notes |
|---|---|---|
| DiscModel.cs | DfisX.hpp Disc_Model | ScriptableObject. ToBlittable() inlined. |
| DiscModelLibrary.cs | disc_params.hpp | Disc asset library + DiscLayoutIndex mapping. |
| DiscModelPresets.cs | disc_params.hpp | 11 reference disc factory methods. |
| ThrowParameters.cs | dvd_DvisEst KF output | KF → DfisX handoff. float3 alias needed. |
| DiscFlightSimulator.cs | DfisX.cpp | Main loop. float3/float3x3 aliases at top. |

### Editor — Assets/DfisX/Editor/
| File | Notes |
|---|---|
| DiscModelPresetsEditor.cs | Creates all preset disc assets in one click. Auto-populates DiscModelLibrary. |

### DiscVisionDeluxe — Assets/DiscVisionDeluxe/
| File | Notes |
|---|---|
| DiscVisualizer.cs | LaunchDfisX() is public. DfisX trail system added. KF trail unchanged. |
| DiscThrowDebugger.cs | Manual throw debug tool. Inspector params + runtime OnGUI buttons. |

---

## DiscDebug Scene Setup
```
DiscDebug (scene)
  Main Camera         — position (-20, 2, 0), rotation (0, 90, 0) to face disc flight direction
  Directional Light
  Plane               — scale (10,1,10), PhysicsMaterial (friction 0.4, bounciness 0.15)
  Disc (GameObject)
    Cylinder          — scale (0.21, 0.015, 0.21), placeholder mesh
    Rigidbody         — Is Kinematic = true
    CapsuleCollider
    LineRenderer      — KF trail (assign to DiscVisualizer.trajectoryLine)
    DiscVisualizer    — assign all refs, discModelLibrary, discRigidbody
    DfisxTrail (child GameObject)
      LineRenderer    — DfisX trail (assign to DiscVisualizer.dfisxTrajectoryLine)
  DiscThrowDebugger   — assign discVisualizer ref
```

### Key Inspector values for DiscThrowDebugger
- discIndex: DRIVER
- speedMps: 22
- headingDeg: 0
- launchAngleDeg: 3
- hyzerDeg: 0
- pitchDeg: 0
- spinRpm: -700
- wobble: 0

---

## Phase Status
- Phase 1 — Data structures: ✅ Complete
- Phase 2 — Daero: ✅ Complete
- Phase 3 — Dgyro + Dpropagate: ✅ Complete
- Phase 4 — DfisX main loop: ✅ Complete
- Phase 5 — Landing handoff + debug tool: ✅ Complete

---

## Open Questions / Decisions Pending
- Confirm Unity render pipeline (Built-in / URP / HDRP) — affects disc material/shader
- GaussState uses System.Random — swap to Unity.Mathematics.Random before Burst migration (TODO in Daero.cs)
- Batch simulation (many throws for prediction) — IJobParallelFor migration when needed
- spinTransferFactor (0.3) needs tuning against real disc landing behaviour
- Flight arc currently goes along Unity X axis — camera must face X, or add heading rotation to scene setup

---

## Session Log
| Session | Work Done |
|---|---|
| Session 1 | Phase 1 complete. Data structures. Blittable struct decision. |
| Session 2 | Phase 2 complete. Daero.cs. GaussState threading fix. double→float. |
| Session 3 | Phase 3 complete. Dgyro.cs + Dpropagate.cs. Eigen transpose. Dead code dropped. |
| Session 4 | Phase 4 complete. DiscFlightSimulator.cs. Z-up decision. Sub-stepping. ToUnitySpace helpers. |
| Session 5 | Phase 5 complete. Extended DiscVisualizer.cs. Rigidbody handoff. asmdef split for Vector3/float3 ambiguity. |
| Session 6 | Debug tool complete. DiscThrowDebugger.cs. DiscModelPresetsEditor.cs. Scene setup. DfisX trail with keepAllThrows. Fixed DfisXStructs (no UnityEngine), DiscModel (inlined ToBlittable), DiscFlightSimulator (float3 alias). Disc flying confirmed. |