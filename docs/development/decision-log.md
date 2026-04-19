# Architecture Decision Log

Chronological record of significant design decisions. Append new entries; do not delete or modify old ones.

---

## DfisX Port Decisions (Sessions 1–6)

### C# Port Over Native Plugin
Daero.cpp is pure math and ports line-for-line. A native plugin (.dll) would lose the Unity debugger, require P/Invoke marshalling, and `gaussrand()`'s static state would be a threading hazard for multi-throw simulation. C# enables Unity Jobs/Burst parallelism later.

### Blittable Structs for DiscState / ForcesState
Target use case: running many simultaneous disc flights for prediction. Blittable structs → `NativeArray<DiscState>` → `IJobParallelFor` → Burst SIMD. `ThrowContainer` stays managed (owns the NativeArray and handles lifetime/resize).

### Unity.Mathematics float3 Over UnityEngine.Vector3
`float3` is blittable and safe in `NativeArray`. `Vector3` is not. Burst understands `float3` natively. `noEngineReferences=true` on `DfisX.Runtime.asmdef` prevents type ambiguity in math files.

### DiscModel as ScriptableObject + DiscModelData Blittable Mirror
`DiscModel` ScriptableObject is Inspector-facing; `DiscModelData` struct is the blittable mirror for the sim. Converted once at throw-start via `DiscModel.ToBlittable()` (inlined — no external factory method dependency).

### GaussState Class (Threading Fix)
C++ used static local state in `gaussrand()` — a threading hazard. `GaussState` is a managed class instance owned by `ThrowContainer`, one per throw. Future Burst path: replace `System.Random` with `Unity.Mathematics.Random`.

### double → float Throughout
Acceptable precision at 1kHz simulation step. Required for Burst compilation.

### Coordinate System: Z-Up Internally
Simulation runs Z-up (X forward, Y right, Z up) matching the original C++. Gravity = `(0, 0, -9.8 * mass)`. Unity Y-up conversion at render time via `ToUnitySpace()` / `ToUnityRotation()`. Disc flies along Unity +X axis.

### C# float3x3 Is Row-Major (No Transpose Needed)
C++ used Eigen (column-major); C# `float3x3` is row-major. Matrices built directly in correct form — no transpose needed, unlike the original C++.

### Dgyro Dead Code Dropped
The `mode == true` branch in `step_Dgyro` was entirely commented-out in the C++. Dropped in C#. The redundant `make_unit_vector` at the end of the step was also dropped (`Dpropagate` renormalises).

### consume_Dcollision Inert Block Omitted
The `overwrite_states_ignore_forces_torques` block in collision handling was guarded by `= false` — entirely inert. Only the active impulse-torque branch was ported.

### Landing Handoff: Rigidbody
DfisX owns the flight phase. At `z <= 0`, `OnThrowFinished` fires and `DiscVisualizer` hands off to PhysX Rigidbody. `spinTransferFactor` (default 0.3) scales angular velocity transfer. PhysX handles bounce/roll/settle via PhysicsMaterial on terrain.

### asmdef Split to Resolve Vector3/float3 Ambiguity
`DfisX.Runtime` (Core/): `noEngineReferences=true`. `DfisX.Unity` (Runtime/): `noEngineReferences=false`. Files in Core/ must never include `using UnityEngine`. `DiscFlightSimulator.cs` requires `using float3 = Unity.Mathematics.float3` alias.

---

## Post-Port Decisions (Sessions 7+)

### INF-1: Named asmdef Required Before Sub-Module asmdefs
Without `DiscVisionDeluxe.asmdef`, all scripts compiled into `Assembly-CSharp`. Sub-module asmdefs (`Camera`, `UI`) cannot reference `Assembly-CSharp` types from sibling asmdefs.

### INF-2: URP Migration Before Scene/Material Work
Migrating URP first prevents double-conversion of materials (Built-in → URP). All new materials and shaders use URP. Built-in render pipeline is no longer supported in this project.

### SIM-1: UI Toolkit for All New Panels
UI Toolkit (UXML/USS) is preferred over uGUI Canvas: supports data binding, UXML/USS separation, better for theming. `DiscThrowDebugger.OnGUI` is the only exception — kept as-is.

### CAM-1: P/Invoke Against SpinnakerC_v140
`SpinnakerNET_v140.dll` is C++/CLI mixed-mode and cannot be used as a managed assembly. P/Invoke against the pure-C `SpinnakerC_v140.dll` is the correct integration path. `SpinnakerCAPI.cs` must be `public` (not `internal`) for the Editor assembly to access it.

### CAM-5: Thread Safety Contract
Background threads may only communicate with the main thread through `ConcurrentQueue`. All Unity API calls (MonoBehaviour methods, UnityEvent callbacks, transform access) must happen on the main thread. This contract is non-negotiable — violations cause intermittent crashes or Unity API exceptions.

### CAM-5: Single Clock Source for All Timestamps
All timestamps in the camera pipeline come from a single `Stopwatch` started in `StartCapture()`. Mixing clock sources (e.g. `DateTime.Now` on capture thread, `Time.realtimeSinceStartup` on main thread) causes immediate detection timeouts due to nanosecond-scale vs. millisecond-scale discrepancies.

### Disc Selection: directModel Field Added to DiscInitState + ThrowParameters
The 12-slot `DiscIndex` enum cannot address 1,272+ disc models. Both structs gained `[NonSerialized] directModel` (DiscModel reference). `DiscFlightSimulator.NewThrow()` uses `p.directModel ?? FindByDiscIndex(p.discIndex)` — direct reference takes priority.

### MAX_STEPS Raised to 30,000
The original 10,000-step limit (~10s at 1kHz) was too short for understable drivers executing long hyzer-flip arcs. Raised to 30,000 (~30s).

### Rigidbody Handoff: Sync Position Before Non-Kinematic
`discRigidbody.transform.position` is synced to `discTransform.position` before disabling kinematic. Required when `discTransform` is a child of the Rigidbody object — without this sync, the Rigidbody's position is stale at handoff. `CollisionDetectionMode.Continuous` is also set to prevent tunneling through the ground.

### Cinemachine 3 API
Unity 6 ships with Cinemachine 3 (`CinemachineCamera`). Do not use `CinemachineVirtualCamera` (CM2). Camera switching via `vcam.Priority = int`. `VCam_Follow` is manually positioned each frame (no procedural components). Auto-switching on throw was removed — it was overriding the user's camera selection.
