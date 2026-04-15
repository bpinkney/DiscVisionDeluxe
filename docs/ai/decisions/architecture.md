# Architecture Decisions — DiscVisionDeluxe
Append-only. Never delete entries.

## Key Invariants (Must Not Violate)
1. Every input pathway (camera, UI sliders, replay) terminates at `DiscVisualizer.LaunchDfisX(DiscInitState)`. Never bypass.
2. No Unity API calls from background threads. Thread boundary = `ConcurrentQueue<DetectionFrame>` / `ConcurrentQueue<KFMeasurement>`.
3. Camera pipeline must output DfisX Z-up frame (X forward, Y right, Z up). Ref: `CsvLogReader.ParseLine()` R-matrix formulas.
4. New physics-facing structs must be blittable (no managed refs, no string fields).
5. `DiscVisionDeluxe` scripts may reference `DfisX.Unity` and `DfisX.Runtime`. The reverse is forbidden.
6. Check INDEX.md key files before creating any new .cs file — extend existing files first.

---

## DfisX Port Decisions (Sessions 1-6)

### C# port over native plugin
Daero.cpp is pure math — ports line-for-line. Native plugin loses Unity debugger, P/Invoke marshalling complexity, gaussrand() static state breaks multithreaded simulation. C# allows Unity Jobs parallelism later.

### Blittable structs for DiscState / ForcesState
Target: many simultaneous disc flights. Blittable structs → NativeArray<DiscState> → IJobParallelFor → Burst SIMD. ThrowContainer stays managed class — owns NativeArray and handles lifetime/resize.

### Unity.Mathematics float3 over UnityEngine.Vector3
float3 is blittable; Vector3 is not safe inside NativeArray. Burst understands float3 natively. noEngineReferences=true on DfisX.Runtime.asmdef prevents Vector3/float3 ambiguity in math files.

### DiscModel as ScriptableObject + DiscModelData blittable mirror
DiscModel ScriptableObject = authored/Inspector-facing. DiscModelData struct = blittable mirror, converted once at throw-start via DiscModel.ToBlittable(). ToBlittable() inlined directly — no FromScriptableObject() dependency.

### GaussState class (gaussrand threading fix)
C++ static locals were a threading hazard. GaussState managed class, one per ThrowContainer. Future Burst path: replace System.Random with Unity.Mathematics.Random (TODO in Daero.cs — tracked as INF-4).

### double → float throughout
Acceptable precision at 1kHz sim step. Required for Burst.

### Coordinate system: Z-up internally
Sim runs Z-up matching C++. Gravity is (0, 0, -9.8*mass). Unity Y-up conversion at render time via DiscFlightSimulator.ToUnitySpace() / ToUnityRotation(). Camera should face along positive X axis.

### C# float3x3 is row-major
Matrices built directly in correct form — no transpose needed (unlike Eigen in C++).

### Dgyro dead code dropped
mode==true branch entirely commented-out. Dropped. Redundant make_unit_vector at end dropped (Dpropagate renormalises).

### consume_Dcollision inert block omitted
overwrite_states_ignore_forces_torques block guarded by =false — entirely inert. Only active impulse-torque branch ported.

### Landing handoff: Rigidbody
DfisX owns flight phase. At z<=0, OnThrowFinished fires, DiscVisualizer hands off to Rigidbody. spinTransferFactor (default 0.3) scales spin → Rigidbody angular velocity. PhysX handles bounce/roll/settle via PhysicsMaterial on terrain.

### asmdef split to resolve Vector3/float3 ambiguity
DfisX.Runtime (Core/): noEngineReferences=true. DfisX.Unity (Runtime/): noEngineReferences=false. DfisXStructs.cs has no 'using UnityEngine'. DiscFlightSimulator.cs needs 'using float3 = Unity.Mathematics.float3' alias at top.

---

## Post-Port Decisions (Sessions 7+)

### INF-1: DiscVisionDeluxe.asmdef required before Camera/UI sub-modules
Before this existed, all DiscVisionDeluxe/ scripts compiled into Assembly-CSharp. Adding a named asmdef is required before any sub-module (Camera, UI) can reference types like KFMeasurement, DiscSimulator from a sibling asmdef.

### INF-2: URP Migration
Migrated from Built-in to URP at Session 7 before scene/material work began to avoid double-conversion cost. All new materials/shaders use URP. Packages: com.unity.render-pipelines.universal 17.4.0 (installed via Package Manager UI).

### INF-6: Package Versions for Unity 6000.4
- com.unity.render-pipelines.universal: 17.4.0
- com.unity.cinemachine: 3.1.6
- com.unity.burst: 1.8.17
- com.unity.jobs: removed (version conflict — not needed)

### SIM-1: UI Toolkit for all new panels
com.unity.modules.uielements already in manifest. UI Toolkit (UXML/USS) preferred over uGUI Canvas — supports data binding, UXML/USS separation, better for theming. Keep OnGUI only in DiscThrowDebugger (debug tool, no change required).

### CAM-1: P/Invoke against SpinnakerC_v140, not SpinnakerNET
SpinnakerNET_v140.dll is C++/CLI mixed-mode — Unity cannot load it as managed assembly regardless of Inspector config. All bindings live in SpinnakerCAPI.cs (DllImport("SpinnakerC_v140")). SpinnakerCAPI must be public (not internal) so Camera.Editor.asmdef can access it.

### CAM-5: Thread Safety Contract
- Background thread: camera capture → AprilTag detect → push to ConcurrentQueue<DetectionFrame>
- Main thread Update(): drain queue → onTagsDetected.Invoke() → LiveDiscTracker.OnTagsDetected() → _kf.AddMeasurement()
- All UnityEvent callbacks (onThrowComplete, etc.) always fire on main thread
- NEVER call any Unity API from background thread

### CAM-5: Clock Synchronisation
All timestamps must come from SpinnakerCameraCapture._captureClock (single Stopwatch started in StartCapture()). LiveDiscTracker reads cameraCapture.ElapsedCaptureNs for current time and cameraCapture.LastDetectionTimestampNs for each detection's capture time. Mixing clocks causes immediate detection timeouts.

### Disc Selection: directModel field
DiscInitState and ThrowParameters both gained a [NonSerialized] directModel field. Panel sets it to discModelLibrary.discs[_selectedModel]. DiscFlightSimulator.NewThrow() uses p.directModel ?? FindByDiscIndex(p.discIndex). Prevents disc model being lost through KF pipeline.

### MAX_STEPS = 30,000
Raised from 10,000. Original 10s cutoff too short for understable drivers executing long hyzer-flip arcs (~30s at 1kHz).

### Rigidbody Handoff: position sync before going non-kinematic
HandoffToRigidbody syncs discRigidbody.transform.position to discTransform.position before going non-kinematic. Critical when discTransform is a child of the Rigidbody object. Also sets CollisionDetectionMode.Continuous to prevent tunneling.

### Cinemachine 3 (CM3 API)
Unity 6 uses CM3 (CinemachineCamera, not CinemachineVirtualCamera which is CM2). VCam_Follow manually positioned every frame (no procedural components). VCam_Side and VCam_Overhead use CinemachineFollow + RotationComposer. VCam switching: vcam.Priority = int.
