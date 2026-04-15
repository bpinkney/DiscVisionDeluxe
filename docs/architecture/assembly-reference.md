# Assembly Reference

## Assembly Definitions

### DfisX.Runtime
**Location:** `Assets/DfisX/Runtime/Core/DfisX.Runtime.asmdef`
```json
{
  "name": "DfisX.Runtime",
  "references": ["Unity.Mathematics", "Unity.Collections"],
  "autoReferenced": false,
  "noEngineReferences": true
}
```
Pure math — no `UnityEngine` dependency. Contains `DfisXStructs.cs`, `Daero.cs`, `Dgyro.cs`, `Dpropagate.cs`.

---

### DfisX.Unity
**Location:** `Assets/DfisX/Runtime/DfisX.Unity.asmdef`
```json
{
  "name": "DfisX.Unity",
  "references": ["DfisX.Runtime", "Unity.Mathematics", "Unity.Collections"],
  "autoReferenced": true,
  "noEngineReferences": false
}
```
Unity-facing code: `DiscFlightSimulator.cs`, `DiscModel.cs`, `DiscModelLibrary.cs`, `DiscModelPresets.cs`, `ThrowParameters.cs`.

---

### DiscVisionDeluxe
**Location:** `Assets/DiscVisionDeluxe/DiscVisionDeluxe.asmdef`
References: `DfisX.Unity`, `DfisX.Runtime`, `Unity.Mathematics`, `Unity.Collections`
Auto-referenced. Contains `DiscState.cs`, `DiscKalmanFilter.cs`, `DiscSimulator.cs`, `DiscVisualizer.cs`, `CsvLogReader.cs`.

---

### DiscVisionDeluxe.Camera
**Location:** `Assets/DiscVisionDeluxe/Camera/DiscVisionDeluxe.Camera.asmdef`
References: `DiscVisionDeluxe`, `Unity.Cinemachine`, `DfisX.Unity`, `DfisX.Runtime`, `Unity.Mathematics`, `Unity.Collections`
**Platforms: Editor + WindowsStandalone64 only** (Spinnaker SDK is Windows-only)
No `precompiledReferences` needed — Emgu CV types are picked up through the managed DLL in Assets/Packages/.

---

### DiscVisionDeluxe.UI
**Location:** `Assets/DiscVisionDeluxe/UI/DiscVisionDeluxe.UI.asmdef`
References: `DiscVisionDeluxe`, `DiscVisionDeluxe.Camera`, `DfisX.Unity`, `DfisX.Runtime`, `Unity.Mathematics`, `Unity.Collections`, `Unity.TextMeshPro`

---

### DiscVisionDeluxe.Camera.Editor
**Location:** `Assets/DiscVisionDeluxe/Camera/Editor/DiscVisionDeluxe.Camera.Editor.asmdef`
Editor-only. References: `DiscVisionDeluxe.Camera`

---

## Dependency Graph

```
DfisX.Runtime ←── DfisX.Unity
      ↑                ↑
      └────────────────┴─── DiscVisionDeluxe
                                    ↑
                            DiscVisionDeluxe.Camera
                                    ↑
                             DiscVisionDeluxe.UI
```

**The reverse direction is forbidden.** `DfisX.Runtime` and `DfisX.Unity` must never reference `DiscVisionDeluxe`. This keeps the physics engine free of application dependencies.

## Platform Restriction

`DiscVisionDeluxe.Camera` is restricted to Editor and Windows 64-bit Standalone. The Spinnaker SDK has no other platform support. Any code in the `Camera/` folder that needs to compile on other platforms must be wrapped in `#if UNITY_EDITOR || UNITY_STANDALONE_WIN` guards.
