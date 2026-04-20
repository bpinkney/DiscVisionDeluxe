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
References: `DfisX.Unity`, `DfisX.Runtime`, `Unity.Mathematics`, `Unity.Collections`, `Unity.Cinemachine`
Auto-referenced. Contains `DiscState.cs`, `DiscKalmanFilter.cs`, `DiscSimulator.cs`, `DiscVisualizer.cs`, `CsvLogReader.cs`. Subfolders `FlightCameras/` (`FollowFlightCamera.cs`), `Visualization/`, `Environment/` also compile into this assembly.

---

### DiscVisionDeluxe.Camera
**Location:** `Assets/DiscVisionDeluxe/Camera/DiscVisionDeluxe.Camera.asmdef`
References: `DiscVisionDeluxe`, `DfisX.Unity`, `DfisX.Runtime`, `Unity.Mathematics`, `Unity.Collections`
**Platforms: Editor + WindowsStandalone64 only** (Spinnaker SDK is Windows-only)
Spinnaker/AprilTag pipeline only — no Cinemachine. No `precompiledReferences` needed — Emgu CV types are picked up through the managed DLL in Assets/Packages/.

---

### DiscVisionDeluxe.UI
**Location:** `Assets/DiscVisionDeluxe/UI/DiscVisionDeluxe.UI.asmdef`
References: `DiscVisionDeluxe`, `DfisX.Unity`, `DfisX.Runtime`, `Unity.Mathematics`, `Unity.Collections`, `Unity.TextMeshPro`

---

### DfisX.Editor
**Location:** `Assets/DfisX/Editor/DfisX.Editor.asmdef`
Editor-only. References: `DfisX.Unity`, `DfisX.Runtime`. Contains `DiscModelPresetsEditor.cs`, `DiscParamsImporter.cs`.

---

### DiscVisionDeluxe.Camera.Editor
**Location:** `Assets/DiscVisionDeluxe/Camera/Editor/DiscVisionDeluxe.Camera.Editor.asmdef`
Editor-only. References: `DiscVisionDeluxe.Camera`

---

## Dependency Graph

```
DfisX.Runtime ←── DfisX.Unity ←── DfisX.Editor (Editor only)
      ↑                ↑
      └────────────────┴─── DiscVisionDeluxe  (incl. FlightCameras/, Visualization/, Environment/)
                                    ↑
                            DiscVisionDeluxe.Camera  (Editor + Win64 only)
                            DiscVisionDeluxe.UI
```

**The reverse direction is forbidden.** `DfisX.Runtime` and `DfisX.Unity` must never reference `DiscVisionDeluxe`. `DiscVisionDeluxe.Camera` and `DiscVisionDeluxe.UI` are sibling dependents of `DiscVisionDeluxe` — UI does not depend on Camera.

## Platform Restriction

`DiscVisionDeluxe.Camera` is restricted to Editor and Windows 64-bit Standalone. The Spinnaker SDK has no other platform support. Code in `Camera/` that needs to compile elsewhere must be wrapped in `#if UNITY_EDITOR || UNITY_STANDALONE_WIN` guards.
