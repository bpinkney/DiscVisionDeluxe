# Camera Hardware Setup

## Camera Specifications — BFS-U3-04S2C-CS (Teledyne FLIR Blackfly S)
| Spec | Value |
|---|---|
| Sensor | Sony IMX287, 1/2.9" CMOS, global shutter |
| Resolution | 720 × 540 (0.4 MP) |
| Max FPS — Mono8 | 290 FPS (recommended; ISP demosaics Bayer → mono) |
| Max FPS — BayerRG8 | 522 FPS (ISP off; requires software debayer; USB bandwidth issues at full rate) |
| Max FPS — 320×240 ROI | ~997–999 FPS |
| Interface | USB 3.1 Gen 1 |
| Pixel format in use | Mono8 (configured via GenICam node map in InitCamera()) |
| Global shutter | Yes — all pixels expose simultaneously, no rolling shutter distortion |

The camera is a **color sensor** (the "C" in BFS-U3-04S2**C**-CS). Requesting `Mono8` triggers on-camera ISP demosaicing, capping rate at 290 FPS. This is the recommended mode.

## Recommended Inspector Settings
| Field | Value | Reason |
|---|---|---|
| Target Fps | 290 | Mono8 maximum |
| Exposure Us | 0 | Auto exposure |
| Auto Exposure Max Us | 1000 | Caps at 1 ms — prevents motion blur on spinning disc in dim light |
| Gain Db | 0 | Auto gain |

Raise `Auto Exposure Max Us` to 2000–4000 µs if the image is too dark.

## Spinnaker SDK Installation

### Step 1: Install Spinnaker SDK
Download and install from the FLIR website. The installer places files in `C:\Program Files\FLIR Systems\Spinnaker\`.

A full PC **reboot** is required after installation to seat the Spinnaker kernel filter driver.

### Step 2: Copy DLLs to Unity

Create `Assets/Plugins/Spinnaker/` and copy all of the following DLLs:

| DLL | Source |
|---|---|
| SpinnakerC_v140.dll | `C:\Program Files\FLIR Systems\Spinnaker\bin64\vs2015\` |
| Spinnaker_v140.dll | same |
| SpinnakerNET_v140.dll | same — present for completeness, not used in code |
| GCBase_MD_VC140_v3_0.dll | same |
| GenApi_MD_VC140_v3_0.dll | same |
| log4cpp_MD_VC140_v3_0.dll | same |
| Log_MD_VC140_v3_0.dll | same |
| MathParser_MD_VC140_v3_0.dll | same |
| NodeMapData_MD_VC140_v3_0.dll | same |
| XMLParser_MD_VC140_v3_0.dll | same |
| libiomp5md.dll | same |
| vcomp140.dll | `C:\Windows\System32\` — NOT in Spinnaker bin64 |

### Step 3: Configure DLL Plugin Settings

For each DLL in Unity Inspector:
- **Type:** Native
- **x86:** UNCHECKED
- **Editor:** ✅ (CPU: x86_64, OS: Windows)
- **Standalone (Windows 64):** ✅ (CPU: x86_64)
- All others: off

**After adding or changing any native DLL: fully close and reopen Unity.** Native plugins are loaded at Editor startup — re-entering Play mode is not enough.

## Why P/Invoke (not SpinnakerNET)

`SpinnakerNET_v140.dll` is a **C++/CLI mixed-mode assembly**. Unity cannot load it as a managed assembly regardless of plugin configuration — it always behaves as a native DLL and produces CS0246 compiler errors if referenced. All camera bindings instead use P/Invoke against `SpinnakerC_v140.dll` (a pure C interface), implemented in `SpinnakerCAPI.cs`.

## Diagnosing Missing DLLs

If the camera fails to initialize at runtime:
1. [Dependencies tool](https://github.com/lucasg/Dependencies) — drag `SpinnakerC_v140.dll` into the tree; red entries are missing dependencies.
2. Process Monitor (Sysinternals) — filter: `Process=Unity.exe`, `Result=NAME NOT FOUND`, `Path contains Spinnaker`.
