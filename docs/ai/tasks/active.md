# Active Tasks — DiscVisionDeluxe
Update this file when starting or completing work. Move completed rows to archive.md.
Add mid-session state here so the next session can resume immediately.

## Mid-Session State
_(empty — update at session start if work was interrupted)_

---

## Open Tasks

### Infrastructure
| ID | Task | Priority |
|---|---|---|
| INF-4 | Burst migration | Medium |
| INF-5 | Main menu scene + AppSettings.cs | Low |

**INF-4 Steps:**
1. Daero.cs: replace `System.Random` in GaussState with `Unity.Mathematics.Random` (seeded on construction)
2. Add `[BurstCompile]` to Dpropagate, Dgyro, Daero step methods
3. Add `allowUnsafeCode: true` to DfisX.Runtime.asmdef
4. Verify simulation output unchanged vs pre-Burst

**INF-5 Notes:**
AppSettings.cs should persist: preferMetric, defaultDisc (DiscModel GUID or name), cameraSerial. PlayerPrefs or JSON at Application.persistentDataPath.

---

### Track A — Camera Pipeline
| ID | Task | Priority |
|---|---|---|
| CAM-ISSUE-1 | Redo camera calibration (RMS 3.4274 → target <1.0) | High |
| CAM-ISSUE-2 | Z axis inversion (blocked on ISSUE-1) | Medium |
| CAM-ISSUE-3 | Real throws not detected without establishment | Medium |
| CAM-ISSUE-4 | Detection timeout margin (update Inspector value) | Low |

**CAM-ISSUE-1 Steps:**
1. Better lighting (even, no harsh shadows on checkerboard)
2. Cover frame corners and edges (distortion worst there)
3. 20-30 frames, varied board angles
4. Measure printed square size with calipers
5. Recalibrate → confirm RMS <1.0 before proceeding

**CAM-ISSUE-3 Options (to try in order):**
1. Reduce image to 360×270 before DetectMarkers (halve intrinsics for solvePnP, or scale corners back up)
2. Lower primeMinVar further in Inspector (try 0.01–0.05)
3. Camera ROI mode at higher fps (requires USB bandwidth investigation for BayerRG8)

---

### Track B — Sim Experience

#### Priority 3 — Polish
| ID | Task | Priority | Status |
|---|---|---|---|
| POL-4 | Throw result statistics panel | Medium | Done — scene wire-up by user |
| POL-5 | Wind indicator HUD | Low | Not started |
| POL-6 | Practice range modes + leaderboard | Low | Not started |
| POL-7 | Replay system | Low | Not started |
| POL-8 | Multiple camera angles | Low | Not started |
| POL-9 | Course holes ScriptableObjects | Stretch | Not started |
| POL-10 | Spin animation on disc mesh | Low | In progress (mesh+foil done) |
| POL-11 | Practice range visual improvements | Low | Not started |
| POL-12 | Collision feedback into flight model | Low | Not started |

**POL-4 Notes:**
Code complete. Scene wire-up required:
1. Empty GameObject → UIDocument (Source = ThrowResultPanel.uxml, Sort Order above ThrowParameterPanel) + ThrowResultPanelController. Assign: discVisualizer, throwParamPanel, discPreview.
2. Separate empty GameObject `DiscPreviewRig` → DiscPreviewController. Assign sourceDisc = DiscVisualController in scene.
3. Assign DiscPreviewRig to ThrowResultPanelController.discPreview in Inspector.
If preview shows black box instead of transparency: URP camera may need allowHDR=false or camera output alpha override.

**POL-5 Notes:**
Windsock style: sock body droops/extends in wind direction, length reflects speed. Gust indicator driven by DiscEnvironment.gustFactor — brief elongation + ripple animation. Corner HUD overlay.

**POL-7 Notes:**
ThrowContainer.discStateArray is NativeArray<DiscState> (blittable). Serialize: .ToArray() → raw bytes → file. Load: read bytes → NativeArray.CopyFrom(). No JSON overhead. Play/pause/scrub coroutine.

**POL-12 Notes:**
OnCollisionEnter: capture contact normal, impulse magnitude, contact point → counter-force + counter-torque in DfisX body frame → inject into DiscFlightSimulator. May need externalForceDelta / externalTorqueDelta (blittable float3) fields on ThrowContainer read each integration step.
