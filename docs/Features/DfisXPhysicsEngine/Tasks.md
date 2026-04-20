# DfisX Physics Engine — Tasks

## Active Session
- **Last action**: None
- **Next action**: INF-4 Burst migration
- **Blocker**: None

## Current Sprint
- [ ] INF-4: Burst migration (see steps below)

## INF-4 Steps
1. Daero.cs: replace `System.Random` in GaussState with `Unity.Mathematics.Random` (seeded on construction)
2. Add `[BurstCompile]` to Dpropagate, Dgyro, Daero step methods
3. Add `allowUnsafeCode: true` to DfisX.Runtime.asmdef
4. Verify simulation output unchanged vs pre-Burst

## Backlog
- [ ] Audit rimWidth values against PDGA approval sheets (Destroyer: imported 0.0245m, PDGA sheet ~0.021m)

## Completed
- [x] Phase 1: DiscState, ThrowContainer, DiscModelData, AeroDebugSettings (Session 1)
- [x] Phase 2: Daero.cs — aero forces, GaussState threading fix, double→float (Session 2)
- [x] Phase 3: Dgyro.cs + Dpropagate.cs — gyro precession, Euler integration, dead code dropped (Session 3)
- [x] Phase 4: DiscFlightSimulator.cs — main loop, sub-stepping, ToUnitySpace helpers (Session 4)
- [x] Phase 5: DiscVisualizer.cs extended — Rigidbody handoff, DfisX trail, asmdef split (Session 5)
- [x] Session 6: Debug tools, DiscModelPresetsEditor, DiscDebug scene. Disc flying confirmed.
- [x] INF-1: DiscVisionDeluxe.asmdef (Session 7)
- [x] INF-2: URP migration (Session 7)
- [x] INF-6: Package versions pinned — URP 17.4.0, CM 3.1.6, Burst 1.8.17 (Session 7)
- [x] SIM-4: Disc database — 1,272 discs imported, DiscParamsImporter, DiscModelLibrary (Session 13)
