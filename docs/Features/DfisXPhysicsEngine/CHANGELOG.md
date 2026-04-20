# DfisX Physics Engine — Changelog

## [Unreleased]
- INF-4: Burst migration (System.Random → Unity.Mathematics.Random, [BurstCompile])

## Session 13 — SIM-4
- 1,272 PDGA disc models imported via DiscParamsImporter
- DiscModelLibrary auto-populate tool
- directModel field added to DiscInitState + ThrowParameters

## Session 7 — INF-1/2/6
- DiscVisionDeluxe.asmdef introduced (required for Camera/UI sub-modules)
- URP migration (Built-in → URP 17.4.0)
- Package versions pinned (CM 3.1.6, Burst 1.8.17)

## Session 5 — Phase 5
- DiscVisualizer.cs extended with Rigidbody handoff at z<=0
- DfisX trail LineRenderer added
- asmdef split into DfisX.Runtime (noEngineReferences=true) + DfisX.Unity

## Session 4 — Phase 4
- DiscFlightSimulator.cs: main loop, StepForUnityFrame(), sub-stepping at 1kHz
- ToUnitySpace() / ToUnityRotation() Z-up → Y-up conversion
- MAX_STEPS = 30,000

## Session 3 — Phase 3
- Dgyro.cs: gyroscopic precession, mode==true dead branch dropped
- Dpropagate.cs: Euler integration, renormalisation, redundant make_unit_vector dropped

## Session 2 — Phase 2
- Daero.cs: aerodynamic lift, drag, pitching moment
- GaussState class introduced (threading fix for gaussrand static state)
- double → float throughout

## Session 1 — Phase 1
- DfisXStructs.cs: DiscState, ForcesState, ThrowContainer, DiscModelData, AeroDebugSettings
- Blittable struct design established
- Unity.Mathematics.float3 over Vector3 decision
