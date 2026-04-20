# Burst Migration — Tasks

## Active Session
- **Last action**: None
- **Next action**: INF-4 step 1 — replace System.Random in GaussState
- **Blocker**: None

## Current Sprint
- [ ] INF-4: Burst migration

## Steps
1. `Daero.cs`: replace `System.Random` in `GaussState` with `Unity.Mathematics.Random` (seeded on construction)
2. Add `[BurstCompile]` to `Dpropagate`, `Dgyro`, `Daero` step methods
3. Add `allowUnsafeCode: true` to `DfisX.Runtime.asmdef`
4. Verify simulation output unchanged vs pre-Burst

## Backlog
_(none)_

## Completed
_(none)_
