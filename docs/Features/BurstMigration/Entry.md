---
name: Burst Migration Entry
description: Status and key paths for migrating DfisX Core to Burst-compilable code
type: project
---

# Burst Migration Entry

- **Status**: ⏳ Pending
- **Active task**: INF-4 — Replace System.Random with Unity.Mathematics.Random; add [BurstCompile] to Dpropagate/Dgyro/Daero; add allowUnsafeCode to asmdef
- **Blocker**: None

## Key Scripts
- `Assets/DfisX/Runtime/Core/Daero.cs` (GaussState — System.Random replacement)
- `Assets/DfisX/Runtime/Core/Dpropagate.cs`
- `Assets/DfisX/Runtime/Core/Dgyro.cs`
- `Assets/DfisX/Runtime/Core/DfisX.Runtime.asmdef`

## Load More
- Active tasks: [Tasks.md](Tasks.md)
- Physics reference: [DfisXPhysicsEngine/Design.md](../DfisXPhysicsEngine/Design.md)
