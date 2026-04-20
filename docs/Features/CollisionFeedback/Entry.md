---
name: Collision Feedback Entry
description: Status and key paths for injecting collision impulses back into the DfisX flight model
type: project
---

# Collision Feedback Entry

- **Status**: ⏳ Pending
- **Active task**: POL-12 — OnCollisionEnter → counter-force/torque in DfisX body frame; may need externalForceDelta/externalTorqueDelta on ThrowContainer
- **Blocker**: None

## Key Scripts
- `Assets/DfisX/Runtime/Core/DfisXStructs.cs` (ThrowContainer — needs new fields)
- `Assets/DiscVisionDeluxe/DiscVisualizer.cs` (OnCollisionEnter injection point)

## Load More
- Active tasks: [Tasks.md](Tasks.md)
- Physics reference: [DfisXPhysicsEngine/Design.md](../DfisXPhysicsEngine/Design.md)
