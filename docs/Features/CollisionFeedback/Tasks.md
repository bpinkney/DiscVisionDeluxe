# Collision Feedback — Tasks

## Active Session
- **Last action**: None
- **Next action**: Design injection interface (POL-12)
- **Blocker**: None

## Current Sprint
- [ ] POL-12: Collision feedback into flight model

## Design Notes
`OnCollisionEnter`: capture contact normal, impulse magnitude, contact point → counter-force + counter-torque in DfisX body frame → inject into `DiscFlightSimulator`.
May need `externalForceDelta` / `externalTorqueDelta` (blittable float3) fields on `ThrowContainer`, read each integration step.

## Backlog
_(none)_

## Completed
_(none)_
