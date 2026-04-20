# Replay System — Tasks

## Active Session
- **Last action**: None
- **Next action**: Implement serialization (POL-7)
- **Blocker**: None

## Current Sprint
- [ ] POL-7: Replay system

## Design Notes
`ThrowContainer.discStateArray` is `NativeArray<DiscState>` (blittable).
Serialize: `.ToArray()` → raw bytes → file. Load: read bytes → `NativeArray.CopyFrom()`. No JSON overhead.
Play/pause/scrub coroutine drives `DiscVisualizer` position/rotation from state array.

## Backlog
_(none)_

## Completed
_(none)_
