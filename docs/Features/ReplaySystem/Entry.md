---
name: Replay System Entry
description: Status and key paths for throw replay (serialize discStateArray, play/pause/scrub)
type: project
---

# Replay System Entry

- **Status**: ⏳ Pending
- **Active task**: POL-7 — discStateArray.ToArray() → raw bytes → file; NativeArray.CopyFrom() on load; play/pause/scrub coroutine
- **Blocker**: None

## Key Scripts
- `Assets/DfisX/Runtime/Core/DfisXStructs.cs` (ThrowContainer.discStateArray — NativeArray<DiscState>)

## Load More
- Active tasks: [Tasks.md](Tasks.md)
