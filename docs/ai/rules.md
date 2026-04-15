# AI Context Rules — DiscVisionDeluxe

## Session Start Protocol
1. Read INDEX.md first.
2. Identify subsystem(s) the task touches. Read ONLY those subsystem files.
3. For cross-cutting constraints or before any architectural decision: read decisions/architecture.md.
4. For native plugin / Emgu CV / Unity quirk debugging: read decisions/gotchas.md.
5. Do NOT pre-load tasks/archive.md, scene-setup.md, or gotchas.md unless specifically required.

## Subsystem Load Rules
| Task involves... | Load |
|---|---|
| DfisX physics, blittable structs, asmdef, Burst | subsystems/dfisX.md |
| KF pipeline, CSV input, DiscSimulator, DiscKalmanFilter | subsystems/kf-pipeline.md |
| FLIR camera, AprilTag, calibration, LiveDiscTracker | subsystems/camera.md |
| UI panels, Cinemachine, minimap, trails, disc viz | subsystems/ui-viz.md |
| Scene wiring, Inspector values, GameObject hierarchy | subsystems/scene-setup.md |
| Architectural decision or constraint check | decisions/architecture.md |
| Debugging native plugin, Emgu CV, solvePnP | decisions/gotchas.md |
| What tasks are open / mid-session resumption | tasks/active.md |

## Before Creating Any .cs File
1. Read the "Key Files" section of INDEX.md.
2. Read the relevant subsystem file.
3. Search the file map for any existing implementation.
4. Extend existing files before creating new ones.

## Updating These Files
- Task complete: move from tasks/active.md to tasks/archive.md as ONE table row.
- New non-obvious bug/fix: append ONE entry (<=8 lines) to decisions/gotchas.md.
- New architectural decision: append to decisions/architecture.md. Never delete entries.
- End of session: append ONE row to SESSION LOG in INDEX.md (<=120 chars).
- Never write session notes into subsystem files. They describe current system state, not history.
- Never expand archive entries. Archive is a tombstone, not a narrative.

## Token Budget
- INDEX.md + one subsystem file: ~600-900 tokens. Covers ~80% of tasks.
- INDEX.md + decisions/architecture.md: ~700 tokens. Covers constraint questions.
- Full load (all files): ~3,500 tokens. Reserve for planning sessions only.
- Reading >3 files for a focused coding task means the task is mis-scoped or the index needs updating.

## What NOT to Do
- Do not read tasks/archive.md and tasks/active.md in the same session unless asked about history.
- Do not read scene-setup.md during code work (Inspector reference, not code reference).
- Do not re-read rules.md mid-session.
- Do not append session notes to subsystem files.
