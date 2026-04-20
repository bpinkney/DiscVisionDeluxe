# Main Menu — Tasks

## Active Session
- **Last action**: None
- **Next action**: INF-5 — create MainMenu.unity scene + AppSettings.cs
- **Blocker**: None

## Current Sprint
- [ ] INF-5: MainMenu.unity scene
- [ ] INF-5: AppSettings.cs persistence

## Design Notes
`AppSettings.cs` should persist:
- `preferMetric` (bool)
- `defaultDisc` (DiscModel GUID or name)
- `cameraSerial` (string)

Storage: PlayerPrefs or JSON at `Application.persistentDataPath`. Load on startup, apply before scene load.

## Backlog
_(none)_

## Completed
_(none)_
