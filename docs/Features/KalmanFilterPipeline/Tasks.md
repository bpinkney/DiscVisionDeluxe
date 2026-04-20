# Kalman Filter Pipeline — Tasks

## Active Session
- **Last action**: None
- **Next action**: None — feature complete
- **Blocker**: KF accuracy on live path depends on CameraPipeline calibration (CAM-ISSUE-1)

## Current Sprint
_(none)_

## Backlog
- [ ] Improve detection of natural throws without disc establishment (blocked on CAM-ISSUE-1/3 fix)

## Completed
- [x] DiscKalmanFilter.cs — full KF implementation with Idle/Priming/Tracking/Complete stages (Session 5-6)
- [x] DiscSimulator.cs — CSV path driver, DataSource enum, PushMeasurement() for live path (Session 5-6)
- [x] CsvLogReader.cs — CSV log parsing, angular extraction from rotation matrix (Session 5-6)
- [x] LiveDiscTracker integration — live path injection via PushMeasurement() (Session 12)
