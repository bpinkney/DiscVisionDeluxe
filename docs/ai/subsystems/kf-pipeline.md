# Kalman Filter Pipeline — Reference

## Two Input Paths (same output)
```
CSV path:    csvlog.csv → CsvLogReader → DiscSimulator → DiscKalmanFilter
Live path:   FLIR frame → AprilTagDetector → LiveDiscTracker → DiscKalmanFilter
Both →       onThrowComplete(DiscInitState) → DiscVisualizer.LaunchDfisX()
```

## Key Files (Assets/DiscVisionDeluxe/)
| File | Role |
|---|---|
| DiscState.cs | KFState, KFMeasurement, DiscInitState, DiscIndex, PosVelVarState |
| DiscKalmanFilter.cs | Full KF implementation. Stages: Idle → Priming → Tracking → Complete. |
| DiscSimulator.cs | Drives KF from CSV. DataSource enum (CSV/LiveCamera). PushMeasurement() for live path. Fires onThrowComplete(DiscInitState). |
| CsvLogReader.cs | Reads Assets/DiscVisionDeluxe/TestLogs/csvlog.csv. ParseLine() builds KFMeasurement. R-matrix angular extraction: hyzer=asin(R[1,2]), pitch=asin(R[0,2]), spin=atan2(R[0,1],R[0,0]). |
| DiscVisualizer.cs | LaunchDfisX(DiscInitState) — public. Called by KF pipeline OR DiscThrowDebugger. |
| DiscThrowDebugger.cs | Manual throw debug tool. Inspector params + OnGUI buttons. Calls LaunchDfisX directly. |

## KFState Fields
linearPositionM (float3, Z-up), linearVelocityMps (float3), hyzerRad, pitchRad, spinRpm, discIndex, directModel.

## DiscInitState
Output of KF. Same fields as KFState plus directModel [NonSerialized]. All values in DfisX Z-up frame.

## KF Params — CSV path (defaults in DiscSimulator)
- primeMaxEntries = 8
- primeCount = 8
- primeMinVar = (default, tighter)

## KF Params — Live camera path (set in LiveDiscTracker Inspector)
- primeMaxEntries = 3 (last 3 frames only)
- primeCount = 3 (need 3 to prime)
- primeMinVar = 0.05 (lowered for slow/establish-then-move testing; was 0.5)

## DiscSimulator Modifications (post-port)
Added DataSource enum (CSV, LiveCamera). Added PushMeasurement(KFMeasurement) for live path injection. CSV path unchanged.

## Coordinate Frame
KF operates in DfisX Z-up frame throughout. LiveDiscTracker applies cameraToWorld before injecting measurements. CsvLogReader produces Z-up measurements directly.
