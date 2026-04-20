# Kalman Filter Pipeline — Design

## Key Files
| File | Role |
|---|---|
| DiscState.cs | KFState, KFMeasurement, DiscInitState, DiscIndex, PosVelVarState |
| DiscKalmanFilter.cs | Full KF implementation. Stages: Idle → Priming → Tracking → Complete. |
| DiscSimulator.cs | Drives KF from CSV or live path. DataSource enum (CSV/LiveCamera). PushMeasurement() for live path. Fires onThrowComplete(DiscInitState). |
| CsvLogReader.cs | Reads Assets/DiscVisionDeluxe/TestLogs/csvlog.csv. ParseLine() builds KFMeasurement. |

## KFState Fields
`linearPositionM` (float3, Z-up), `linearVelocityMps` (float3), `hyzerRad`, `pitchRad`, `spinRpm`, `discIndex`, `directModel`.

## DiscInitState Output
All values in DfisX Z-up frame:
- `linearPositionM`, `linearVelocityMps` (float3)
- `hyzerRad`, `pitchRad`, `spinRpm`
- `discIndex` — enum for the 12 disc layout slots
- `directModel` [NonSerialized] — direct DiscModel reference, bypasses index lookup

## KF Parameters
| Parameter | CSV Path | Live Camera Path | Notes |
|---|---|---|---|
| primeMaxEntries | 8 | 3 | Frames used for priming |
| primeCount | 8 | 3 | Frames needed to prime |
| primeMinVar | (default, tighter) | 0.05 | Velocity variance threshold |

Live camera params are more relaxed to account for lower effective detection rate (~25fps vs. CSV's dense log).

## Angular Extraction (CsvLogReader / AprilTagDetector)
From rotation matrix R (row-major, Z-up frame):
```
hyzer = asin(R[1,2])
pitch = asin(R[0,2])
spin  = atan2(R[0,1], R[0,0])
```

## DiscSimulator Modifications
`DataSource` enum added (CSV, LiveCamera). `PushMeasurement(KFMeasurement)` added for live path injection. CSV path unchanged.

## Coordinate Frame
KF operates in DfisX Z-up frame throughout. `LiveDiscTracker` applies `cameraToWorld` before injecting measurements. `CsvLogReader` produces Z-up measurements directly.

## Clock Synchronisation (Live Path)
All timestamps must come from `SpinnakerCameraCapture._captureClock` (a single `Stopwatch` started in `StartCapture()`). Do not mix with `DateTime.Now` or `Time.realtimeSinceStartup` — clock mixing causes immediate detection timeouts.
