# Kalman Filter Pipeline

The Kalman Filter estimates the disc's initial state (position, velocity, spin, orientation) from a series of noisy pose measurements, then fires a completed `DiscInitState` to launch the DfisX simulation.

## Data Flow

```
CSV path:
  Assets/DiscVisionDeluxe/TestLogs/csvlog.csv
    → CsvLogReader.ParseLine() → KFMeasurement
    → DiscSimulator → DiscKalmanFilter
    → onThrowComplete(DiscInitState)

Live camera path:
  SpinnakerCameraCapture + AprilTagDetector
    → LiveDiscTracker.BuildMeasurement() → KFMeasurement
    → DiscKalmanFilter (via DiscSimulator.PushMeasurement())
    → onThrowComplete(DiscInitState)
```

## KF Stages
`Idle` → `Priming` (collecting initial measurements) → `Tracking` → `Complete` → fires `onThrowComplete`

The KF transitions to Complete when it has enough primed measurements with sufficient velocity variance.

## Key Parameters

| Parameter | CSV Path | Live Camera Path | Notes |
|---|---|---|---|
| primeMaxEntries | 8 | 3 | Frames used for priming |
| primeCount | 8 | 3 | Frames needed to prime |
| primeMinVar | (default) | 0.05–0.5 | Velocity variance threshold |

Live camera params are more relaxed to account for lower effective detection rate (~25fps vs CSV's dense log).

## KFMeasurement Fields
- `timestampNs` — nanosecond timestamp (must come from the same clock source — see clock sync)
- `positionM` (float3, Z-up) — disc position in world frame
- `hyzerRad`, `pitchRad`, `spinRpm` — disc orientation/spin

## DiscInitState Output
All values in DfisX Z-up frame:
- `linearPositionM`, `linearVelocityMps` (float3)
- `hyzerRad`, `pitchRad`, `spinRpm`
- `discIndex` — enum for the 12 disc layout slots
- `directModel` [NonSerialized] — direct DiscModel reference, bypasses index lookup in DiscFlightSimulator

## Angular Extraction (CsvLogReader / AprilTagDetector)
From rotation matrix R (row-major):
```
hyzer = asin(R[1,2])
pitch = asin(R[0,2])
spin  = atan2(R[0,1], R[0,0])
```

## DiscSimulator Modifications
`DataSource` enum added (CSV, LiveCamera). `PushMeasurement(KFMeasurement)` added for live path injection.

## Clock Synchronisation (Live Path)
All timestamps must come from `SpinnakerCameraCapture._captureClock` (a single `Stopwatch` started in `StartCapture()`). Do not mix with `DateTime.Now` or `Time.realtimeSinceStartup` — clock mixing causes immediate detection timeouts.
