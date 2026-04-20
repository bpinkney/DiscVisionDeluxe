# Kalman Filter Pipeline — Changelog

## Session 12 — CAM-5 Live Integration
- LiveDiscTracker.cs wired to DiscKalmanFilter via PushMeasurement()
- Z inversion workaround added in FireComplete() (clamp linearPositionM.z=1.0f when negative)
- primeMinVar lowered to 0.05 for live camera path

## Session 5-6 — Initial Implementation
- DiscKalmanFilter.cs: full KF with Idle/Priming/Tracking/Complete stages
- DiscSimulator.cs: CSV path, DataSource enum, onThrowComplete event
- CsvLogReader.cs: ParseLine(), angular extraction from R-matrix
- DiscInitState defined; directModel [NonSerialized] field added
