# UI Results Panel — Changelog

## Session 14 — POL-4
- ThrowResultPanelController.cs: stats panel, real-time updates, 5-throw history dropdown, PopulateFromHistory
- DiscPreviewController.cs: 3D disc preview in 288×172 RenderTexture, isolated at y=5000, transparent camera
- ThrowContainer snapshot fields added to DfisXStructs.cs (throwSpinRateRadS, throwHyzerRad, throwPitchRad, throwDiscName)
- FlightStats extended with Turn and Fade calculations
- Spin sign convention: negated so RHBH displays positive RPM
