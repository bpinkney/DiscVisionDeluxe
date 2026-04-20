# UI Results Panel — Design

## Files
- `Assets/DiscVisionDeluxe/UI/ThrowResultPanelController.cs`
- `Assets/DiscVisionDeluxe/UI/DiscPreviewController.cs`
- `Assets/UI/ThrowResultPanel.uxml`
- `Assets/UI/ThrowResultPanel.uss`

## ThrowResultPanelController

Stats shown: Disc Name, Speed, Spin RPM, Spin Factor, Hyzer, Nose Angle, Elevation, Azimuth, Distance, Lateral, Time Aloft, Turn, Fade.

- Panel appears immediately on throw (`HandleSimStarted`). Distance / Lateral / Time update in real-time via `Update()`.
- `HandleThrowFinished` finalises Turn, Fade, and corrects Distance / Lateral / Time with `BuildStats` values.
- History dropdown (top-bar): stores last 5 throws. Selecting from dropdown calls `PopulateFromHistory` on `ThrowParameterPanelController`.

**Spin sign convention:** `spinRpm = -rawSpin * 60 / (2π)` — negated so RHBH (negative spinRate in physics) displays positive.

**Turn/Fade:** `perpDir` = 90° CCW from throwDir (points right); `turnSide` = ±1 from spin sign; `fadeM` = `max(0, maxTurnLateral − signedLatLanding)`.

**Horizontal distance:** `length(relFinish.xy)` — 2D magnitude, NOT dot product (would undercount with lateral drift).

## ThrowContainer Snapshot Fields
Set in `DiscFlightSimulator.NewThrow()` (DfisXStructs.cs — Core, no UnityEngine):
`throwSpinRateRadS`, `throwHyzerRad`, `throwPitchRad`, `throwDiscName`.

## DiscPreviewController
Renders a 3D disc preview into a 288×172 RenderTexture, displayed as a `backgroundImage` on the `disc-preview` VisualElement.

**Isolation:** Entire rig (camera + disc copy) placed at y=5000. Camera `farClipPlane=3.0m` — main scene never bleeds in.

**Camera:** Local position (0, 0.4, 1.3) — from in front of disc, ~17° above horizontal. Shows hyzer as left/right tilt, nose angle as face pitch. FOV=14°, transparent clear `(0,0,0,0)`.

**Disc copy:** MeshFilter + MeshRenderer child. `MirrorDiscAppearance()` copies sharedMesh + creates Material instances from sourceDisc (`DiscVisualController`).

**UpdateOrientation(hyzerDeg, noseAngleDeg):** uses DiscNormal formula (same as `ThrowParameters.DiscNormal`): `nx=sin(-p)cos(h), ny=sin(h)cos(p), nz=cos(p)cos(h)`; DfisX→Unity axis swap `(x,y,z)→(x,z,y)`; then `Quaternion.LookRotation(Vector3.forward, discNormalUnity)`.

**RT binding:** one-frame coroutine in `ThrowResultPanelController.Start()` — `Background.FromRenderTexture(discPreview.PreviewRT)`.

**Known issue:** If RT shows black box instead of transparency, URP camera output alpha may need `allowHDR=false` or a camera output action tweak.

## Scene Wiring (Required)
Two GameObjects needed:

**ThrowResultPanel (GameObject)**
- UIDocument: Source = ThrowResultPanel.uxml, Sort Order = 2 (above ThrowParameterPanel)
- ThrowResultPanelController: assign `discVisualizer`, `throwParamPanel`, `discPreview`

**DiscPreviewRig (GameObject)**
- DiscPreviewController: assign `sourceDisc` = DiscVisualController on Disc GameObject
- No transform setup needed — `Awake()` moves it to y=5000 automatically

`ThrowResultPanelController.discPreview` → DiscPreviewRig's DiscPreviewController.
