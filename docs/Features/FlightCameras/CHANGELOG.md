# Flight Cameras — Changelog

## Session 9
- Camera auto-switch on throw removed (was overriding user camera selection)

## Session 8 — SIM-3
- FollowFlightCamera.cs: 4 modes (Follow, Side, Overhead, Overview)
- CM3 API (CinemachineCamera, not CM2 VirtualCamera)
- VCam_Follow: manually positioned each frame, no procedural components
- VCam_Side + VCam_Overhead: CinemachineFollow + RotationComposer
- Camera switching via vcam.Priority = int
