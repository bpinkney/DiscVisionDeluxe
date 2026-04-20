# UI Throw Panel — Bugs

## Resolved / Hard-Won Facts

### Camera Dropdown: CameraMode Enum vs UI Order Mismatch
`CameraMode` enum order (Overview=0, Follow=1, Side=2, Overhead=3) does not match UI choices order (Follow=0, Side=1, Overhead=2, Overview=3).
Always use `ModeToIndex()` / `IndexToMode()` in `ThrowParameterPanelController` — never cast directly.
Also: `dropdown.index = x` does not fire `RegisterValueChangedCallback`; call `SetMode()` explicitly on startup.
