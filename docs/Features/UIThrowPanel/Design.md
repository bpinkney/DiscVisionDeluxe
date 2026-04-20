# UI Throw Panel — Design

## Architecture
UI Toolkit (UXML/USS). `ThrowParameterPanelController` is a MonoBehaviour with UIDocument.

## Key State
- `_selectedModel` — single authoritative `DiscModel` ref for all physics builds. Set by dropdown OR recent-pill click.
- `RebuildDiscDropdown()` — applies 4 AND filters: type, stability, manufacturer, free-text search (case-insensitive, matches mold name or manufacturer).
- If `_selectedModel` still in filtered list after filter change: scroll to it (don't reset to index 0).
- Manufacturer filter: dynamically populated from `discModelLibrary.discs` (SortedSet<string>).
- Filter dropdowns: `compact-filter` USS class, 26px row height.

## Disc Recents
`static List<DiscModel> _recentDiscs` (max 5). Static — survives re-binds within play session.
Populated on each Launch via `AddToRecents(_selectedModel)` — deduplicates, newest first.
Pill buttons (`recent-btn` USS class). Active pill: `recent-btn--active` highlight.
`SelectRecent()`: sets `_selectedModel` directly, syncs main dropdown if disc visible in current filter.

## Aero Debug Sliders
Six global physics-model tuning constants — NOT per-disc parameters:
`cdEdge(0.6)`, `clCavity(45.0)`, `clCamber(1.0)`, `cavityEdgeExposedAreaFactor(1.0)`, `pitchingMomentCavityLiftOffset(0.042)`, `pitchingMomentCamberLiftOffset(0.15)`.

## Files
- `Assets/DiscVisionDeluxe/UI/ThrowParameterPanelController.cs`
- `Assets/UI/ThrowParameterPanel.uxml`
- `Assets/UI/ThrowParameterPanel.uss`
