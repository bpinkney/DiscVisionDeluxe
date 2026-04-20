# Disc Database — Design

## CSV Source
`Assets/Resources/DiscModels/disc_params_pdga_final.csv`

## CSV Columns
| Column | Type | Notes |
|---|---|---|
| mold_name | string | Disc model name (e.g. "Destroyer") |
| manufacturer | string | Manufacturer name (e.g. "Innova") |
| disc_type | string | Driver, Midrange, Putter, etc. |
| stability | string | Overstable, Neutral, Understable |
| rim_camber_shape | string | Flat, Concave, Convex, NONE |
| mass_kg | float | |
| radius_m | float | |
| rim_width_m | float | |
| thickness_m | float | |
| rim_depth_m | float | |
| rim_camber_h_m | float | |
| dome_height_m | float | |

`rim_camber_shape = NONE` means unknown. `DiscModel.ToBlittable()` treats anything not Concave or Convex as Flat (int=0) — graceful degradation.

## Import Tool
`DfisX → Import CSV Disc Database` from the Unity menu bar.
- Reads `disc_params_pdga_final.csv`
- Creates one `DiscModel .asset` per row under `Assets/Resources/DiscModels/`
- Uses `CultureInfo.InvariantCulture` for float parsing
- Skip-existing toggle (default: on)

After import: select `DiscModelLibrary` asset → `Assets → Create → DfisX → Auto-populate Selected DiscModelLibrary`.

## DiscModel ScriptableObject
Physical dimension fields only. All aero behaviour emerges from dimensions in `Daero.cs`. No per-disc speed/glide/turn/fade constants.

## DiscModelData (blittable mirror)
`DiscModel.ToBlittable()` produces a `DiscModelData` struct for the physics engine. Fields: `mass`, `radius`, `rimWidth`, `thickness`, `rimDepth`, `rimCamberHeight`, `domeHeight`, `rimCamberShape` (int).

## Disc Selection UI (ThrowParameterPanelController)
Four AND-combined filters:
- **Type** dropdown (Driver, Fairway Driver, Midrange, Putter)
- **Stability** dropdown (Overstable, Neutral, Understable)
- **Manufacturer** dropdown (dynamically populated from library, sorted)
- **Search** text field (case-insensitive; matches mold name or manufacturer; fires on keystroke)

If the currently selected disc is still in the filtered list after a filter change, the dropdown scrolls to it rather than resetting to index 0.

## Disc Recents
Last 5 thrown discs shown as pill buttons above the disc dropdown. `static List<DiscModel> _recentDiscs` — survives re-binds within a play session. Clicking a pill selects that disc directly, bypassing filters.

## Key Files
| File | Role |
|---|---|
| Assets/DfisX/Runtime/DiscModel.cs | ScriptableObject, ToBlittable() |
| Assets/DfisX/Runtime/DiscModelLibrary.cs | Holds full list + DiscLayoutIndex mapping |
| Assets/DfisX/Runtime/DiscModelPresets.cs | 11 named reference disc factory methods |
| Assets/DfisX/Editor/DiscParamsImporter.cs | CSV → .asset import tool |
| Assets/Resources/DiscModels/ | 1,272 .asset files + DiscModelLibrary.asset |
