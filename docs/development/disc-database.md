# Disc Database

The project includes 1,272 disc models imported from real PDGA-measured physical data.

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

`rim_camber_shape = NONE` means the shape is unknown. It is preserved as-is. `DiscModel.ToBlittable()` treats anything that is not `Concave` or `Convex` as Flat (int=0), so NONE entries degrade gracefully.

## Import Tool
Open `DfisX → Import CSV Disc Database` from the Unity menu bar.
- Reads `disc_params_pdga_final.csv`
- Creates one `DiscModel .asset` file per row under `Assets/Resources/DiscModels/`
- Uses `CultureInfo.InvariantCulture` for float parsing
- Skip-existing toggle (default: on)
- Progress bar shown during import

After import: select the `DiscModelLibrary` asset → `Assets → Create → DfisX → Auto-populate Selected DiscModelLibrary` to fill the 12 named type/stability slots and the full `discs` list.

## DiscModel ScriptableObject
Each disc is a `DiscModel` asset with physical dimension fields. All aerodynamic behaviour (turn, fade, glide) emerges from these dimensions in `Daero.cs`. There are no per-disc "speed", "glide", "turn", "fade" constants.

Known issue: `rimWidth` values in the imported data appear slightly high (Destroyer: 0.0245m, PDGA sheet: ~0.021m). All assets should eventually be audited against PDGA approval sheets.

## Disc Selection UI
`ThrowParameterPanelController` provides four AND-combined filters:
- **Type** dropdown (Driver, Fairway Driver, Midrange, Putter)
- **Stability** dropdown (Overstable, Neutral, Understable)
- **Manufacturer** dropdown (dynamically populated from library, sorted)
- **Search** text field (case-insensitive; matches mold name or manufacturer; fires on every keystroke)

If the currently selected disc is still in the filtered list after a filter change, the dropdown scrolls to it rather than resetting to index 0.

## Recents
The last 5 thrown discs are shown as pill buttons above the disc dropdown. Static list (`static List<DiscModel> _recentDiscs`) — survives re-binds within a play session. Clicking a pill selects that disc directly, bypassing filters.
