# Disc Database — Overview

## Purpose
Provides 1,272 real disc golf disc models with physically-measured PDGA dimensions. All aerodynamic behaviour emerges from these dimensions in `Daero.cs` — there are no hand-tuned "turn", "fade", or "glide" constants.

## Requirements
- Import all PDGA-measured disc models from CSV into Unity ScriptableObject assets
- Expose full library through a filtered, searchable selection UI
- Support fast access via recents and direct model references (not just DiscIndex enum)

## Architecture
- `DiscModel` ScriptableObject: one asset per disc, authored physical dimensions
- `DiscModelLibrary` ScriptableObject: holds the full list + 12 DiscIndex type/stability slots
- `DiscParamsImporter`: Editor tool that reads CSV → creates one .asset per row
- `ThrowParameterPanelController`: provides the selection UI (4 AND-combined filters + recents)

## Place in the Project
Consumed by `DfisX` (via `DiscModel.ToBlittable()`), `DiscVisualization` (mesh generation from dimensions), `UIThrowPanel` (selection UI), and `UIResultsPanel` (disc name / stats display). The `directModel` field on `DiscInitState` and `ThrowParameters` ensures the selected disc flows through the full pipeline.
