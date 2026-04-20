# MiniMap — Bugs

## Resolved / Hard-Won Facts

### DiscVisionDeluxe.UI.asmdef Missing Unity.TextMeshPro Reference
Symptom: CS0246 on TMP types in MiniMap.cs.
Fix: add `Unity.TextMeshPro` to `DiscVisionDeluxe.UI.asmdef` references.
