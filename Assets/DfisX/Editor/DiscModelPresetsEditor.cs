// Place this file in Assets/DfisX/Editor/
// It does NOT need an asmdef — Unity automatically compiles files in Editor/ folders
// as editor-only code.

using UnityEngine;
using UnityEditor;
using System.IO;
using DfisX;

namespace DfisXEditor
{
    public static class DiscModelPresetsEditor
    {
        [MenuItem("Assets/Create/DfisX/Create All Preset Discs")]
        public static void CreateAllPresetDiscs()
        {
            // Save into whichever folder is selected in the Project window,
            // falling back to Assets/Resources/DiscModels/
            string folder = GetSelectedFolder();

            int created = 0;
            int skipped = 0;

            foreach (DiscModel disc in DiscModelPresets.All())
            {
                string assetName = $"{disc.manufacturer}_{disc.moldName}";
                string path      = $"{folder}/{assetName}.asset";

                if (AssetDatabase.LoadAssetAtPath<DiscModel>(path) != null)
                {
                    Debug.Log($"[DfisX] Skipped '{assetName}' — already exists at {path}");
                    skipped++;
                    continue;
                }

                AssetDatabase.CreateAsset(disc, path);
                created++;
                Debug.Log($"[DfisX] Created '{assetName}' at {path}");
            }

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            EditorUtility.DisplayDialog(
                "DfisX Preset Discs",
                $"Done!\n\n{created} disc(s) created.\n{skipped} skipped (already existed).\n\nSaved to: {folder}",
                "OK");
        }

        [MenuItem("Assets/Create/DfisX/Create All Preset Discs", validate = true)]
        public static bool CreateAllPresetDiscsValidate()
        {
            // Only show the menu item when a folder is selected
            return Selection.activeObject != null;
        }

        // ---------------------------------------------------------------
        // Also add a menu item to auto-populate a selected DiscModelLibrary
        // with the preset discs that match its slots by disc type.
        // ---------------------------------------------------------------
        [MenuItem("Assets/Create/DfisX/Auto-populate Selected DiscModelLibrary")]
        public static void AutoPopulateLibrary()
        {
            DiscModelLibrary lib = Selection.activeObject as DiscModelLibrary;
            if (lib == null)
            {
                EditorUtility.DisplayDialog("DfisX", 
                    "Select a DiscModelLibrary asset first, then run this menu item.", "OK");
                return;
            }

            // Search project for all DiscModel assets
            string[] guids = AssetDatabase.FindAssets("t:DiscModel");
            DiscModel[] allDiscs = new DiscModel[guids.Length];
            for (int i = 0; i < guids.Length; i++)
                allDiscs[i] = AssetDatabase.LoadAssetAtPath<DiscModel>(
                    AssetDatabase.GUIDToAssetPath(guids[i]));

            int assigned = 0;

            // Match by disc type and stability to the library slots
            lib.fallback   = FindDisc(allDiscs, "Midrange", "Stable")
                          ?? FindDisc(allDiscs, null, null);

            lib.putter     = FindDisc(allDiscs, "Putter",   "Stable");
            lib.putterOS   = FindDisc(allDiscs, "Putter",   "Overstable");
            lib.putterUS   = FindDisc(allDiscs, "Putter",   "Understable");
            lib.midrange   = FindDisc(allDiscs, "Midrange", "Stable");
            lib.midrangeOS = FindDisc(allDiscs, "Midrange", "Overstable");
            lib.midrangeUS = FindDisc(allDiscs, "Midrange", "Understable");
            lib.fairway    = FindDisc(allDiscs, "Fairway",  "Stable");
            lib.fairwayOS  = FindDisc(allDiscs, "Fairway",  "Overstable");
            lib.fairwayUS  = FindDisc(allDiscs, "Fairway",  "Understable");
            lib.driver     = FindDisc(allDiscs, "Driver",   "Stable");
            lib.driverOS   = FindDisc(allDiscs, "Driver",   "Overstable");
            lib.driverUS   = FindDisc(allDiscs, "Driver",   "Understable");

            // Also populate the Discs list with everything found
            lib.discs.Clear();
            lib.discs.AddRange(allDiscs);

            EditorUtility.SetDirty(lib);
            AssetDatabase.SaveAssets();

            // Count non-null assignments
            assigned += lib.putter     != null ? 1 : 0;
            assigned += lib.putterOS   != null ? 1 : 0;
            assigned += lib.putterUS   != null ? 1 : 0;
            assigned += lib.midrange   != null ? 1 : 0;
            assigned += lib.midrangeOS != null ? 1 : 0;
            assigned += lib.midrangeUS != null ? 1 : 0;
            assigned += lib.fairway    != null ? 1 : 0;
            assigned += lib.fairwayOS  != null ? 1 : 0;
            assigned += lib.fairwayUS  != null ? 1 : 0;
            assigned += lib.driver     != null ? 1 : 0;
            assigned += lib.driverOS   != null ? 1 : 0;
            assigned += lib.driverUS   != null ? 1 : 0;

            EditorUtility.DisplayDialog("DfisX",
                $"Library populated!\n{assigned} slot(s) assigned.\n{allDiscs.Length} disc(s) added to Discs list.",
                "OK");
        }

        [MenuItem("Assets/Create/DfisX/Auto-populate Selected DiscModelLibrary", validate = true)]
        public static bool AutoPopulateLibraryValidate()
        {
            return Selection.activeObject is DiscModelLibrary;
        }

        // ---------------------------------------------------------------
        // Helpers
        // ---------------------------------------------------------------
        static string GetSelectedFolder()
        {
            string path = AssetDatabase.GetAssetPath(Selection.activeObject);

            if (string.IsNullOrEmpty(path))
                path = "Assets/Resources/DiscModels";
            else if (!Directory.Exists(path))
                path = Path.GetDirectoryName(path);

            // Ensure folder exists
            if (!Directory.Exists(path))
                Directory.CreateDirectory(path);

            return path;
        }

        static DiscModel FindDisc(DiscModel[] discs, string discType, string stability)
        {
            foreach (var d in discs)
            {
                if (d == null) continue;
                bool typeMatch      = discType  == null || d.discType  == discType;
                bool stabilityMatch = stability == null || d.stability == stability;
                if (typeMatch && stabilityMatch) return d;
            }
            return null;
        }
    }
}
