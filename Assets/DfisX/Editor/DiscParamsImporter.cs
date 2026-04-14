// Place this file in Assets/DfisX/Editor/
// Unity automatically compiles files in Editor/ folders as editor-only code.

using UnityEngine;
using UnityEditor;
using System.IO;
using System.Globalization;
using DfisX;

namespace DfisXEditor
{
    public class DiscParamsImporter : EditorWindow
    {
        string _csvPath      = "Assets/Resources/DiscModels/disc_params_pdga_final.csv";
        string _targetFolder = "Assets/Resources/DiscModels";
        bool   _skipExisting = true;

        [MenuItem("DfisX/Import CSV Disc Database")]
        public static void ShowWindow()
        {
            GetWindow<DiscParamsImporter>("Import Disc CSV");
        }

        void OnGUI()
        {
            GUILayout.Label("CSV Disc Database Importer", EditorStyles.boldLabel);
            EditorGUILayout.Space();

            _csvPath      = EditorGUILayout.TextField("Source CSV",     _csvPath);
            _targetFolder = EditorGUILayout.TextField("Target Folder",  _targetFolder);
            _skipExisting = EditorGUILayout.Toggle("Skip Existing",     _skipExisting);

            EditorGUILayout.Space();
            EditorGUILayout.HelpBox(
                "Creates one DiscModel .asset per CSV row.\n" +
                "rim_camber_shape='NONE' is preserved as-is (marks unknown shape for later population).\n" +
                "After import, select your DiscModelLibrary asset and run:\n" +
                "  Assets → Create → DfisX → Auto-populate Selected DiscModelLibrary",
                MessageType.Info);
            EditorGUILayout.Space();

            if (GUILayout.Button("Import", GUILayout.Height(30)))
                RunImport();
        }

        void RunImport()
        {
            string fullCsvPath = _csvPath;

            if (!File.Exists(fullCsvPath))
            {
                EditorUtility.DisplayDialog("Error", $"CSV not found:\n{fullCsvPath}", "OK");
                return;
            }

            // Ensure target folder exists
            if (!AssetDatabase.IsValidFolder(_targetFolder))
                Directory.CreateDirectory(_targetFolder);

            string[] lines  = File.ReadAllLines(fullCsvPath);
            int      total   = lines.Length - 1;  // minus header
            int      created = 0;
            int      skipped = 0;
            int      errors  = 0;

            try
            {
                // Line 0 is the header row — skip it
                for (int i = 1; i < lines.Length; i++)
                {
                    EditorUtility.DisplayProgressBar(
                        "Importing Discs",
                        $"Row {i}/{total}…",
                        (float)(i - 1) / total);

                    string line = lines[i].Trim();
                    if (string.IsNullOrEmpty(line)) continue;

                    string[] cols = line.Split(',');
                    if (cols.Length < 12)
                    {
                        Debug.LogWarning($"[DiscImporter] Row {i + 1}: expected ≥12 columns, got {cols.Length}. Skipping.");
                        errors++;
                        continue;
                    }

                    string moldName     = cols[0].Trim();
                    string manufacturer = cols[1].Trim();
                    string discType     = cols[2].Trim();
                    string stability    = cols[3].Trim();
                    string rimCamber    = cols[4].Trim();   // "NONE" preserved intentionally

                    if (!TryParseFloat(cols[5],  out float mass))          { LogParseError(i, "mass_kg");          errors++; continue; }
                    if (!TryParseFloat(cols[6],  out float radius))        { LogParseError(i, "radius_m");         errors++; continue; }
                    if (!TryParseFloat(cols[7],  out float rimWidth))      { LogParseError(i, "rim_width_m");      errors++; continue; }
                    if (!TryParseFloat(cols[8],  out float thickness))     { LogParseError(i, "thickness_m");      errors++; continue; }
                    if (!TryParseFloat(cols[9],  out float rimDepth))      { LogParseError(i, "rim_depth_m");      errors++; continue; }
                    if (!TryParseFloat(cols[10], out float rimCamberH))    { LogParseError(i, "rim_camber_h_m");   errors++; continue; }
                    if (!TryParseFloat(cols[11], out float domeHeight))    { LogParseError(i, "dome_height_m");    errors++; continue; }

                    // Build a filesystem-safe asset name
                    string safeName  = $"{Sanitize(manufacturer)}_{Sanitize(moldName)}";
                    string assetPath = $"{_targetFolder}/{safeName}.asset";

                    if (_skipExisting && AssetDatabase.LoadAssetAtPath<DiscModel>(assetPath) != null)
                    {
                        skipped++;
                        continue;
                    }

                    var disc = ScriptableObject.CreateInstance<DiscModel>();
                    disc.moldName        = moldName;
                    disc.manufacturer    = manufacturer;
                    disc.discType        = discType;
                    disc.stability       = stability;
                    disc.rimCamberShape  = rimCamber;
                    disc.mass            = mass;
                    disc.radius          = radius;
                    disc.rimWidth        = rimWidth;
                    disc.thickness       = thickness;
                    disc.rimDepth        = rimDepth;
                    disc.rimCamberHeight = rimCamberH;
                    disc.domeHeight      = domeHeight;

                    AssetDatabase.CreateAsset(disc, assetPath);
                    created++;
                }
            }
            finally
            {
                EditorUtility.ClearProgressBar();
            }

            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            string msg = $"Import complete.\n\n" +
                         $"  Created : {created}\n" +
                         $"  Skipped : {skipped}\n" +
                         $"  Errors  : {errors}\n\n" +
                         $"Saved to: {_targetFolder}";

            EditorUtility.DisplayDialog("Import Complete", msg, "OK");
            Debug.Log($"[DiscImporter] {msg}");
        }

        // ── Helpers ──────────────────────────────────────────────────────────────

        static bool TryParseFloat(string s, out float result) =>
            float.TryParse(s.Trim(), NumberStyles.Float, CultureInfo.InvariantCulture, out result);

        static void LogParseError(int row, string column) =>
            Debug.LogWarning($"[DiscImporter] Row {row + 1}: failed to parse '{column}'. Skipping.");

        static string Sanitize(string s)
        {
            foreach (char c in Path.GetInvalidFileNameChars())
                s = s.Replace(c.ToString(), "_");
            return s.Replace(' ', '_');
        }
    }
}
