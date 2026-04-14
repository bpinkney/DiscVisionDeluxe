using System;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace DiscVisionDeluxe
{
    /// <summary>
    /// Reads CSV logs produced by dvd_DvisEst (matching dvd_DvisEst_load_csv_log.m schema).
    ///
    /// CSV column layout (1-indexed, header row skipped):
    ///   1:  time_s
    ///   2:  pos_x (m)
    ///   3:  pos_y (m)
    ///   4:  pos_z (m)
    ///   5:  Qw
    ///   6:  Qx
    ///   7:  Qy
    ///   8:  Qz
    ///   9:  R00  10: R01  11: R02
    ///   12: R10  13: R11  14: R12
    ///   15: R20  16: R21  17: R22
    ///
    /// Angular measurements are derived from the R matrix using the same
    /// formulas as angle_hyzer_pitch_spin_from_R in dvd_DvisEst_estimate.cpp:
    ///   hyzer = asin(R[1,2])
    ///   pitch = asin(R[0,2])
    ///   spin  = atan2(R[0,1], R[0,0])
    /// </summary>
    public static class CsvLogReader
    {
        /// <summary>
        /// Load all measurements from a CSV log file.
        /// Returns null and logs an error if the file cannot be read.
        /// </summary>
        public static List<KFMeasurement> LoadFromFile(string filePath)
        {
            if (!File.Exists(filePath))
            {
                Debug.LogError($"[CsvLogReader] File not found: {filePath}");
                return null;
            }

            var measurements = new List<KFMeasurement>();

            try
            {
                string[] lines = File.ReadAllLines(filePath);
                // Skip header row
                for (int i = 1; i < lines.Length; i++)
                {
                    string line = lines[i].Trim();
                    if (string.IsNullOrEmpty(line)) continue;

                    var meas = ParseLine(line, (uint)i);
                    if (meas != null)
                        measurements.Add(meas);
                }
            }
            catch (Exception e)
            {
                Debug.LogError($"[CsvLogReader] Failed to read {filePath}: {e.Message}");
                return null;
            }

            Debug.Log($"[CsvLogReader] Loaded {measurements.Count} measurements from {Path.GetFileName(filePath)}");
            return measurements;
        }

        /// <summary>
        /// Load from a TextAsset (for files bundled in Resources/).
        /// </summary>
        public static List<KFMeasurement> LoadFromTextAsset(TextAsset asset)
        {
            if (asset == null)
            {
                Debug.LogError("[CsvLogReader] TextAsset is null.");
                return null;
            }

            var measurements = new List<KFMeasurement>();
            string[] lines = asset.text.Split('\n');

            for (int i = 1; i < lines.Length; i++)
            {
                string line = lines[i].Trim();
                if (string.IsNullOrEmpty(line)) continue;

                var meas = ParseLine(line, (uint)i);
                if (meas != null)
                    measurements.Add(meas);
            }

            Debug.Log($"[CsvLogReader] Loaded {measurements.Count} measurements from {asset.name}");
            return measurements;
        }

        // -------------------------------------------------------------------
        private static KFMeasurement ParseLine(string line, uint frameId)
        {
            string[] parts = line.Split(',');
            if (parts.Length < 17) return null;

            try
            {
                double timeS = ParseD(parts[0]);

                double posX = ParseD(parts[1]);
                double posY = ParseD(parts[2]);
                double posZ = ParseD(parts[3]);

                // R matrix (columns 9-17, 0-indexed 8-16)
                double r00 = ParseD(parts[8]);
                double r01 = ParseD(parts[9]);
                double r02 = ParseD(parts[10]);
                // R10, R11 not used for angle extraction but available
                double r12 = ParseD(parts[13]);

                // Angular measurement from R matrix
                // Mirrors angle_hyzer_pitch_spin_from_R in dvd_DvisEst_estimate.cpp
                double hyzer = Math.Asin(Clamp(r12, -1.0, 1.0));
                double pitch = Math.Asin(Clamp(r02, -1.0, 1.0));
                double spin  = Math.Atan2(r01, r00);

                // Convert time to nanoseconds for the KF
                ulong timestampNs = (ulong)(timeS * 1e9);

                return new KFMeasurement
                {
                    timestampNs = timestampNs,
                    frameId     = frameId,
                    linX        = posX,
                    linY        = posY,
                    linZ        = posZ,
                    angHyzer    = hyzer,
                    angPitch    = pitch,
                    angSpin     = spin,
                    discIndex   = DiscIndex.NONE,  // not in raw log; set externally if known
                    player      = 0
                };
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[CsvLogReader] Could not parse line {frameId}: {e.Message}");
                return null;
            }
        }

        private static double ParseD(string s) =>
            double.Parse(s.Trim(), System.Globalization.CultureInfo.InvariantCulture);

        private static double Clamp(double v, double min, double max) =>
            v < min ? min : v > max ? max : v;
    }
}
