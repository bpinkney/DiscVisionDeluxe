using UnityEngine;
using System.Collections.Generic;

namespace DfisX
{
    /// <summary>
    /// Holds references to all available DiscModel assets.
    /// Replaces the disc_object_array static vector in disc_params.hpp.
    ///
    /// Create one instance via: Assets → Create → DfisX → Disc Model Library
    /// Then populate the list by dragging in DiscModel assets.
    ///
    /// Look up by name: DiscModelLibrary.Find("Destroyer")
    /// Look up by DiscIndex enum: DiscModelLibrary.FindByIndex(DiscIndex.DRIVER_OS)
    /// </summary>
    [CreateAssetMenu(fileName = "DiscModelLibrary", menuName = "DfisX/Disc Model Library")]
    public class DiscModelLibrary : ScriptableObject
    {
        [Tooltip("All available disc models. Drag DiscModel assets here.")]
        public List<DiscModel> discs = new List<DiscModel>();

        // ---- DiscIndex → DiscModel mapping ----
        // Mirrors the switch(disc_index) block in DfisX.cpp new_throw().
        // Assign your preferred mold for each flight category.

        [Header("DiscIndex → DiscModel mapping")]
        [Tooltip("DiscIndex.NONE — fallback disc")]
        public DiscModel fallback;
        public DiscModel putter;
        public DiscModel putterOS;
        public DiscModel putterUS;
        public DiscModel midrange;
        public DiscModel midrangeOS;
        public DiscModel midrangeUS;
        public DiscModel fairway;
        public DiscModel fairwayOS;
        public DiscModel fairwayUS;
        public DiscModel driver;
        public DiscModel driverOS;
        public DiscModel driverUS;
        public DiscModel special;
        public DiscModel groundplane;

        // ---- Lookup Methods ----

        /// <summary>
        /// Find a disc by exact mold name (case-sensitive).
        /// Returns null if not found.
        /// </summary>
        public DiscModel Find(string moldName)
        {
            return discs.Find(d => d != null && d.moldName == moldName);
        }

        /// <summary>
        /// Find a disc by mold name, falling back to <paramref name="fallbackModel"/> if missing.
        /// Logs a warning in the Editor so missing entries are easy to spot.
        /// </summary>
        public DiscModel FindOrFallback(string moldName, DiscModel fallbackModel = null)
        {
            var result = Find(moldName);
            if (result == null)
            {
                Debug.LogWarning($"[DfisX] DiscModel '{moldName}' not found in library. " +
                                 "Using fallback.");
                return fallbackModel != null ? fallbackModel : this.fallback;
            }
            return result;
        }

        /// <summary>
        /// Map a dvd_DvisEst DiscIndex enum to a DiscModel.
        /// Mirrors the switch statement in DfisX.cpp.
        /// </summary>
        public DiscModel FindByDiscIndex(DiscLayoutIndex index)
        {
            switch (index)
            {
                case DiscLayoutIndex.PUTTER:      return putter     ?? fallback;
                case DiscLayoutIndex.PUTTER_OS:   return putterOS   ?? fallback;
                case DiscLayoutIndex.PUTTER_US:   return putterUS   ?? fallback;
                case DiscLayoutIndex.MIDRANGE:    return midrange   ?? fallback;
                case DiscLayoutIndex.MIDRANGE_OS: return midrangeOS ?? fallback;
                case DiscLayoutIndex.MIDRANGE_US: return midrangeUS ?? fallback;
                case DiscLayoutIndex.FAIRWAY:     return fairway    ?? fallback;
                case DiscLayoutIndex.FAIRWAY_OS:  return fairwayOS  ?? fallback;
                case DiscLayoutIndex.FAIRWAY_US:  return fairwayUS  ?? fallback;
                case DiscLayoutIndex.DRIVER:      return driver     ?? fallback;
                case DiscLayoutIndex.DRIVER_OS:   return driverOS   ?? fallback;
                case DiscLayoutIndex.DRIVER_US:   return driverUS   ?? fallback;
                case DiscLayoutIndex.SPECIAL:     return special    ?? fallback;
                case DiscLayoutIndex.GROUNDPLANE:
                case DiscLayoutIndex.GROUNDPLANE_BIG:
                    return groundplane ?? fallback;
                default:
                    return fallback;
            }
        }
    }

    /// <summary>
    /// C# mirror of the DiscIndex enum from disc_layouts.hpp.
    /// Used to map dvd_DvisEst detections to DfisX disc models.
    /// </summary>
    public enum DiscLayoutIndex
    {
        NONE            = 0,
        GROUNDPLANE     = 1,
        GROUNDPLANE_BIG = 2,
        PUTTER          = 3,
        PUTTER_OS       = 4,
        PUTTER_US       = 5,
        MIDRANGE        = 6,
        MIDRANGE_OS     = 7,
        MIDRANGE_US     = 8,
        FAIRWAY         = 9,
        FAIRWAY_OS      = 10,
        FAIRWAY_US      = 11,
        DRIVER          = 12,
        DRIVER_OS       = 13,
        DRIVER_US       = 14,
        SPECIAL         = 15
    }
}
