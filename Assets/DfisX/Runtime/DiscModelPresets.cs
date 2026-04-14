using UnityEngine;

namespace DfisX
{
    /// <summary>
    /// Factory methods to create DiscModel instances with values taken directly
    /// from disc_params.hpp.  Use these in Editor scripts or tests.
    ///
    /// Each method matches a well-tested disc from the C++ codebase; they are
    /// the same discs called out in the switch(disc_index) block in DfisX.cpp.
    /// </summary>
    public static class DiscModelPresets
    {
        // ---- helper ----
        static DiscModel Make(
            string mold, string mfr, string type, string stability, string camber,
            float mass, float radius, float rimWidth, float thickness,
            float rimDepth, float rimCamberH, float domeH)
        {
            var d               = ScriptableObject.CreateInstance<DiscModel>();
            d.moldName          = mold;
            d.manufacturer      = mfr;
            d.discType          = type;
            d.stability         = stability;
            d.rimCamberShape    = camber;
            d.mass              = mass;
            d.radius            = radius;
            d.rimWidth          = rimWidth;
            d.thickness         = thickness;
            d.rimDepth          = rimDepth;
            d.rimCamberHeight   = rimCamberH;
            d.domeHeight        = domeH;
            d.name              = $"{mfr}_{mold}";
            return d;
        }

        // ---- Tested reference discs (from disc_params.hpp) ----

        /// <summary>Innova Destroyer DX — Stable driver, reference overstable driver in DfisX.cpp</summary>
        public static DiscModel Destroyer() => Make(
            "Destroyer",     "Innova",    "Driver",   "Stable",      "Concave",
            mass: 0.170f,    radius: 0.10450f,
            rimWidth: 0.02450f, thickness: 0.01700f, rimDepth: 0.01200f,
            rimCamberH: 0.00375f, domeH: 0.00500f);

        /// <summary>Innova Shryke Star — Understable driver</summary>
        public static DiscModel Shryke() => Make(
            "Shryke",        "Innova",    "Driver",   "Understable", "Concave",
            mass: 0.171f,    radius: 0.10575f,
            rimWidth: 0.02350f, thickness: 0.01550f, rimDepth: 0.01100f,
            rimCamberH: 0.00425f, domeH: 0.00525f);

        /// <summary>Innova TeeBird Star — Stable fairway driver</summary>
        public static DiscModel TeeBird() => Make(
            "TeeBird",       "Innova",    "Fairway",  "Stable",      "NONE",
            mass: 0.175f,    radius: 0.10570f,
            rimWidth: 0.01780f, thickness: 0.01600f, rimDepth: 0.01130f,
            rimCamberH: 0.00621f, domeH: 0.00294f);

        /// <summary>Innova Wraith Star — Stable driver</summary>
        public static DiscModel Wraith() => Make(
            "Wraith",        "Innova",    "Driver",   "Stable",      "Concave",
            mass: 0.167f,    radius: 0.10500f,
            rimWidth: 0.02200f, thickness: 0.01800f, rimDepth: 0.01200f,
            rimCamberH: 0.00500f, domeH: 0.00600f);

        /// <summary>Innova Mako3 Champion — Stable midrange</summary>
        public static DiscModel Mako3() => Make(
            "Mako3",         "Innova",    "Midrange", "Stable",      "Convex",
            mass: 0.175f,    radius: 0.10850f,
            rimWidth: 0.01250f, thickness: 0.01700f, rimDepth: 0.01300f,
            rimCamberH: 0.00800f, domeH: 0.00075f);

        /// <summary>Discraft Buzzz Foil Pro-D — Stable midrange</summary>
        public static DiscModel BuzzzFoil() => Make(
            "Buzzz Foil",    "Discraft",  "Midrange", "Stable",      "Concave",
            mass: 0.180f,    radius: 0.11000f,
            rimWidth: 0.01300f, thickness: 0.01800f, rimDepth: 0.01300f,
            rimCamberH: 0.00700f, domeH: 0.00200f);

        /// <summary>Discraft Magnet Jawbreaker — Stable putter</summary>
        public static DiscModel Magnet() => Make(
            "Magnet",        "Discraft",  "Putter",   "Stable",      "NONE",
            mass: 0.173f,    radius: 0.10550f,
            rimWidth: 0.00900f, thickness: 0.02200f, rimDepth: 0.01400f,
            rimCamberH: 0.00750f, domeH: 0.00400f);

        /// <summary>Discraft Zone Jawbreaker — Overstable putter</summary>
        public static DiscModel Zone() => Make(
            "Zone",          "Discraft",  "Putter",   "Overstable",  "Flat",
            mass: 0.174f,    radius: 0.10700f,
            rimWidth: 0.01150f, thickness: 0.01650f, rimDepth: 0.01400f,
            rimCamberH: 0.00800f, domeH: 0.00000f);

        /// <summary>Innova Firebird Champion — Overstable driver</summary>
        public static DiscModel Firebird() => Make(
            "Firebird",      "Innova",    "Driver",   "Overstable",  "Concave",
            mass: 0.175f,    radius: 0.10600f,
            rimWidth: 0.01950f, thickness: 0.01700f, rimDepth: 0.01175f,
            rimCamberH: 0.00700f, domeH: 0.00300f);

        /// <summary>Innova Leopard3 G-Star — Understable fairway</summary>
        public static DiscModel Leopard3() => Make(
            "Leopard3",      "Innova",    "Fairway",  "Understable", "Concave",
            mass: 0.175f,    radius: 0.10560f,
            rimWidth: 0.01680f, thickness: 0.01510f, rimDepth: 0.01100f,
            rimCamberH: 0.00700f, domeH: 0.00250f);

        /// <summary>Discraft Luna Jawbreaker — Stable putter</summary>
        public static DiscModel Luna() => Make(
            "Luna",          "Discraft",  "Putter",   "Stable",      "NONE",
            mass: 0.170f,    radius: 0.10510f,
            rimWidth: 0.01080f, thickness: 0.01950f, rimDepth: 0.01430f,
            rimCamberH: 0.00404f, domeH: 0.00226f);

        // ---- All presets as an array ----

        /// <summary>
        /// Returns all preset discs. Useful for populating a DiscModelLibrary in the Editor.
        /// </summary>
        public static DiscModel[] All() => new[]
        {
            Destroyer(), Shryke(), TeeBird(), Wraith(), Mako3(),
            BuzzzFoil(), Magnet(), Zone(), Firebird(), Leopard3(), Luna()
        };
    }
}
