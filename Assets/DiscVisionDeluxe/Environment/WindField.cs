using UnityEngine;
using DfisX;

namespace DiscVisionDeluxe
{
    /// <summary>
    /// Scene-level wind state. Owns direction/speed/gust parameters and converts them
    /// into a DiscEnvironment for DfisX. Referenced by DiscVisualizer, ThrowParameterPanelController,
    /// and WindIndicator — all three read from the same instance.
    /// </summary>
    public class WindField : MonoBehaviour
    {
        [Header("Wind")]
        [Tooltip("Compass bearing the wind blows FROM (0 = North, 90 = East, 180 = South, 270 = West)")]
        [Range(0f, 359f)]
        public float windDirectionDeg = 0f;

        [Tooltip("Horizontal wind speed in kph")]
        [Range(0f, 100f)]
        public float windSpeedKph = 0f;

        [Tooltip("Vertical wind component in m/s (positive = upward)")]
        [Range(-5f, 5f)]
        public float windVerticalMs = 0f;

        [Tooltip("Turbulence level — drives gust simulation in Daero")]
        public GustFactor gustFactor = GustFactor.ZeroDeadDiddly;

        [Header("Atmosphere")]
        [Tooltip("Air density kg/m³. ISA sea-level = 1.225; decreases with altitude.")]
        [Range(0.8f, 1.4f)]
        public float airDensity = 1.225f;

        public float WindSpeedMs    => windSpeedKph / 3.6f;
        public float WindSpeedKnots => windSpeedKph / 1.852f;

        /// <summary>8-point cardinal label for the direction wind comes FROM.</summary>
        public string CardinalDirection
        {
            get
            {
                float d      = ((windDirectionDeg % 360f) + 360f) % 360f;
                int   sector = Mathf.RoundToInt(d / 45f) % 8;
                return sector switch
                {
                    0 => "N",
                    1 => "NE",
                    2 => "E",
                    3 => "SE",
                    4 => "S",
                    5 => "SW",
                    6 => "W",
                    7 => "NW",
                    _ => "N"
                };
            }
        }

        /// <summary>
        /// Builds a DiscEnvironment from current wind state.
        /// Wind direction uses compass convention (FROM bearing):
        ///   DfisX frame X = forward (North), Y = right (East), Z = up.
        ///   Wind FROM bearing θ → velocity vector toward (θ + 180°).
        /// </summary>
        public DiscEnvironment BuildEnvironment()
        {
            float towardRad = (windDirectionDeg + 180f) * Mathf.Deg2Rad;
            float speedMs   = WindSpeedMs;
            float wx        = Mathf.Cos(towardRad) * speedMs;
            float wy        = Mathf.Sin(towardRad) * speedMs;

            return new DiscEnvironment
            {
                windVectorXYZ = new Unity.Mathematics.float3(wx, wy, windVerticalMs),
                gustFactor    = gustFactor,
                airDensity    = airDensity
            };
        }
    }
}
