using UnityEngine;
using Unity.Cinemachine;

namespace DiscVisionDeluxe.FlightCameras
{
    /// <summary>
    /// Manages Cinemachine virtual cameras that track the disc during flight.
    ///
    /// Scene setup:
    ///   1. Add a CinemachineBrain to your Main Camera.
    ///   2. Create four child GameObjects under this one, each with a CinemachineCamera:
    ///        VCam_Overview   — static overview from behind the tee
    ///        VCam_Follow     — chases the disc from behind and above
    ///        VCam_Side       — fixed Y, disc moves left-to-right in frame
    ///        VCam_Overhead   — top-down, disc centred
    ///   3. Assign them in the Inspector below.
    ///   4. Assign discTransform (the disc mesh Transform).
    ///   5. Wire discVisualizer in the Inspector.
    ///
    /// Camera switches automatically on throw start / landing.
    /// Press C (or the UI button) to cycle modes manually.
    /// </summary>
    public class FollowFlightCamera : MonoBehaviour
    {
        // ── Inspector ────────────────────────────────────────────────────────

        [Header("References")]
        public DiscVisualizer discVisualizer;
        public Transform      discTransform;

        [Header("Virtual Cameras (assign CinemachineCamera components)")]
        public CinemachineCamera vcamOverview;
        public CinemachineCamera vcamFollow;
        public CinemachineCamera vcamSide;
        public CinemachineCamera vcamOverhead;

        [Header("Follow Camera (world-space, no rotation)")]
        [Tooltip("Distance behind the disc along -X (the throw axis).")]
        public float followDistance = 8f;

        [Tooltip("Height above the disc.")]
        public float followHeight = 2f;

        [Tooltip("Lateral offset from the disc (+ = right / +Z in Unity).")]
        public float followLateral = 0f;

        [Tooltip("Camera tilt in degrees. 0 = perfectly level, positive = angled slightly down.")]
        public float followTiltDeg = 5f;

        [Header("Input")]
        [Tooltip("Key to cycle through camera modes.")]
        public KeyCode cycleCameraKey = KeyCode.C;

        // ── Camera mode ──────────────────────────────────────────────────────

        public enum CameraMode { Overview, Follow, Side, Overhead }

        [Header("State")]
        public CameraMode currentMode = CameraMode.Follow;


        // ── Unity lifecycle ──────────────────────────────────────────────────

        void Start()
        {
            ApplyMode(currentMode);
        }

        void Update()
        {
            if (discVisualizer == null) return;

            // Manual cycle
            if (UnityEngine.Input.GetKeyDown(cycleCameraKey))
                CycleMode();

            // Update follow camera offset each frame so it stays behind the disc
            UpdateFollowTarget();
        }

        // ── Public API ───────────────────────────────────────────────────────

        public void SetMode(CameraMode mode)
        {
            currentMode = mode;
            ApplyMode(mode);
        }

        public void CycleMode()
        {
            int next = ((int)currentMode + 1) % System.Enum.GetValues(typeof(CameraMode)).Length;
            SetMode((CameraMode)next);
        }

        // ── Private ──────────────────────────────────────────────────────────

        void ApplyMode(CameraMode mode)
        {
            SetActive(vcamOverview, mode == CameraMode.Overview);
            SetActive(vcamFollow,   mode == CameraMode.Follow);
            SetActive(vcamSide,     mode == CameraMode.Side);
            SetActive(vcamOverhead, mode == CameraMode.Overhead);
        }

        void SetActive(CinemachineCamera vcam, bool active)
        {
            if (vcam == null) return;
            vcam.gameObject.SetActive(active);
        }

        void UpdateFollowTarget()
        {
            if (discTransform == null) return;

            // VCam_Follow: manually positioned in world space each frame.
            // Disc flies along +X, so "behind" = -X direction.
            // Rotation is fixed — looks along +X, tilted down by followTiltDeg.
            // Set Position Control = None and Rotation Control = None on VCam_Follow.
            if (vcamFollow != null)
            {
                Vector3 behindDisc = discTransform.position
                    + new Vector3(-followDistance, followHeight, followLateral);
                vcamFollow.transform.position = behindDisc;
                vcamFollow.transform.rotation = Quaternion.Euler(followTiltDeg, 90f, 0f);
            }

            // VCam_Side and VCam_Overhead: assign Tracking Target in Inspector.
            // Their CinemachineFollow offset drives the position.
        }
    }
}
