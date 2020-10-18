#ifndef DVD_DVISEST_ESTIMATE_HPP
#define DVD_DVISEST_ESTIMATE_HPP

#include <stdint.h>
#include <string>
#include <atomic>

// OpenCV stuff
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// Disc Stuff
#include <dvd_DvisEst_maths.hpp>
#include <disc_layouts.hpp>

// define a few physics maximums to help our initial estimates along
#define MAX_LIN_VEL (37.0)  // m/s
#define MAX_ANG_VEL (150.0) // rad/s

#define KF_EST_STAGE_MEAS_COLLECT (0)
#define KF_EST_STAGE_READY        (1)
#define KF_EST_STAGE_PRIME        (2)
#define KF_EST_STAGE_ACTIVE       (3)
#define KF_EST_STAGE_COMPLETE     (4)

#define KF_EST_MAX_MEAS_FRAMES (500)

extern std::atomic<bool> gv_force_continuous_mode;
extern std::atomic<bool> gv_force_complete_threads;

// define measurement and state structures
struct pos_vel_var_state_t
{
  double      pos;          // position 
  double      vel;          // velocity
  double      MAT2X2(var);  // covariance matrix
};

// X is defined as positive toward the throw direction (toward screen)
// Y is defined as to the right
// Z is defined as 'positive up'
// origin is at the ground plane point
struct dvd_DvisEst_kf_meas_t
{
  uint64_t    timestamp_ns;         // Timestamp reported by camera, normalized to our uptime, nanoseconds
  uint32_t    frame_id;             // Frame ID reported by camera, sequential. We'll use this to check against the reserved measurement slots
  double      VEC3(lin_xyz_pos);    // Linear XYZ position measurement (m)
  double      VEC3(ang_hps_pos);    // Angular Hyzer, Pitch, Spin measurement (rad) (recall the special disc-frame definition for spin position here)
  DiscIndex   disc_index;           // DiscIndex enum
  uint8_t     player;               // player num lookup
};

struct dvd_DvisEst_kf_state_t
{
  uint64_t    timestamp_ns;              // Timestamp reported by camera, nanoseconds
  pos_vel_var_state_t     VEC3(lin_xyz); // Linear XYZ states: position (m), velocity (m/s), and covariance matrices
  pos_vel_var_state_t     VEC3(ang_hps); // Angular Hyzer, Pitch, Spin states: position (rad), velocity (rad/s), and covariance matrices
  double      wobble_mag;
  DiscIndex   disc_index;                // DiscIndex enum
};

// Init Kalman filter states and measurement queues
bool dvd_DvisEst_estimate_init(const bool kflog);

bool dvd_DvisEst_estimate_set_ground_plane_file(const std::string gnd_plane_filepath);
bool dvd_DvisEst_estimate_set_log_dir(const std::string log_dir);
std::string dvd_DvisEst_estimate_get_log_dir(void);

// If we have recent tag detections, suppress frame skipping
// expires after a long enough hiatus of detections (if the filter isn't active that is)
void dvd_DvisEst_estimate_set_tags_detected(bool tags_detected, uint32_t frame_id);

bool dvd_DvisEst_estimate_get_tags_detected(void);

// get stage of KF
uint8_t dvd_DvisEst_get_estimate_stage(void);
bool dvd_DvisEst_estimate_complete(void);

// Indicate that a new frame has been received, and is currently in processing
// reserve a slot in the incoming measurement queue so that measurements are processed in the correct order
// return slot #
bool dvd_DvisEst_estimate_reserve_measurement_slot(uint32_t frame_id, uint8_t * slot_id, uint16_t skipped_frames);

// Perhaps AprilTag detection failed? cancel our slot reservation
void dvd_DvisEst_estimate_cancel_measurement_slot(const uint8_t slot_id, const bool meas_mode, const uint32_t frame_id);

// Add the actual measurement output to a previously reserved slot in the incoming queue
void dvd_DvisEst_estimate_fulfill_measurement_slot(const uint8_t slot_id, const uint32_t frame_id, dvd_DvisEst_kf_meas_t * kf_meas);

// Transform Apriltag measurement into KF disc measurement (includes ground plane transformation)
void dvd_DvisEst_estimate_transform_measurement(cv::Matx33d R_CD, cv::Matx31d T_CD, dvd_DvisEst_kf_meas_t * kf_meas);

// Update groundplane during ground plane setting runtime
void dvd_DvisEst_estimate_update_groundplane(cv::Matx33d R_CG, cv::Matx31d T_CG);

// Run the Kalman Filter
void dvd_DvisEst_estimate_process_filter(void);

// get output
bool dvd_DvisEst_estimate_get_ideal_output_state(dvd_DvisEst_kf_state_t * kf_state);

// wrap up logging files etc.
void dvd_DvisEst_estimate_complete_filter(void);

// Join Kalman Filter thread
void dvd_DvisEst_estimate_end_filter(void);

#endif // DVD_DVISEST_ESTIMATE_HPP
