#ifndef DVD_DVISEST_ESTIMATE_HPP
#define DVD_DVISEST_ESTIMATE_HPP

#include <stdint.h>
#include <string>

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

// define measurement and state structures
struct pos_vel_var_state_t
{
  double      VEC3(pos);    // position 
  double      VEC3(vel);    // velocity
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
};

// Init Kalman filter states and measurement queues
bool dvd_DvisEst_estimate_init(cv::String gnd_plane_file);

// If we have recent tag detections, suppress frame skipping
// expires after a long enough hiatus of detections (if the filter isn't active that is)
void dvd_DvisEst_estimate_set_tags_detected(bool tags_detected);

bool dvd_DvisEst_estimate_get_tags_detected(void);

// check whether the estimation has completed
bool dvd_DvisEst_estimate_complete(void);

// Indicate that a new frame has been received, and is currently in processing
// reserve a slot in the incoming measurement queue so that measurements are processed in the correct order
// return slot #
bool dvd_DvisEst_estimate_reserve_measurement_slot(uint32_t frame_id, uint8_t * slot_id, uint16_t skipped_frames);

// Perhaps AprilTag detection failed? cancel our slot reservation
void dvd_DvisEst_estimate_cancel_measurement_slot(uint8_t slot_id, bool popped_frame);

// Add the actual measurement output to a previously reserved slot in the incoming queue
void dvd_DvisEst_estimate_fulfill_measurement_slot(uint8_t slot_id, dvd_DvisEst_kf_meas_t * kf_meas);

// Transform Apriltag measurement into KF disc measurement (includes ground plane transformation)
void dvd_DvisEst_estimate_transform_measurement(cv::Matx33d R_CD, cv::Matx31d T_CD, dvd_DvisEst_kf_meas_t * kf_meas);

// Run the Kalman Filter
void dvd_DvisEst_estimate_process_filter(void);

// Join Kalman Filter thread
void dvd_DvisEst_estimate_end_filter(void);

#endif // DVD_DVISEST_ESTIMATE_HPP
