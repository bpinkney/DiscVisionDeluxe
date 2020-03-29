#ifndef DVD_DVISEST_ESTIMATE_HPP
#define DVD_DVISEST_ESTIMATE_HPP

#include <stdint.h>
#include <dvd_DvisEst_maths.hpp>

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
  uint64_t    timestamp_ns;         // Timestamp reported by camera, nanoseconds
  uint32_t    frame_id;             // Frame ID reported by camera, sequential. We'll use this to check against the reserved measurement slots
  double      VEC3(lin_xyz_pos);    // Linear XYZ position measurement (m)
  double      VEC3(ang_hps_pos);    // Angular Hyzer, Pitch, Spin measurement (rad) (recall the special disc-frame definition for spin position here)
};

struct dvd_DvisEst_kf_state_t
{
  uint64_t    timestamp_ns;              // Timestamp reported by camera, nanoseconds
  pos_vel_var_state_t     VEC3(lin_xyz); // Linear XYZ states: position (m), velocity (m/s), and covariance matrices
  pos_vel_var_state_t     VEC3(ang_hps); // Angular Hyzer, Pitch, Spin states: position (rad), velocity (rad/s), and covariance matrices
};

// Init Kalman filter states and measurement queues
void dvd_DvisEst_estimate_init();
// Indicate that a new frame has been received, and is currently in processing
// reserve a slot in the incoming measurement queue so that measurements are processed in the correct order
// return slot #
uint8_t dvd_DvisEst_estimate_reserve_measurement_slot(uint32_t frame_id);
// Perhaps AprilTag detection failed? cancel our slot reservation
void dvd_DvisEst_estimate_cancel_measurement_slot(uint8_t slot_id);
// Add the actual measurement output to a previously reserved slot in the incoming queue
void dvd_DvisEst_estimate_fulfill_measurement_slot(uint8_t slot_id, dvd_DvisEst_kf_meas_t * kf_meas);

#endif // DVD_DVISEST_ESTIMATE_HPP
