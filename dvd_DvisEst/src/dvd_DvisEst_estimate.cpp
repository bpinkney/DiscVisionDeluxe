#include <dvd_DvisEst_estimate.hpp>

#include<iostream>

// include thread stuff
#include <mutex>
#include <atomic>

using namespace std;

// Define macros for our measurement queue
// Queue states are:
// - available -> No apriltag thread has claimed this slot yet
// - reserved  -> An apriltag thread has indicated that is is currently processing a measurement for this slot
// - populated -> Apriltag thread has returned with data, measurement is awaiting consumption by the Kalman Filter
#define MEAS_QUEUE_STATUS_AVAILABLE (0)
#define MEAS_QUEUE_STATUS_RESERVED  (1)
#define MEAS_QUEUE_STATUS_POPULATED (2)

// We need atomic arrays, but that will cause the compiler some sadness
// so we can cheat and use an eval-style macro to access our pseudo-array of atomics
// (this is undoubtably bad form, oh well, need ma loops)
// for now, let's just have 3 slots, we can expand this later if necessary pending how
// many concurrent apriltag threads we expect

//Concatenate preprocessor tokens x and y after macro-expanding them.
#define JOIN_NX(x,y) (x##y)
#define JOIN(x,y) JOIN_NX(x,y)

#define MEAS_QUEUE_STATUS(y) JOIN(meas_queue_status_,y)
std::atomic<uint8_t> meas_queue_status_0 (MEAS_QUEUE_STATUS_AVAILABLE);
std::atomic<uint8_t> meas_queue_status_1 (MEAS_QUEUE_STATUS_AVAILABLE);
std::atomic<uint8_t> meas_queue_status_2 (MEAS_QUEUE_STATUS_AVAILABLE);
#define MEAS_QUEUE_SIZE (3)

// Init Kalman filter states and measurement queues
void dvd_DvisEst_estimate_init()
{
  // test
  MEAS_QUEUE_STATUS(0) = MEAS_QUEUE_STATUS_RESERVED;

  int meas_queue_status_0_print_test = MEAS_QUEUE_STATUS(0);
  int meas_queue_status_1_print_test = MEAS_QUEUE_STATUS(1);

  cerr << "Queue status 0: " << meas_queue_status_0_print_test << ", Queue status 1: " << meas_queue_status_1_print_test << endl;
}

// Indicate that a new frame has been received, and is currently in processing
// reserve a slot in the incoming measurement queue so that measurements are processed in the correct order
// return slot #
uint8_t dvd_DvisEst_estimate_reserve_measurement_slot(uint32_t frame_id)
{

  return 0;
}
// Perhaps AprilTag detection failed? cancel our slot reservation
void dvd_DvisEst_estimate_cancel_measurement_slot(uint8_t slot_id)
{

}

// Add the actual measurement output to a previously reserved slot in the incoming queue
void dvd_DvisEst_estimate_fulfill_measurement_slot(uint8_t slot_id, dvd_DvisEst_kf_meas_t * kf_meas)
{

}
