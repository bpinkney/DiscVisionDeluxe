#include <dvd_DvisEst_estimate.hpp>
#include <dvd_DvisEst_image_capture.hpp>

#include <iostream>
#include <vector>
#include <cstring>
#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>

// Timer stuff
#include <unistd.h>
#include <chrono>
#define _BSD_SOURCE
#include <sys/time.h>
#include <stack>
#include <ctime>

// threading stuff
#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <atomic>

#include <exception>
#include <typeinfo>
#include <stdexcept>

// More opencv stuff
#include <opencv2/core/matx.hpp>

using namespace std;
using namespace cv;

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

#define KF_FILTER_PRED_DT_NS (1000000) // 1.0 ms, 1000Hz for now
// if the filter isn't active, and we havent had a tag detection for at least this long
// allow frame skips again
#define KF_FILTER_TAG_DETECT_RESET_TIME_NS (0.25 * 1000000000) // too long for now

//Concatenate preprocessor tokens x and y after macro-expanding them.
/*#define JOIN_NX(x,y) (x##y)
#define JOIN(x,y) JOIN_NX(x,y)
#define MEAS_QUEUE_STATUS(y) JOIN(sv_meas_queue_status_,y)
#define MEAS_QUEUE_MEAS(y)   JOIN(sv_meas_queue_meas_,y)*/

// should this match the thread count in the apriltag threadpool? probably not if we can't process frames fast enough
#define MEAS_QUEUE_SIZE (200)
std::vector<std::atomic<uint8_t>>  sv_meas_queue_status(MEAS_QUEUE_SIZE);
std::vector<dvd_DvisEst_kf_meas_t> sv_meas_queue_meas(MEAS_QUEUE_SIZE);
#define MEAS_QUEUE_STATUS(y) (sv_meas_queue_status[y])
#define MEAS_QUEUE_MEAS(y)   (sv_meas_queue_meas[y])

// current index of measurement queue (atomic since we want to observe it from multiple threads)
std::atomic<uint8_t> sv_meas_queue_read_idx      (0);
std::atomic<uint8_t> sv_meas_queue_write_idx     (0);

// are we getting apriltag detections? we may wish to suppress frame skips for now
std::atomic<bool> sv_kf_estimate_tags_detected (false);
std::atomic<uint64_t> sv_kf_estimate_tag_detect_time_ns (0);

// is the estimate ready? we can stop the detection and capture threads then
std::atomic<bool> sv_kf_estimate_complete (false);

// filter thread
std::thread kf_process_filter;

// ground plane transformation
cv::Matx33d R_CG = cv::Matx33d(0,0,0,0,0,0,0,0,0);
cv::Matx31d T_CG = cv::Matx31d(0,0,0);

// Kalman Filter statics
dvd_DvisEst_kf_state_t sv_kf_state;

std::atomic<uint32_t> meas_count_populated (0);
std::atomic<uint32_t> meas_count_empty     (0);

// Static Funcs
static void kf_meas_update_step(void);
static void kf_prediction_step(void);
static void angle_hyzer_pitch_spin_from_R(double * hyzer_angle, double * pitch_angle, double * spin_angle, cv::Matx33d R_GD);
static void kf_get_kalman_gain(const double S2dm, const double MAT2X2(P), double VEC2(K));
static void kf_apply_meas_update_state(const double VEC2(K), const double innovation, double * pos_state, double * vel_state);
static void kf_apply_meas_update_var(const double VEC2(K), double MAT2X2(P));
static void kf_prediction_state(const double dt, double * pos_state, double * vel_state);
static void kf_prediction_var(const double S2dp, const double S2vp, const double dt, double MAT2X2(P));

static uint8_t get_next_meas_queue_idx(uint8_t start_idx)
{
  uint8_t next_idx = start_idx + 1;
  if(next_idx > MEAS_QUEUE_SIZE - 1)
  {
    next_idx = 0;
  }
  return next_idx;
}

// Get time stamp in nanoseconds.
static uint64_t nanos()
{
  uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::
          now().time_since_epoch()).count();
  return ns; 
}

// Get boot-time stamp in nanoseconds.
static uint64_t uptime_get_ns()
{
  static uint64_t start_time_ns = 0;
  if(start_time_ns == 0)
  {
    start_time_ns = nanos();
  }

  return (nanos() - start_time_ns); 
}

// timing and profiling
static uint64_t profile_start_time_ns = 0;
void tic() {
  profile_start_time_ns = nanos();
}
void toc() {
  const uint64_t profile_end_time_ns = nanos();
  double s_elapsed = (profile_end_time_ns - profile_start_time_ns) * 0.000000001;
  std::cerr << "Time elapsed: "
            << s_elapsed * 1000
            << " ms ("
            << 1.0/s_elapsed * (int)(meas_count_populated + meas_count_empty)
            << " Hz for "
            << (int)(meas_count_populated + meas_count_empty)
            << " samples). ["
            << (int)(meas_count_populated)
            << "/"
            << (int)(meas_count_empty)
            << "] valid/empty meas"
            << std::flush;

/*  // why is this necessary? do your job flush...
  int i;
  for(i=0;i<10;i++)
  {
    std::cerr << std::endl;
  }*/
}

// Init Kalman filter states and measurement queues
bool dvd_DvisEst_estimate_init(cv::String gnd_plane_file)
{
  cerr << "Call dvd_DvisEst_estimate_init" << endl;

  // init structs to zero
  memset(&sv_kf_state, 0, sizeof(dvd_DvisEst_kf_state_t));

  int i;
  for(i = 0; i < MEAS_QUEUE_SIZE; i++)
  {
    MEAS_QUEUE_STATUS(i) = MEAS_QUEUE_STATUS_AVAILABLE;
    memset(&MEAS_QUEUE_MEAS(i), 0, sizeof(dvd_DvisEst_kf_meas_t));
  }

  // load ground plane transformation
  FileStorage fs;
  fs.open(gnd_plane_file, FileStorage::READ);
  if (!fs.isOpened())
  {
    cerr << "Failed to open gnd_plane_file: " << gnd_plane_file << endl;
    return false;
  }

  FileNode R_CG_fn = fs["R_CG"];
  FileNode T_CG_fn = fs["T_CG"];

  try
  {
    // looks like we can just set these directly due to contiguous memory
    R_CG = R_CG_fn.mat();
    T_CG = T_CG_fn.mat();
    //cv::Mat R_CG_mat = R_CG_fn.mat();
    //cv::Mat T_CG_mat = T_CG_fn.mat();

    /*int j;
    for(i = 0; i < 3; i++)
    {      
      T_CG(i, 0) = T_CG_mat.at<double>(i, 0);
      for(j = 0; j < 3; j++)
      {
        R_CG(i, j) = R_CG_mat.at<double>(i, j);
      }
    }*/
  }
  catch (...) 
  {
    std::exception_ptr p = std::current_exception();
    std::cerr <<(p ? p.__cxa_exception_type()->name() : "null") << std::endl;
    exit(1);
  }

  return true;
}

void dvd_DvisEst_estimate_set_tags_detected(bool tags_detected)
{
  sv_kf_estimate_tags_detected = tags_detected;
  if(tags_detected)
  {
    sv_kf_estimate_tag_detect_time_ns = uptime_get_ns();
  }
}

bool dvd_DvisEst_estimate_get_tags_detected()
{
  return sv_kf_estimate_tags_detected;
}

bool dvd_DvisEst_estimate_complete()
{
  return sv_kf_estimate_complete;
}

// Indicate that a new frame has been received, and is currently in processing
// reserve a slot in the incoming measurement queue so that measurements are processed in the correct order
bool dvd_DvisEst_estimate_reserve_measurement_slot(uint32_t frame_id, uint8_t * slot_id, uint16_t skipped_frames)
{
  // increment skipped frames count for book-keeping
  meas_count_empty += skipped_frames;

  // TODO: use frame_id to help with consumption order?

  // only give out next slot if it is available
  // only give out next slot to preserve adjacency
  if(MEAS_QUEUE_STATUS(sv_meas_queue_write_idx) == MEAS_QUEUE_STATUS_AVAILABLE)
  {
    (*slot_id) = sv_meas_queue_write_idx;
  }

  uint8_t next_idx = get_next_meas_queue_idx(sv_meas_queue_write_idx);
  if(MEAS_QUEUE_STATUS(next_idx) == MEAS_QUEUE_STATUS_AVAILABLE)
  {
    (*slot_id) = next_idx;
    sv_meas_queue_write_idx = next_idx;
    return true;
  }
  return false;
}
// Perhaps AprilTag detection failed? cancel our slot reservation
void dvd_DvisEst_estimate_cancel_measurement_slot(uint8_t slot_id, bool popped_frame)
{
  // free measurement slot
  //cerr << "Cancel measurement in slot " << (int)slot_id << endl;
  MEAS_QUEUE_STATUS(slot_id) = MEAS_QUEUE_STATUS_AVAILABLE;

  // indicate that measurement returned empty
  if(popped_frame)
  {
    meas_count_empty++;
  }
}

// Add the actual measurement output to a previously reserved slot in the incoming queue
void dvd_DvisEst_estimate_fulfill_measurement_slot(uint8_t slot_id, dvd_DvisEst_kf_meas_t * kf_meas)
{
  // add measurement to queue
  MEAS_QUEUE_MEAS(slot_id)   = (*kf_meas);
  // mark measurement slot populated and ready for consumption
  MEAS_QUEUE_STATUS(slot_id) = MEAS_QUEUE_STATUS_POPULATED;

  // indicate the measurement returned with a tag detection
  meas_count_populated++;
}

// Transform Apriltag measurement into KF disc measurement (includes ground plane transformation)
// Transform camera-to-disc transformation into groundplane-to-disc transformation
// Then compute the disc measurement
void dvd_DvisEst_estimate_transform_measurement(cv::Matx33d R_CD, cv::Matx31d T_CD, dvd_DvisEst_kf_meas_t * kf_meas)
{
  try
  {
  // rotate by base groundplane
  // R_GD = R_CG * R_CD;
  cv::Matx33d R_GD = R_CG * R_CD;

  // rotate xyz_CD positions into xyz_GD frame
  // subtract base xyz offset defined in CG frame
  cv::Matx31d T_GD = R_CG * (T_CD - T_CG);
  //T_GD = R_CG * T_GD;
  // invert the y axis per our axis defs
  T_GD(1, 0) = -T_GD(1, 0);

  // Get the angular measurement parameters
  angle_hyzer_pitch_spin_from_R(&kf_meas->ang_hps_pos[0], &kf_meas->ang_hps_pos[1], &kf_meas->ang_hps_pos[2], R_GD);

  kf_meas->lin_xyz_pos[0] = T_GD(0, 0);
  kf_meas->lin_xyz_pos[1] = T_GD(1, 0);
  kf_meas->lin_xyz_pos[2] = T_GD(2, 0);

  }
  catch (...) 
  {
    std::exception_ptr p = std::current_exception();
    std::cerr <<(p ? p.__cxa_exception_type()->name() : "null") << std::endl;
    exit(1);
  }
}

static void process_filter_thread(void)
{
  uint64_t last_loop_ns = 0;
  uint64_t now;

  bool latch_tic = false;
  bool latch_toc = false;

  while(!sv_kf_estimate_complete)
  {
    now = uptime_get_ns();

    // check whether to reset the frame skip lock-out
    if((now - sv_kf_estimate_tag_detect_time_ns) >= KF_FILTER_TAG_DETECT_RESET_TIME_NS && dvd_DvisEst_estimate_get_tags_detected())
    {
      std::cerr << "Time out apriltag detects!" << std::endl;
      dvd_DvisEst_estimate_set_tags_detected(false);
    }

    // if we got some cancellations, just skip them and move the read idx forward
    if(MEAS_QUEUE_STATUS(sv_meas_queue_read_idx) == MEAS_QUEUE_STATUS_AVAILABLE)
    {
      sv_meas_queue_read_idx = get_next_meas_queue_idx(sv_meas_queue_read_idx);        
    }    

    // run at KF_FILTER_PRED_DT_NS intervals
    if((now - last_loop_ns) >= KF_FILTER_PRED_DT_NS)
    {
      // for now (test), just assume the measurement is consumed right away
      // consume all ready adjacent measurements    
      while(MEAS_QUEUE_STATUS(sv_meas_queue_read_idx) == MEAS_QUEUE_STATUS_POPULATED)
      {
        // start timing profile block
        if(!latch_tic)
        {
          // tic
          tic();
          latch_tic = true;
        }

        cerr << "Consumed slot " << (int)sv_meas_queue_read_idx << ", FID: " << MEAS_QUEUE_MEAS(sv_meas_queue_read_idx).frame_id << " at " << MEAS_QUEUE_MEAS(sv_meas_queue_read_idx).timestamp_ns * 0.000001 << " ms (uptime = " << now * 0.000001 << " ms) ";
        cerr << "XYZ: [" << MEAS_QUEUE_MEAS(sv_meas_queue_read_idx).lin_xyz_pos[0] << ", " << MEAS_QUEUE_MEAS(sv_meas_queue_read_idx).lin_xyz_pos[1] << ", " << MEAS_QUEUE_MEAS(sv_meas_queue_read_idx).lin_xyz_pos[0] << "]" << endl;

        // perform the measurement update step
        kf_meas_update_step();

        MEAS_QUEUE_STATUS(sv_meas_queue_read_idx) = MEAS_QUEUE_STATUS_AVAILABLE;

        // be careful about this ++, atomics, amirite?
        sv_meas_queue_read_idx = get_next_meas_queue_idx(sv_meas_queue_read_idx);
      }

      // run the kalman filter prediction step
      kf_prediction_step();

      // after the queue is empty, let's mark the estimate complete so we can test thread joining
      // and do some profiling
      if(dvd_DvisEst_image_capture_image_capture_queue_empty() && latch_tic && !latch_toc)
      {
        toc();
        sv_kf_estimate_complete = true;
        latch_toc = true;
      }
    }
    else
    {
      // sleep for a bit so we don't busy poll
      usleep(1000);
    }
  }
  cerr << "Estimate Thread completed." << endl;
}

// Kalman Filter Functions

static void kf_meas_update_step()
{
  // TODO: Queue the measurement if the filter has not yet initialized

  // TODO: If we have already initialized the filter, consume the measurement
}

static void kf_prediction_step()
{

}

// Tranform GroundPlane-to-Disc rotation matrix into hyzer (worldframe X), pitch (worlframe Y), and spin (discframe Z) angles
static void angle_hyzer_pitch_spin_from_R(double * hyzer_angle, double * pitch_angle, double * spin_angle, cv::Matx33d R_GD)
{
  *hyzer_angle  = asin(R_GD(1,2));
  *pitch_angle  = asin(R_GD(0,2));
  *spin_angle   = atan2(R_GD(0,1), R_GD(0,0));
}

// measurement update functions
static void kf_get_kalman_gain(const double S2dm, const double MAT2X2(P), double VEC2(K))
{
  const double denom = max((S2dm + P[i2x2(0,0)]), CLOSE_TO_ZERO);

  K[0] = P[i2x2(0,0)] / denom;
  K[1] = P[i2x2(1,0)] / denom;
}

static void kf_apply_meas_update_state(const double VEC2(K), const double innovation, double * pos_state, double * vel_state)
{
  // innovation = (dk-dmk)
  (*pos_state) += K[0] * innovation;
  (*vel_state) += K[1] * innovation;
}

static void kf_apply_meas_update_var(const double VEC2(K), double MAT2X2(P))
{
  double p11 = P[i2x2(0,0)];
  double p12 = P[i2x2(0,1)];

  P[i2x2(0,0)] += (-K[0] * p11);
  P[i2x2(0,1)] += (-K[0] * p12);
  P[i2x2(1,0)] += (-K[1] * p11);
  P[i2x2(1,1)] += (-K[1] * p12);
}

// prediction/propagation functions
// Constant velocity assumption for the prediction step suffices
static void kf_prediction_state(const double dt, double * pos_state, double * vel_state)
{
  (*pos_state) += (*vel_state) * dt;
  //(*vel_state) += 0;
}

static void kf_prediction_var(const double S2dp, const double S2vp, const double dt, double MAT2X2(P))
{
  double p12 = P[i2x2(0,1)];
  double p21 = P[i2x2(1,0)];
  double p22 = P[i2x2(1,1)];

  double dte2 = dt*dt;

  P[i2x2(0,0)] += (dte2 * p22) + (dt * (p12 + p21)) + (S2dp);
  P[i2x2(0,1)] += (dt * p22);
  P[i2x2(1,0)] += (dt * p22);
  P[i2x2(1,1)] += (S2vp);
}

// end Kalman Filter Functions

void dvd_DvisEst_estimate_process_filter(void)
{
  kf_process_filter = std::thread(process_filter_thread);
}

void dvd_DvisEst_estimate_end_filter(void)
{
  kf_process_filter.join();
}

// remember that we want to load in a ground-plane, and use that to modify 
// the measurements coming from the apriltag threads!
