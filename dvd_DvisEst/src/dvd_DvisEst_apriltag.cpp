#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>

// CPU AprilTags
#include "apriltag.h"
#include <tag36h11.h>
#include "common/homography.h"

// threading stuff
#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <atomic>

// Disc Stuff
#include <dvd_DvisEst_apriltag.hpp>
#include <dvd_DvisEst_image_capture.hpp>
#include <dvd_DvisEst_image_processing.hpp>
#include <dvd_DvisEst_estimate.hpp>
#include <disc_layouts.hpp>

using namespace std;

// define camera calibration parameters
double Fx = 0;
double Fy = 0;
double Cx = 0;
double Cy = 0;

// should this match the meas queue count in the estimate meas queue? probably not
// in general, it seems like this should be << than the meas queue to account for
// the variability in apriltag detection speed
#define AT_THREAD_COUNT     (8)
#define AT_INT_THREAD_COUNT (4)
//std::vector<std::atomic<uint8_t>>  sv_meas_queue_status(MEAS_QUEUE_SIZE);
std::vector<std::thread> at_detection_thread (AT_THREAD_COUNT);

// Start apriltag thread (image_capture_t not yet populated)
// I don't think we need any return values from these right now
int at_detection_thread_run(uint8_t thread_id)
{
  //cerr << "Thread #" << (int)thread_id << " returned right away (for test!)!" << endl;
  //return 0;

  // Local def for image capture
  image_capture_t image_capture;
  // which measurement slot we are assigned
  uint8_t meas_slot_id;

  // Apriltag objects

/*  ///////////////////////////////////////////////////////////////
  // User-configurable parameters.

  // How many threads should be used?
  int nthreads;

  // detection of quads can be done on a lower-resolution image,
  // improving speed at a cost of pose accuracy and a slight
  // decrease in detection rate. Decoding the binary payload is
  // still done at full resolution. .
  float quad_decimate;

  // What Gaussian blur should be applied to the segmented image
  // (used for quad detection?)  Parameter is the standard deviation
  // in pixels.  Very noisy images benefit from non-zero values
  // (e.g. 0.8).
  float quad_sigma;

  // When non-zero, the edges of the each quad are adjusted to "snap
  // to" strong gradients nearby. This is useful when decimation is
  // employed, as it can increase the quality of the initial quad
  // estimate substantially. Generally recommended to be on (1).
  //
  // Very computationally inexpensive. Option is ignored if
  // quad_decimate = 1.
  int refine_edges;

  // How much sharpening should be done to decoded images? This
  // can help decode small tags but may or may not help in odd
  // lighting conditions or low light conditions.
  //
  // The default value is 0.25.
  double decode_sharpening;

  // When non-zero, write a variety of debugging images to the
  // current working directory at various stages through the
  // detection process. (Somewhat slow).
  int debug;*/

  apriltag_family_t *tf   = tag36h11_create();
  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family_bits(td, tf, 1);
  td->quad_decimate   = 0.0;
  td->quad_sigma      = 0.0;
  td->nthreads        = AT_INT_THREAD_COUNT;
  td->debug           = 0;
  td->refine_edges    = 0; // might want to disable this for 522fps
  td->decode_sharpening = 0.25;

  // Define housing for grayscale tag
  cv::Mat img_grey;

  while(!dvd_DvisEst_estimate_complete())
  {
    // Look for a new frame from the camera (or perhaps from loaded test images)
    const bool got_image = dvd_DvisEst_image_capture_get_next_image_capture(&image_capture);

    // did we get a frame? Neat, let's reserve a measurement queue slot until apriltag detection is complete
    if(got_image)
    {
      if(dvd_DvisEst_estimate_reserve_measurement_slot(image_capture.frame_id, &meas_slot_id))
      {
        // Cool, we have an image now, and a measurement slot reserved, let's perform the AprilTag detection!

        // First, undistort the image
        dvd_DvisEst_image_processing_undistort_image(&(image_capture.image_data));

        // convert to greyscale
        cvtColor(image_capture.image_data, img_grey, cv::COLOR_RGB2GRAY);
    
        //format opencv Mat into apriltag wrapper
        image_u8_t img_header = 
        {     
          .width  = img_grey.cols,
          .height = img_grey.rows,
          .stride = img_grey.cols,
          .buf    = img_grey.data
        };

        // detect those apriltags
        zarray_t *detections = apriltag_detector_detect(td, &img_header);

        const int detect_num = zarray_size(detections);

        // Using the apriltag measurement, generate a measurement update for our Kalman filter
        dvd_DvisEst_kf_meas_t kf_meas;
        memset(&kf_meas, 0, sizeof(dvd_DvisEst_kf_meas_t));
        kf_meas.timestamp_ns  = detect_num > 0 ? image_capture.timestamp_ns : 0;
        kf_meas.frame_id      = detect_num > 0 ? image_capture.frame_id : 0;

        // for now? let's just return an array of zeros, ha
        //cerr << "Delivered measurement!" << endl;
        dvd_DvisEst_estimate_fulfill_measurement_slot(meas_slot_id, &kf_meas);

        apriltag_detections_destroy(detections);
      }
      else
      {
        // did we not get a measurement slot reservation? weird. Maybe we haven't processed the old measurements yet
        cerr << "Frame ID " << image_capture.frame_id << " was dropped due to a lack of available measurement slots!! Do your estimate loop faster!" << endl;
      }
    }
  }

  // clean up apriltag objects
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
}

// Test a simple call to the apriltag lib
bool dvd_DvisEst_apriltag_test(void)
{
  apriltag_family_t *tf = tag36h11_create();
  tag36h11_destroy(tf);

  return true;
}

// Initialize apriltag thread pool (note, this should be called AFTER dvd_DvisEst_image_processing_init so the camera cal is loaded!)
void dvd_DvisEst_apriltag_init(void)
{
  cerr << "Call dvd_DvisEst_apriltag_init" << endl;

  // Get initial camera calibration parameters
  dvd_DvisEst_image_processing_get_camera_params(&Fx, &Fy, &Cx, &Cy);

  // initialize detection threads
  int i;
  for(i = 0; i < AT_THREAD_COUNT; i++)
  {
    at_detection_thread[i] = std::thread(at_detection_thread_run, i);
  }

  // join all threads right away
  for(i = 0; i < AT_THREAD_COUNT; i++)
  {
    //at_detection_thread[i].join();
  }
}
