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

#include <exception>
#include <typeinfo>
#include <stdexcept>

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

//std::vector<std::atomic<uint8_t>>  sv_meas_queue_status(MEAS_QUEUE_SIZE);
std::vector<std::thread>          at_detection_thread (AT_THREAD_COUNT);
std::vector<std::atomic<uint8_t>> at_detection_thread_mode (AT_THREAD_COUNT);

std::mutex                  test_mutex;

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
  // This slows down detection a bit
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
  td->decode_sharpening = 0.0;

  // Define housing for grayscale tag
  cv::Mat img_grey;

  // How many frames did we skip each sample to meet our real-time criteria?
  uint16_t skipped_frames = 0;

  while(!dvd_DvisEst_estimate_complete())
  {
    // Check for thread mode update
    if(!dvd_DvisEst_estimate_get_tags_detected())
    {
      if(at_detection_thread_mode[thread_id] != AT_DETECTION_THREAD_MODE_SCOUT)
      {
        at_detection_thread_mode[thread_id] = AT_DETECTION_THREAD_MODE_SCOUT;
        //cout << "Thread #" << (int)thread_id << " changed back to SCOUT mode!" << endl;
      }
    }
    else if(at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_SCOUT)
    {
      // start processing concurrent frames
      at_detection_thread_mode[thread_id] = AT_DETECTION_THREAD_MODE_MEAS;
      cout << "Thread #" << (int)thread_id << " changed to MEAS mode!" << endl; 
    }

    //test_mutex.lock();
    // Look for a new frame from the camera (or perhaps from loaded test images)
    const bool got_image = dvd_DvisEst_image_capture_get_next_image_capture(&image_capture, &skipped_frames, at_detection_thread_mode[thread_id]);

    // did we get a frame? Neat, let's reserve a measurement queue slot until apriltag detection is complete
    if(got_image)
    {
      if(dvd_DvisEst_estimate_reserve_measurement_slot(image_capture.frame_id, &meas_slot_id, skipped_frames))
      {
        //test_mutex.unlock();
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

        // Check for scout mode, update apriltag detection timer, update thread mode if applicable.
        // For a scout mode thread, never report a measurement
        if(detect_num > 0 && at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_SCOUT)
        {
          dvd_DvisEst_estimate_set_tags_detected(true);
        }

        if(detect_num > 0 && at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_MEAS)
        {
          // TODO: take first detection which falls within our lookup sets
          for (int tag = 0; tag < min(detect_num, 1); tag++) 
          {
            apriltag_detection_t *det;
            zarray_get(detections, tag, &det);

            // get apriltag ID
            const uint16_t apriltag_id = det->id;

            //cerr << "Got apriltag with ID " << (int)apriltag_id << endl;

            // Look up apriltag and disc parameters based on apriltag ID
            map<uint16_t, disc_layout_t>::const_iterator dl_lookup = disc_layout_by_id.find(apriltag_id);
            disc_layout_t dl = dl_lookup->second;

            DiscIndex disc_index = dl.disc_index;
            uint8_t player       = dl.player;
            uint16_t tag_size_mm = dl.tag_size_mm;

            // get homography transform from tag size
            const float homography_to_m  = tag_size_mm / 2.0 * 0.001;

            // Using the apriltag measurement, generate a measurement update for our Kalman filter
            dvd_DvisEst_kf_meas_t kf_meas;
            memset(&kf_meas, 0, sizeof(dvd_DvisEst_kf_meas_t));

            // update metadata
            kf_meas.timestamp_ns  = image_capture.timestamp_ns;
            kf_meas.frame_id      = image_capture.frame_id;
            kf_meas.disc_index    = disc_index;
            kf_meas.player        = player;

            // determine actual pose (rotation and translation) from Homography matrix and camera intrinsics
            // This seems to get the depth correct, but not the translations in X and Y
            // might need better projection from the sensor through to the camera frame?
            // For homography_to_pose, you have to pass in a negative fx parameter.
            // This is again due to the OpenCV convention of having z negative.
            matd_t * T = homography_to_pose(det->H, -Fx, Fy, Cx, Cy);

            cv::Matx33d R_CD
            ( 
              MATD_EL(T, 0, 0), MATD_EL(T, 0, 1), MATD_EL(T, 0, 2),
              MATD_EL(T, 1, 0), MATD_EL(T, 1, 1), MATD_EL(T, 1, 2),
              MATD_EL(T, 2, 0), MATD_EL(T, 2, 1), MATD_EL(T, 2, 2)
            );

            cv::Matx31d T_CD
            ( 
              MATD_EL(T, 0, 3)*homography_to_m, 
              MATD_EL(T, 1, 3)*homography_to_m, 
              MATD_EL(T, 2, 3)*homography_to_m
            );          

            // calculate disc measurements from AprilTag measurements
            dvd_DvisEst_estimate_transform_measurement(R_CD, T_CD, &kf_meas);

            // fulfill measurement reservation
            dvd_DvisEst_estimate_fulfill_measurement_slot(meas_slot_id, &kf_meas);
          }
        }
        else
        {
          // No apriltag detections, cancel reservation
          dvd_DvisEst_estimate_cancel_measurement_slot(meas_slot_id, at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_MEAS);
        }

        apriltag_detections_destroy(detections);
      }
      else
      {
        // did we not get a measurement slot reservation? weird. Maybe we haven't processed the old measurements yet
        //test_mutex.unlock();
        cerr << "Frame ID " << image_capture.frame_id << " was dropped due to a lack of available measurement slots!! Do your estimate loop faster!" << endl;
      }
    }
    else
    {
      // sleep for a bit so we don't busy poll
      //test_mutex.unlock();
      usleep(1000);
    }

  }

  // clean up apriltag objects
  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);

  cerr << "AprilTag Thread#" << (int)thread_id << " completed." << endl;
  return 0;
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

void dvd_DvisEst_apriltag_end(void)
{
  cerr << "Call dvd_DvisEst_apriltag_end" << endl;

  // join all apriltag threads
  int i;
  for(i = 0; i < AT_THREAD_COUNT; i++)
  {
    at_detection_thread[i].join();
  }
}
