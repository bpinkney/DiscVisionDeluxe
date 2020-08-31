#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(WIN64) || defined(_WIN64) || defined(__WIN64)) && !defined(IS_WINDOWS)
#define IS_WINDOWS

#include <windows.h>
// option to disable all warnings (does this work? NOPE)
#pragma warning(push, 0)

// fix for garbage MSVC c++17 support (c'mon guys, sweet christ)
#define _HAS_STD_BYTE 0
#endif

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <iostream>

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

// timer overloads for windows
#if defined(IS_WINDOWS)

static void usleep(__int64 usec) 
{ 
    HANDLE timer; 
    LARGE_INTEGER ft; 

    ft.QuadPart = -(10*usec); // Convert to 100 nanosecond interval, negative value indicates relative time

    timer = CreateWaitableTimer(NULL, TRUE, NULL); 
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0); 
    WaitForSingleObject(timer, INFINITE); 
    CloseHandle(timer); 
}
#endif

using namespace std;

// define camera calibration parameters
double Fx = 0;
double Fy = 0;
double Cx = 0;
double Cy = 0;

//std::vector<std::atomic<uint8_t>>  sv_meas_queue_status(MEAS_QUEUE_SIZE);
std::vector<std::thread>          at_detection_thread (AT_THREAD_COUNT);
std::vector<std::atomic<uint8_t>> at_detection_thread_mode (AT_THREAD_COUNT);

// store apriltag pixel centroid for consumption by auto gain/exposure
std::atomic<float> at_pixel_centroid{0};

std::mutex                  test_mutex;

// Get intensity centroid of pixels which make up apriltag, normalized to [0,1]
static void at_calculate_pixel_centroid(cv::Mat img_grey, cv::Point at_corner_points[1][4], uint64_t timestamp_ns, const double des_centroid)
{
  const cv::Point* ppt[1] = { at_corner_points[0] };
  int npt[] = { 4 };
  int lineType = 8;

  cv::Mat img_mask;
  img_grey.copyTo(img_mask);

  // change mask to black
  img_mask = cv::Scalar(0);

  // fill apriltag bounding box with white pixels
  fillPoly(img_mask,
           ppt,
           npt,
           1,
           cv::Scalar( 255, 255, 255 ),
           lineType);

  vector<cv::Point> indices;
  const int count = countNonZero(img_mask);
  if(count > 0)
  {
    findNonZero(img_mask, indices);
  }

  uint8_t histogram_256[256] = {0};
  cv::Scalar pixel_colour;
  for(int point = 0; point < count; point++)
  {
    // get original colour at point specified within mask
    pixel_colour = img_grey.at<uchar>(indices[point]);

    // build histogram
    int bin = (uint8_t)pixel_colour[0];
    histogram_256[bin]++;
  }

  // Generate centroid from histogram
  float centroid_num = 0;
  float centroid_den = 0;
  int i;
  for(i = 0; i <= 255; i++) 
  {
      centroid_num += (float)(i+1) * (float)histogram_256[i];
      centroid_den += (float)histogram_256[i];
  }
    
  // Calculate MSV and Normalize to [0-1]
  at_pixel_centroid = (centroid_num / std::max(centroid_den, (float)0.001)) / 255.0;

  // TODO: wrap this behind the debug flag
  static uint64_t last_show_time = 0;
  if(abs(at_pixel_centroid - des_centroid) < 0.1 && timestamp_ns - last_show_time > MS_TO_NS(1000))
  {
    last_show_time = timestamp_ns;
    // for debugging, show masked frame
    cv::Mat img_masked;
    img_grey.copyTo(img_masked, img_mask);
    ostringstream s;
    s << "Centroid = " << at_pixel_centroid;
    cv::putText(img_masked, //target image
              s.str(), //text
              cv::Point(10, img_masked.rows / 2), //top-left position
              cv::FONT_HERSHEY_DUPLEX,
              1.0,
              CV_RGB(255, 255, 255), //font color
              2);
    imshow("Exposure Gain Search", img_masked);
    cv::waitKey(10);

    /*cout << "[";
    for(i = 0; i <= 255; i++) 
    {
      cout << (int)histogram_256[i] << ", ";
    }
    cout << "]" << endl;*/
  }
  
}

float dvd_DvisEst_apriltag_get_at_pixel_centroid(void)
{
  return at_pixel_centroid;
}

// Start apriltag thread (image_capture_t not yet populated)
// I don't think we need any return values from these right now
int at_detection_thread_run(uint8_t thread_id, const bool convert_from_bayer, const bool calc_groundplane)
{
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
  td->refine_edges    = 1; // might want to disable this for 522fps
  td->decode_sharpening = 0.25;

  // Define housing for grayscale tag
  cv::Mat img_grey;
  cv::Mat img_grey_dist;

  // How many frames did we skip each sample to meet our real-time criteria?
  uint16_t skipped_frames = 0;

  uint64_t last_frame_detect_ns = 0;

  while(dvd_DvisEst_get_estimate_stage() < KF_EST_STAGE_PRIME)
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
      cerr << "Thread #" << (int)thread_id << " changed to MEAS mode!" << endl; 
    }

    //test_mutex.lock();
    // Look for a new frame from the camera (or perhaps from loaded test images)
    const bool got_image = dvd_DvisEst_image_capture_get_next_image_capture(&image_capture, &skipped_frames, &at_detection_thread_mode[thread_id], thread_id, calc_groundplane);

    // did we get a frame? Neat, let's reserve a measurement queue slot until apriltag detection is complete
    if(got_image)
    {
      // always process the frame when setting the ground plane
      // but don't bother reserving measurement queue slots
      bool process_frame = calc_groundplane;
      if(!calc_groundplane)
      {
        process_frame = dvd_DvisEst_estimate_reserve_measurement_slot(image_capture.frame_id, &meas_slot_id, skipped_frames);
      }

      if(process_frame)
      {
        // Cool, we have an image now, and a measurement slot reserved, let's perform the AprilTag detection!    

        // if we're outputting debug logging, save image to output dir
        // only bother if we're in MEAS thread mode
        string log_dir = dvd_DvisEst_estimate_get_log_dir();
        if(!log_dir.empty() && (at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_MEAS || calc_groundplane))
        {
          std::stringstream img_filename;
          img_filename << log_dir << "images/" << std::setfill('0') << std::setw(8) << std::to_string(image_capture.frame_id) << "_frame.jpg";
          // can't save Bayer directly, convert to standard RGB
          cv::Mat img_rgb;
          if(convert_from_bayer)
          {
            cvtColor(image_capture.image_data, img_rgb, cv::COLOR_BayerRG2RGB);
          }
          else
          {
            img_rgb = image_capture.image_data;
          }
          imwrite(img_filename.str(), img_rgb);
        }

        // if using spinnaker images, we are converting from bayer
        // Make sure to convert before undistorting!!
        if(convert_from_bayer)
        {
          cvtColor(image_capture.image_data, img_grey_dist, cv::COLOR_BayerRG2GRAY);
        }
        else
        {
          cvtColor(image_capture.image_data, img_grey_dist, cv::COLOR_RGB2GRAY);
        }

        // undistort the grey image
        dvd_DvisEst_image_processing_undistort_image(&img_grey_dist, &img_grey);

        // log undistorted image if desired
        //if(!log_dir.empty() && (at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_MEAS || calc_groundplane))
        //{
        //  std::stringstream img_filename;
        //  img_filename << log_dir << "images_undist/" << std::setfill('0') << std::setw(8) << std::to_string(image_capture.frame_id) << "_frame.jpg";
        //  imwrite(img_filename.str(), img_grey);
        //}

        //cv::Size image_size = image_capture.image_data.size();

        //cerr << "Image Size [" << image_size.width << ", " << image_size.height << "]" << endl;

        // convert to greyscale
        //format opencv Mat into apriltag wrapper
        #if !defined(IS_WINDOWS)
        image_u8_t img_header = 
        {     
          .width  = img_grey.cols,
          .height = img_grey.rows,
          .stride = img_grey.cols,
          .buf    = img_grey.data
        };
        #else
        image_u8_t img_header = *image_u8_create_stride(img_grey.cols, img_grey.rows, img_grey.cols);
        img_header.buf    = img_grey.data;
        #endif

        // detect those apriltags
        zarray_t *detections = apriltag_detector_detect(td, &img_header);

        const int detect_num = zarray_size(detections);

        /*if(detect_num > 0)
        {
          cerr << "*** GOT TAG ***" << endl;
        }*/

        if(detect_num > 0)
        {
          last_frame_detect_ns = image_capture.timestamp_ns;
        }

        // Check for scout mode, update apriltag detection timer, update thread mode if applicable.
        // For a scout mode thread, never report a measurement
        // Keep things in Scout mode while we set the groundplane, since we want to update it indefinitely while the 
        // exposure and gain are being set
        if(detect_num > 0 && at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_SCOUT && !calc_groundplane)
        {
          dvd_DvisEst_estimate_set_tags_detected(true, image_capture.frame_id);
        }

        if(detect_num > 0 && (at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_MEAS || calc_groundplane))
        {
          /*if(!log_dir.empty())
          {
            cerr << "Write frame AGAIN " << std::to_string(image_capture.frame_id) << endl;
            string img_filename = log_dir + "images/" + std::to_string(image_capture.frame_id) + "_frameAGAIN.jpg";
            imwrite(img_filename, image_capture.image_data);
          }*/

          // TODO: take first detection which falls within our lookup sets
          for (int tag = 0; tag < min(detect_num, 1); tag++) 
          {
            apriltag_detection_t *det;
            zarray_get(detections, tag, &det);

            // get apriltag ID
            const uint16_t apriltag_id = det->id;

            // Look up apriltag and disc parameters based on apriltag ID
            map<uint16_t, disc_layout_t>::const_iterator dl_lookup = disc_layout_by_id.find(apriltag_id);
            disc_layout_t dl = dl_lookup->second;

            DiscIndex disc_index = dl.disc_index;
            uint8_t player       = dl.player;
            uint16_t tag_size_mm = dl.tag_size_mm;

            // get homography transform from tag size
            const float homography_to_m  = (float)tag_size_mm / 2.0 * 0.001;

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

            /*cerr << "Got apriltag with ID " << (int)apriltag_id << ", Tag Size(mm): " << (int)tag_size_mm << ", T_CD(m) = [" << 
              T_CD(0, 0) << ", " << T_CD(1, 0) << ", " << T_CD(2, 0) << "]" << endl;*/

            if(calc_groundplane && (disc_index == GROUNDPLANE || disc_index == GROUNDPLANE_BIG))
            {
              // Update ground plane
              dvd_DvisEst_estimate_update_groundplane(R_CD, T_CD);

              // get pixel intensity centroid from detected apriltag [0,1]
              cv::Point at_corner_points[1][4];
              for (int c = 0; c < 4; c++)
              {
                at_corner_points[0][c] = cv::Point(det->p[c][0], det->p[c][1]);
              }

              // We can be a bit smarter about what the target centroid should be based on the number
              // of black and white pixels in each apriltag
              // ideally, these would be chosen to be balanced, but for now, just scratch in some quick math
              // also an issue if you print things on non-white cardstock like I did..... skip and fudge factors for this for now
              // GROUNDPLANE     -> tag 386 for 36h11 -> 18 white squares, 46 black squares, des_centroid_for_balance = 18/46 = 0.39
              // GROUNDPLANE_BIG -> tag 387 for 36h11 -> 15 white squares, 49 black squares, des_centroid_for_balance = 15/49 = 0.31
              const double des_centroid = (disc_index == GROUNDPLANE ? 0.39 : 0.31);

              // later, we could add this to the regular tag detection and continue to adjust the 
              // gain/exposure throughout throws. For now, just do it when the ground plane is set.
              at_calculate_pixel_centroid(img_grey, at_corner_points, image_capture.timestamp_ns, des_centroid);

              dvd_DvisEst_image_capture_calculate_exposure_gain(des_centroid, at_pixel_centroid, true);
            }
            else if(!calc_groundplane)
            {
              // calculate normal disc measurements from AprilTag measurements
              dvd_DvisEst_estimate_transform_measurement(R_CD, T_CD, &kf_meas);

              // fulfill measurement reservation
              dvd_DvisEst_estimate_fulfill_measurement_slot(meas_slot_id, &kf_meas);
            }
          }
        }
        else if(!calc_groundplane)
        {
          // No apriltag detections, cancel reservation
          dvd_DvisEst_estimate_cancel_measurement_slot(meas_slot_id, at_detection_thread_mode[thread_id] == AT_DETECTION_THREAD_MODE_MEAS, image_capture.frame_id);
        }
        else
        {
          // We're setting the ground plane, and the exposure starts from "very dark"
          // assume that we need to make the scene progressively lighter for now
          // target is 0.5, so 0.05 of centroid error
          // wait until we lose detections before trying to sweep
          if(image_capture.timestamp_ns - last_frame_detect_ns > MS_TO_NS(5000))
          {
            // sweep up with a fixed error
            dvd_DvisEst_image_capture_calculate_exposure_gain(0.5, 0.35, false);
            static uint64_t last_show_time = 0;

            if(image_capture.timestamp_ns - last_show_time > MS_TO_NS(1000))
            {
              imshow("Exposure Gain Search", img_grey);
              last_show_time = image_capture.timestamp_ns;
              cv::waitKey(10);
            }
          }          
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
      // check for test case (queue is empty due to loaded images)
      if(dvd_DvisEst_image_capture_thread_ready())
      {
        if(dvd_DvisEst_image_capture_get_image_capture_queue_size() < 2)
        {
          // end thread
          break;
        }
      }

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
void dvd_DvisEst_apriltag_init(const bool convert_from_bayer, const bool calc_groundplane)
{
  cerr << "Call dvd_DvisEst_apriltag_init" << endl;

  // Get initial camera calibration parameters
  dvd_DvisEst_image_processing_get_camera_params(&Fx, &Fy, &Cx, &Cy);

  cerr << "Camera Calibration Parameters: Fx = " << Fx << ", Fy = " << Fy << ", Cx = " << Cx << ", Cy = " << Cy << endl;

  // initialize detection threads
  int i;
  for(i = 0; i < AT_THREAD_COUNT; i++)
  {
    at_detection_thread[i] = std::thread(at_detection_thread_run, i, convert_from_bayer, calc_groundplane);
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

  cerr << "dvd_DvisEst_apriltag_end threads have finished joining!" << endl;
}
