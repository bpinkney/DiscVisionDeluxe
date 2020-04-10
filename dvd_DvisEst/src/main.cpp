#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32)) && !defined(SPINNAKER_ALLOWED)
// not available in mingw64 for windows! (sad)
// I'm starting to think spinnaker and apriltag are never meant to
// be together on windows...
#else
#define SPINNAKER_ALLOWED
#endif

#include <string>
#include <iostream>
#include <unistd.h>
#include <iomanip>

#include <atomic>

// OpenCV stuff
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// disc includes
#include <dvd_DvisEst_image_capture.hpp>
#include <dvd_DvisEst_image_processing.hpp>
#include <dvd_DvisEst_apriltag.hpp>
#include <dvd_DvisEst_estimate.hpp>

using namespace std;

int main(int argc, char** argv )
{
  // Note: values which are not proceeded by a flag can be defined by @arg
  const cv::String keys =
    "{help h usage ? |           | Print this help message   }"
    "{helloworld hw  |false      | Call basic test functions to ensure libraries are loading correctly}"
    "{fromimgsdir fi |           | If this is populated, don't use the camera stream.\n"
                  "                Instead, load images from the directory specified after this flag.\n"
                  "                Currently only this test mode is supported on Windows due to Spinnaker mingw64 support (or lack thereof) }"
    "{dt             |0.00191570881226054| Nominal value of dt (in seconds) between frames. Default 1/522\n"
                  "                (only used when fromimgsdir is populated and dt is unknown)}"
    "{dispimgs di    |false      | Show resulting images at ~x5 slo-mo"
                  "                (Can be used for image files or real-time, but will severely slow down processing)}"
    "{saveimgs si    |false      | Save images to file}"
    "{camcal cc      |../bin/camera_cal800.yaml| Camera calibration yaml file, generated by camera_calibration_from_jpgs in the dvd_DvisEst sandbox}"
    "{groundplane gp |../bin/sample_ground_plane.yaml| Ground plane rotation and pre-rot translation.\n"
                  "                Defines transformation between camera and ground plane frames wrt camera frame.}"
    "{setgroundplane sgp |false| Place sample AprilTag on the ground, and determine resulting ground plane}"
    "{kflog log      |true       | Generate log files for KF meas and states}"
    "{matlab ml      |false      | Run some matlab plots using the generated kflog}"
    "{dfisx rdf      |false      | Run dfisx and the matlab renderer for it, nice!}"
    ;

  cv::CommandLineParser parser(argc, argv, keys);
  if (parser.has("help"))
  {
    parser.printMessage();
    return 0;
  }

  // grab commandline args
  const bool        helloworld  = parser.get<bool>("helloworld");
  const cv::String  imgdir_src  = parser.get<cv::String>("fromimgsdir");
  const bool        camera_src  = imgdir_src.empty();
  const double      dt_src      = parser.get<double>("dt");
  const bool        imgdisp     = parser.get<bool>("dispimgs");
  const cv::String  camera_cal  = parser.get<cv::String>("camcal");
  const cv::String  gnd_plane   = parser.get<cv::String>("groundplane");
  const bool      set_gnd_plane = parser.get<bool>("setgroundplane");
  const bool        kflog       = parser.get<bool>("kflog");
  const bool        matlab      = parser.get<bool>("matlab");
  const bool        dfisx       = parser.get<bool>("dfisx");

  // check for basic function test call
  if(helloworld)
  {
    cerr << endl << "AprilTag function test?  " << dvd_DvisEst_apriltag_test() << "/1" << endl;
    cerr << endl << "OpenCV function test?    " << dvd_DvisEst_image_processing_test() << "/1" << endl;
    #if defined(SPINNAKER_ALLOWED)
    cerr << endl << "Spinnaker function test? " << dvd_DvisEst_image_capture_test() << "/1" << endl;
    #else
    cerr << endl << "Spinnaker function test? " << dvd_DvisEst_image_capture_test() << "/0!" << endl;
    #endif
    cerr << endl;
    cerr << "dvd_DvisEst helloworld test complete.";
    cerr << endl;
    return 0;
  }

  // This would be very similar to a standard throw, except that we may enforce a lower variance threshold
  // (or perhaps a low magnitude velocity state!)
  // We could also consider changing the filter covariance, and even the veloicty states, with the assumption that this is indeed stationary.
  if(set_gnd_plane)
  {
    cerr << "Sorry! Setting the ground plane isn't implemented yet!" << endl;
  }

  // Init undistort and image processing
  // Right now, we're just using a fixed scaling coefficient based on the level
  // of detail lost during the undistort (roughly ~x2 the pixels)
  // This could decrease for a less distorted lens
  const double resize_factor = (1040.0/720.0);
  if(!dvd_DvisEst_image_processing_init(camera_cal, resize_factor))
  {
    cerr << "Could not load camera calibration!" << endl;
    return 1;
  }

  // check for test images directory
  if(!camera_src && imgdisp)
  {
    cerr << "Test images directory <" << imgdir_src << ">" << endl;

    // explicitly queue up images from imgdir_src into the queue as if
    // they had been captured from the camera.
    // In this case, we'll avoid launching the thread and just fill the queue
    // with all available images (careful for your RAM here!)
    const bool imgs_queued = dvd_DvisEst_image_capture_load_test_queue(imgdir_src, dt_src);
    if(!imgs_queued)
    {
      cerr << "No test images found!" << endl;
      return 1;
    }

    // try to read some images for fun
    if(imgdisp)
    {
      double   slomofactor = 5.0;
      uint32_t wait_ms = (uint32_t)(dt_src * 1000.0 * slomofactor);

      cerr << "Got images, now show 'em" << endl;
      image_capture_t image_capture;
      bool got_one;
      uint16_t skipped_frames = 0; 
      while(1)
      {
        skipped_frames = 888;// quick flag to avoid frame skips
        std::atomic<uint8_t> thread_mode (AT_DETECTION_THREAD_MODE_MEAS);
        got_one = dvd_DvisEst_image_capture_get_next_image_capture(&image_capture, &skipped_frames, &thread_mode, 0);

        if(got_one)
        {
          // undistort image first
          dvd_DvisEst_image_processing_undistort_image(&image_capture.image_data);

          //cv::Size img_size = image_capture.image_data.size();
          //cerr << "Image Width: " << img_size.width << ", Height: " << img_size.height << endl;

          imshow("Image View", image_capture.image_data);
          if(0)
          {
            // wait for keypress to proceed through frames
            char c = (char)cv::waitKey();
            if( c == 27 || c == 'q' || c == 'Q' )
                break;
          }
          else
          {
            cv::waitKey(wait_ms);
          }

        }
        else
        {
          cerr << "Done imreading queue!" << endl;
          return 0;
        }
      }
    }
  }

  // Init remaining calls
  // allocate measurement slots and prep Kalman Filter initial states
  if(!dvd_DvisEst_estimate_init(gnd_plane, kflog))
  {
    cerr << "Can't read ground plane, exiting...!" << endl;
    return 1;
  }

  if(!camera_src)
  {
    const bool imgs_queued = dvd_DvisEst_image_capture_load_test_queue(imgdir_src, dt_src);
  }

  // Now let's read the image frames out of our queue, undistort them, and run them through our apriltag detector
  // start estimation thread (Kalman filter)
  // wait for estimation thread to return with a low-variance state
  // start this before your detection thread so we're ready to consume!
  dvd_DvisEst_estimate_process_filter();

  // start apriltag detection thread pool
  dvd_DvisEst_apriltag_init();

  // to test the apriltag and estimate code correctly, we need to load
  // the test images in in real-time (as if they were being supplied by the camera)
  // this is obviously slower (maybe make a couple queues later and we can move things over at the FPS of the camera)
  if(!camera_src)
  {
    // run as thread instead to test 'real-time-style' processing
    //dvd_DvisEst_image_capture_load_test_queue_threaded(imgdir_src, dt_src);
  }

  // Finally, start the capture thread now that everything is ready
  if(camera_src)
  {
    // Not running a test from image files?
    // Then we are actively capturing from our capture thread
    // and queueing frames when available
    // Note: This thread only joins when our KF thread indicates that
    // the initial disc states have stabilized

    // Init camera interface
    dvd_DvisEst_image_capture_init();

    // start image capture thread
  }
  
  // We should be calling reverse-order thread join/destroy functions here
  // Once the filter thread returns, we know we can wrap everything up
  cerr << "Waiting Until threads join..." << endl;
  dvd_DvisEst_estimate_end_filter();
  dvd_DvisEst_apriltag_end();
  dvd_DvisEst_image_capture_stop(camera_src);

  // get final output state
  dvd_DvisEst_kf_state_t kf_state;
  const bool got_output = dvd_DvisEst_estimate_get_ideal_output_state(&kf_state);

  if(!got_output)
  {
    cerr << ("Not ouput state! Sorry!\n") << endl;
    return 0;
  }

  // plotting with matlab!
  if(matlab && kflog && !dfisx)
  {
    cerr << ("Executing Matlab plot...\n") << endl;
    system("cd ~/disc_vision_deluxe/DiscVisionDeluxe/matlab/visualizers/; matlab -nosplash -nodesktop -r \"plot_test_log_kfstate\" &");
  }

  // run Skinner's stuff
  if(dfisx)
  {
    char output_cmd[512] = {0};
    sprintf(output_cmd, "cd ~/disc_vision_deluxe/DiscVisionDeluxe/dvd_DfisX/; ./dfisx hyzer %0.5f pitch %0.5f discmold 0 posx %0.3f posy %0.3f posz %0.3f velx %0.3f vely %0.3f velz %0.3f spinrate %0.5f wobble 0",
      kf_state.ang_hps[0].pos,
      kf_state.ang_hps[1].pos,
      kf_state.lin_xyz[0].pos,
      kf_state.lin_xyz[1].pos,
      kf_state.lin_xyz[2].pos,
      kf_state.lin_xyz[0].vel,
      kf_state.lin_xyz[1].vel,
      max((double)kf_state.lin_xyz[2].vel, 0.0),
      kf_state.ang_hps[2].vel
      );

    cerr << "Output String: " << output_cmd << endl;

    system(output_cmd);

    cerr << "Output String: " << output_cmd << endl;

    if(matlab)
    {
      //drive10, 13, 16, 17
      //angle 1, 2
      //putt 2

      // Let's plot it up boys!
      system("cd ~/disc_vision_deluxe/DiscVisionDeluxe/matlab/visualizers/; matlab -nosplash -nodesktop -r \"dvd_DfisX_plot_disc_trajectory\" &");
      //cerr << "Output String: " << output_cmd << endl;
    }
  }

  return 0;
}