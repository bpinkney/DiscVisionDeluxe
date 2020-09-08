#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(WIN64) || defined(_WIN64) || defined(__WIN64)) && !defined(IS_WINDOWS)
#define IS_WINDOWS

#include <windows.h>
// option to disable all warnings (does this work? NOPE)
#pragma warning(push, 0)

// fix for garbage MSVC c++17 support (c'mon guys, sweet christ)
#define _HAS_STD_BYTE 0

#endif

//#if (defined(IS_WINDOWS) && !defined(SPINNAKER_ALLOWED))
// not available in mingw64 for windows! (sad)
// I'm starting to think spinnaker and apriltag are never meant to
// be together on windows...
//#elsed
#define SPINNAKER_ALLOWED
//#endif

#include <string>
#include <iostream>
#if !defined(IS_WINDOWS)
  #include <unistd.h>
#else
  #include "wtypes.h"
#endif
#include <iomanip>
#include <chrono>
#include <ctime>
#include <atomic>
#include <cstdlib>
#include <fstream>

// OpenCV stuff
#include "opencv2/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/core/utils/filesystem.hpp"
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

// get path of current executable in linux and windows for logging purposes
static cv::String get_executable_path(void)
{
  #if defined(IS_WINDOWS)
  cv::String path = "";
  char ownPth[1024];
  // When NULL is passed to GetModuleHandle, the handle of the exe itself is returned
  HMODULE hModule = GetModuleHandle(NULL);
  if (hModule != NULL)
  {
    // Use GetModuleFileName() with module handle to get the path
    GetModuleFileName(hModule, ownPth, (sizeof(ownPth))); 
    path = cv::String(ownPth);
    // make sure it exists since windows crashes the program otherwise.....
    auto pos = path.rfind("\\");
    if (pos != std::string::npos)
    {
      path.erase(pos);
    }
  }
  else
  {
    cerr << "Executable path handle is NULL" << endl ;
  }
  cerr << "Executable path: " << path.c_str() << endl;
  return path;
  #else
  cv::String path = "";
  char buff[PATH_MAX];
  ssize_t len = ::readlink("/proc/self/exe", buff, sizeof(buff)-1);
  if (len != -1) 
  {
    buff[len] = '\0';
    path = cv::String(buff);
    auto pos = path.rfind('/');
    if (pos != std::string::npos)
    {
      path.erase(pos);
    }
  }  
  cerr << "Executable path: " << path.c_str() << endl;
  return path;
  #endif
}

/*static void replace_slashes_linux_to_windows(std::string * linux_path)
{
  #if defined(IS_WINDOWS)
  //std::replace(*linux_path.begin(), *linux_path.end(), '/', '\\');
  #endif
}*/

// Get the horizontal and vertical screen sizes in pixel
static void dvd_DvisEst_get_desktop_resolution(int * horizontal, int * vertical)
{
  #if defined(IS_WINDOWS)
  RECT desktop;
  // Get a handle to the desktop window
  const HWND hDesktop = GetDesktopWindow();
  // Get the size of screen to the variable desktop
  GetWindowRect(hDesktop, &desktop);
  // The top left corner will have coordinates (0,0)
  // and the bottom right corner will have coordinates
  // (horizontal, vertical)
  *horizontal = desktop.right;
  *vertical = desktop.bottom;
  #else
  std::system("xdpyinfo | grep dimensions | sed -r 's/^[^0-9]*([0-9]+x[0-9]+).*$/\\1/' > /tmp/getres");
  std::ostringstream ss;
  ss << std::ifstream("/tmp/getres").rdbuf();
  std::string res_s = ss.str(); 
  //cerr << ss.str() << endl;
  std::string delimiter = "x";
  size_t dpos = res_s.find(delimiter);
  *horizontal = stoi(res_s.substr(0, dpos));
  res_s.erase(0, dpos + delimiter.length());
  *vertical   = stoi(res_s);
  #endif
}

static void dvd_DvisEst_display_text(const std::string * text_to_show, const int num_strings, const int time_s)
{
  int res_x;
  int res_y;
  dvd_DvisEst_get_desktop_resolution(&res_x, &res_y);
  cerr << "Res: " << res_x << ", " << res_y << endl;

  // build stats display matrix
  // set background to light grey
  cv::Mat throw_stats(cv::Size(res_x, res_y), CV_8UC3, cv::Scalar(240,240,240));

  // for idx,lbl in enumerate(lbls):
  //  cv2.putText(frame, str(lbl), (x,y+offset*idx), cv2.FONT_HERSHEY_SIMPLEX, 1, (0,0,0), 2)

  // assuming this scales linearly, we can use it to determine the max font size for each line
  const float text_size = 1.0;
  const float text_thickness = text_size * 2.0/3.0;

  int text_baseline = 0;
  const cv::Size text_pixels_10 = cv::getTextSize("TENLETTERS", cv::FONT_HERSHEY_PLAIN, text_size, text_thickness, &text_baseline);
  const float base_letter_width  = ((float)text_pixels_10.width) * 0.1;
  const float base_letter_height = ((float)text_pixels_10.height);

  int baseline_sum = 0;
  for(int idx = 0; idx < num_strings; idx++)
  {
    // get length of string
    const std::string next_line = text_to_show[idx];
    const float base_line_length = base_letter_width * next_line.length();
    const float width_mult = (float)res_x / base_line_length;

    const int baseline = (int)((text_baseline + text_thickness) * width_mult * 3.0);
    baseline_sum += baseline;

    // tack on some extra at the end here for the height scale
    if(idx == num_strings-1)
    {
      baseline_sum += baseline * 0.5;
    }
  }

  // now figure out what our 'height scale' needs to be
  // this is wasteful, but meh, I don't want dynamically sized arrays

  float height_scale = (float)res_y / (float)baseline_sum;
  // don't bother blowing up the letters if there aren't enough lines
  height_scale = height_scale > 1 ? 1.0 : height_scale;

  baseline_sum = 0;
  cv::Scalar rgb;
  for(int idx = 0; idx < num_strings; idx++)
  {
    // get length of string
    const std::string next_line = text_to_show[idx];
    const float base_line_length = base_letter_width * next_line.length();
    const float width_mult = (float)res_x / base_line_length * height_scale;

    const int baseline = (int)((text_baseline + text_thickness) * width_mult * 3.0);
    baseline_sum += baseline;

    switch(idx)
    {
      case 0:
      default:
        rgb = CV_RGB(0, 0, 0);
      break;
      case 1:
        rgb = CV_RGB(230, 30, 30);
      break;
      case 2:
        rgb = CV_RGB(30, 200, 30);
      break;
      case 3:
        rgb = CV_RGB(30, 30, 200);
      break;
      case 4:
        rgb = CV_RGB(200, 150, 0);
      break;
    }

    cv::putText(throw_stats, //target image
      next_line, //text
      cv::Point(0, baseline_sum), //top-left position //throw_stats.rows / 2
      cv::FONT_HERSHEY_PLAIN, 
      text_size * width_mult,
      rgb, //font color
      text_thickness * width_mult);
  }  

  cv::imshow("Throw Stats", throw_stats);
  cv::moveWindow("Throw Stats",0,0);
  cv::waitKey(time_s*1000);
}


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
    "{camcal cc      || Camera calibration yaml file, generated by camera_calibration_from_jpgs in the dvd_DvisEst sandbox}"
    "{groundplane gp |           | Ground plane rotation and pre-rot translation.\n"
                  "                Defines transformation between camera and ground plane frames wrt camera frame.}"
    "{setgroundplane sgp |false| Place sample AprilTag on the ground, and determine resulting ground plane}"
    "{d debug log    |false      | Enable kflog and capture images from apriltag detections for each throw}"
    "{ch chime       |false      | Play a chime when primed for next throw}"
    "{matlab ml      |false      | Run some matlab plots using the generated kflog}"
    "{dfisx rdf      |false      | Run dfisx and the matlab renderer for it, nice!}"
    "{gianttext gt   |false      | Show giant text with basic dvd_DvisEst throw info}"
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
        cv::String  camera_cal  = parser.get<cv::String>("camcal");
        cv::String  gnd_plane   = parser.get<cv::String>("groundplane");
  const bool      set_gnd_plane = parser.get<bool>("setgroundplane");
  const bool        debug       = parser.get<bool>("debug");
  const bool        chime       = parser.get<bool>("chime");
  const bool        matlab      = parser.get<bool>("matlab");
  const bool        dfisx       = parser.get<bool>("dfisx");
  const bool        gianttext   = parser.get<bool>("gianttext");

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

  // Set up directories
  // get date string for logging
  std::time_t now = std::time(NULL);
  std::tm * ptm = std::localtime(&now);
  string datestring;// = std::ctime(&end_time);
  char buffer[32];
  std::strftime(buffer, 32, "%Y-%m-%d_%H-%M-%S", ptm);
  datestring = buffer;
  cerr << "Current DateTime: " << datestring << endl;

  vector<string> fn;

  cv::String executable_path = get_executable_path();

  // set groundplane path
  #if defined(IS_WINDOWS)
  std::string groundplane_path = executable_path + "\\ground_planes\\";
  #else
  std::string groundplane_path = executable_path + "/ground_planes/";
  #endif
  //replace_slashes_linux_to_windows(&groundplane_path);
  cv::utils::fs::createDirectory(groundplane_path);
  cv::glob(groundplane_path,fn,false);
  if(!set_gnd_plane && gnd_plane.empty())
  {
    // use latest ground plane in dir
    cerr << "Using Latest collected groundplane automatically!" << endl;
    gnd_plane = fn[fn.size()-1];
  }
  else if(set_gnd_plane)
  {
    // generate new ground plane filename using number of files in dir +1
    const uint32_t ground_plane_fidx = fn.size();
    std::stringstream gp_filename;
    gp_filename << groundplane_path << std::setfill('0') << std::setw(8) << std::to_string(ground_plane_fidx) << "_" << datestring << "_ground_plane.yaml";
    gnd_plane = gp_filename.str();
  }
  cerr << "Ground Plane Path: "  << gnd_plane << endl;
  dvd_DvisEst_estimate_set_ground_plane_file(gnd_plane);

  std::string log_debug_path;
  if(debug)
  {
    #if defined(IS_WINDOWS)
    std::string log_path = executable_path + "\\logs\\";
    #else
    std::string log_path = executable_path + "/logs/";
    #endif
    //replace_slashes_linux_to_windows(&log_path);
    cv::utils::fs::createDirectory(log_path);
    #if defined(IS_WINDOWS)
    log_debug_path = log_path + datestring + "_log_data\\";
    #else
    log_debug_path = log_path + datestring + "_log_data/";
    #endif
    //replace_slashes_linux_to_windows(&log_debug_path);
    cv::utils::fs::createDirectory(log_debug_path);
    #if defined(IS_WINDOWS)
    std::string log_path_images = log_debug_path + "images\\";
    #else
    std::string log_path_images = log_debug_path + "images/";
    #endif
    //replace_slashes_linux_to_windows(&log_path_images);
    cv::utils::fs::createDirectory(log_path_images);
    dvd_DvisEst_estimate_set_log_dir(log_debug_path);
    cerr << "Logging Path: "  << log_debug_path << endl;
  }


  if(camera_src)
  {
    // Init camera interface
    dvd_DvisEst_image_capture_init(chime);
    // use the camera cal defined by the serial number if available (and no other camera cal provided)
    const uint32_t camera_serial_num = dvd_DvisEst_image_capture_get_camera_serial_number();
    if(camera_serial_num > 0 && camera_cal.empty())
    {
      #if defined(IS_WINDOWS)
      camera_cal = executable_path + "\\camera_calibrations\\" + std::to_string(camera_serial_num) + ".yaml";
      #else
      camera_cal = executable_path + "/camera_calibrations/" + std::to_string(camera_serial_num) + ".yaml";
      #endif
      //replace_slashes_linux_to_windows(&camera_cal);
      cerr << "Automatically retrieved camera calibration using camera serial number: " << endl << camera_cal <<  endl;
    }
  }

  if(camera_cal.empty())
  {
    // Use a default cal if none is provided (bad practice!)
    cerr << "USING DEFAULT CAMERA CAL, THIS IS DISCOURAGED!" << endl;
    #if defined(IS_WINDOWS)
    camera_cal = executable_path + "\\camera_calibrations/19508898_camera_cal4001.yaml";
    #else
    camera_cal = executable_path + "/camera_calibrations/19508898_camera_cal4001.yaml";
    #endif
    //replace_slashes_linux_to_windows(&camera_cal);
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
        got_one = dvd_DvisEst_image_capture_get_next_image_capture(&image_capture, &skipped_frames, &thread_mode, 0, false);

        if(got_one)
        {
          // undistort image first
          cv::Mat ud_image_data;
          dvd_DvisEst_image_processing_undistort_image(&image_capture.image_data, &ud_image_data);

          //cv::Size img_size = image_capture.image_data.size();
          //cerr << "Image Width: " << img_size.width << ", Height: " << img_size.height << endl;

          imshow("Image View", ud_image_data);
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

  if(!set_gnd_plane)
  {  
    // allocate measurement slots and prep Kalman Filter initial states
    if(!dvd_DvisEst_estimate_init(debug))
    {
      cerr << "Can't read ground plane, exiting...!" << endl;
      return 1;
    }
  }

  if(!camera_src)
  {
    const bool imgs_queued = dvd_DvisEst_image_capture_load_test_queue(imgdir_src, dt_src);
  }

  // Now let's read the image frames out of our queue, undistort them, and run them through our apriltag detector
  // start estimation thread (Kalman filter)
  // wait for estimation thread to return with a low-variance state
  // start this before your detection thread so we're ready to consume!
  if(!set_gnd_plane)
  { 
    dvd_DvisEst_estimate_process_filter();
  }

  // start apriltag detection thread pool
  dvd_DvisEst_apriltag_init(camera_src, set_gnd_plane);

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

    // start image capture thread
    dvd_DvisEst_image_capture_start();
  }
  
  // We should be calling reverse-order thread join/destroy functions here
  // Once the filter thread returns, we know we can wrap everything up
  cerr << "Waiting Until threads join..." << endl;
  bool got_output = false;
  char output_cmd[512] = {0};
  dvd_DvisEst_kf_state_t kf_state;

  if(!set_gnd_plane)
  {
    dvd_DvisEst_estimate_end_filter();

    // get final output state
    got_output = dvd_DvisEst_estimate_get_ideal_output_state(&kf_state);
    // we should print this state ASAP for consumption by Mike's dvd_DfisX
    sprintf(output_cmd, "hyzer %0.5f pitch %0.5f posx %0.3f posy %0.3f posz %0.3f velx %0.3f vely %0.3f velz %0.3f spinrate %0.5f wobble %0.3f discmold %d",
      kf_state.ang_hps[0].pos,
      kf_state.ang_hps[1].pos,
      kf_state.lin_xyz[0].pos,
      kf_state.lin_xyz[1].pos,
      kf_state.lin_xyz[2].pos,
      kf_state.lin_xyz[0].vel,
      kf_state.lin_xyz[1].vel,
      kf_state.lin_xyz[2].vel,
      kf_state.ang_hps[2].vel,
      kf_state.wobble_mag,
      kf_state.disc_index
      );
    cerr << endl << endl;
    // this is the only line is this program which should be printed to stdout
    cout << output_cmd << endl;

    // also pipe this into the metadata file
    if(debug)
    {
      std::stringstream ss_metadata;
      ss_metadata << "FILEPATH=$(ls " << log_debug_path << "metadata*); echo \"" << output_cmd << "\" >> " << "$FILEPATH";
      std::system(ss_metadata.str().c_str());
    }

    cerr << endl << endl;
  }

  dvd_DvisEst_apriltag_end();

  if(!set_gnd_plane)
  {
    if(!got_output)
    {
      cerr << ("Not ouput state! Sorry!\n") << endl;
      return 0;
    }

    // plotting with matlab!
    if(matlab && debug && !dfisx)
    {
      #if defined(IS_WINDOWS)
      cerr << ("Matlab plots not supported on Windows just yet...\n") << endl;
      #else
      cerr << ("Executing Matlab plot...\n") << endl;

      // get abs path
      char abs_path[512];
      realpath(log_debug_path.c_str(), abs_path);

      sprintf(output_cmd, "cd ../matlab/visualizers/; matlab -nosplash -nodesktop -r \"plot_test_log_kfstate('%s')\" &",
        abs_path);
      system(output_cmd);
      #endif
    }

    sprintf(output_cmd, "cd ../dvd_DfisX/; ./dfisx hyzer %0.5f pitch %0.5f posx %0.3f posy %0.3f posz %0.3f velx %0.3f vely %0.3f velz %0.3f spinrate %0.5f wobble %0.3f discmold %d",
      kf_state.ang_hps[0].pos,
      kf_state.ang_hps[1].pos,
      kf_state.lin_xyz[0].pos,
      kf_state.lin_xyz[1].pos,
      kf_state.lin_xyz[2].pos,
      kf_state.lin_xyz[0].vel,
      kf_state.lin_xyz[1].vel,
      kf_state.lin_xyz[2].vel,
      kf_state.ang_hps[2].vel,
      kf_state.wobble_mag*0,
      kf_state.disc_index*0 + 1
      );

    cerr << "Output String: " << output_cmd << endl;

    // run Skinner's stuff
    if(dfisx)
    {
      system(output_cmd);

      //cerr << "Output String: " << output_cmd << endl;

      if(matlab)
      {
        //drive10, 13, 16, 17
        //angle 1, 2
        //putt 2

        // Let's plot it up boys!
        system("cd ../matlab/visualizers/; matlab -nosplash -nodesktop -r \"dvd_DfisX_plot_disc_trajectory\" &");
        
      }

      cerr << "Output String: " << output_cmd << endl;
    }

    if(gianttext)
    {
      // show info for 10s
      const float vel_mag_kph = sqrt(kf_state.lin_xyz[0].vel*kf_state.lin_xyz[0].vel + kf_state.lin_xyz[1].vel*kf_state.lin_xyz[1].vel + kf_state.lin_xyz[2].vel*kf_state.lin_xyz[2].vel) * 3.6;
      const float throw_deg_up = RAD_TO_DEG(atan2(kf_state.lin_xyz[2].vel, kf_state.lin_xyz[0].vel));
      std::string throw_ud = "";
      if(throw_deg_up > 0)
      {
        throw_ud = "UP";
      }
      else
      {
        throw_ud = "DOWN";
      }
      const float throw_deg_right = RAD_TO_DEG(atan2(kf_state.lin_xyz[1].vel, kf_state.lin_xyz[0].vel));
      std::string throw_left_right = "";
      if(throw_deg_right < 0)
      {
        throw_left_right = "LEFT";
      }
      else
      {
        throw_left_right = "RIGHT";
      }
      const float spin_rad_d = (kf_state.ang_hps[2].vel);
      const float hyzer_rad  = RAD_TO_DEG(kf_state.ang_hps[0].pos);
      std::string throw_spin_dir = "";
      std::string hyzer_dir = "";
      if(spin_rad_d > 0)
      {
        throw_spin_dir = "CCW";
        if(hyzer_rad < 0)
        {
          hyzer_dir = "ANHYZER";
        }
        else
        {
          hyzer_dir = "HYZER";
        }
      }
      else
      {
        throw_spin_dir = "CW";
        if(hyzer_rad > 0)
        {
          hyzer_dir = "ANHYZER";
        }
        else
        {
          hyzer_dir = "HYZER";
        }
      }

      std::string throw_wobble = "";
      if(kf_state.wobble_mag > 0.8)
      {
        throw_wobble = "ULTRA";
      }
      else if(kf_state.wobble_mag > 0.6)
      {
        throw_wobble = "SUPER";
      }
      else if(kf_state.wobble_mag > 0.4)
      {
        throw_wobble = "LOTS OF";
      }
      else if(kf_state.wobble_mag > 0.2)
      {
        throw_wobble = "MODERATE";
      }
      else if(kf_state.wobble_mag > 0.1)
      {
        throw_wobble = "MILD";
      }
      else
      {
        throw_wobble = "LOW";
      }

      // deprecated, and replaced with the more generic 'dvd_DvisEst_display_text'

      // sadly, sm can't handle negative numbers (sigh), so well directionalize everything
      /*sprintf(output_cmd, "echo '%0.1f kph SPEED \\r\\n%0.1f° %s  %0.1f° %s \\r\\n%0.1f° %s\\r\\n%0.1f rad/s %s\\r\\n%s WOBBLE' > /tmp/disptext; timeout 12s sm -a 1 `cat /tmp/disptext` &",
        vel_mag_kph,
        fabs(throw_deg_up),
        throw_ud.c_str(),
        fabs(throw_deg_right),
        throw_left_right.c_str(),
        fabs(hyzer_rad),
        hyzer_dir.c_str(),
        fabs(spin_rad_d),
        throw_spin_dir.c_str(),
        throw_wobble.c_str()
        );
      */

      const int num_strings = 5;
      
      std::string text_to_show[num_strings] = {""};
      sprintf(output_cmd, "%0.1f kph SPEED", 
        vel_mag_kph);
      text_to_show[0] = output_cmd;

      sprintf(output_cmd, "%0.1f deg %s  %0.1f deg %s", 
        fabs(throw_deg_up),
        throw_ud.c_str(),
        fabs(throw_deg_right),
        throw_left_right.c_str());
      text_to_show[1] = output_cmd;

      sprintf(output_cmd, "%0.1f deg %s", 
        fabs(hyzer_rad),
        hyzer_dir.c_str());
      text_to_show[2] = output_cmd;

      sprintf(output_cmd, "%0.1f rad/s %s", 
        fabs(spin_rad_d),
        throw_spin_dir.c_str());
      text_to_show[3] = output_cmd;

      sprintf(output_cmd, "%s WOBBLE", 
        throw_wobble.c_str());
      text_to_show[4] = output_cmd;

      dvd_DvisEst_display_text(text_to_show, num_strings, 10);
    }
  }

  // do this at the end since it takes a while and we could be showing off text...
  dvd_DvisEst_image_capture_stop(camera_src);

  // if we didn't log anything, delete the logging dir
  if(debug && !log_debug_path.empty())
  {
    cv::glob(log_debug_path,fn,true);
    if(fn.size() == 0)
    {
      cv::utils::fs::remove_all(log_debug_path);
      cerr << "Deleted logging folder '" << log_debug_path << "'" << endl;
    }
  }

  return 0;
}

