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
#include <csignal>
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

std::atomic<bool> gv_force_continuous_mode (false);
std::atomic<bool> gv_force_complete_threads (false);

bool gv_handle_camera_on_sigint (false);

void signal_handler(int signum) 
{
  cerr << "Interrupt signal (" << signum << ") received.\n";
  cout << "ready:0," << endl;
  cout << "error:" << (int)dvd_DvisEst_error::PROGRAM_TERMINATED << "," << endl;
  gv_force_complete_threads = true;
  if(gv_handle_camera_on_sigint)
  {
    dvd_DvisEst_image_capture_set_force_capture_thread_closure(true);
  }
  dvd_DvisEst_apriltag_end();
  if(gv_handle_camera_on_sigint)
  {
    dvd_DvisEst_image_capture_stop(true);
  }
  dvd_DvisEst_estimate_end_filter();

  exit(signum);
}

// timer overloads for windows
#if defined(IS_WINDOWS)

static void usleep(__int64 usec) 
{ 
    /*HANDLE timer; 
    LARGE_INTEGER ft; 

    ft.QuadPart = -(10*usec); // Convert to 100 nanosecond interval, negative value indicates relative time

    timer = CreateWaitableTimer(NULL, TRUE, NULL); 
    SetWaitableTimer(timer, &ft, 0, NULL, NULL, 0); 
    WaitForSingleObject(timer, INFINITE); 
    CloseHandle(timer); */
  std::this_thread::sleep_for(std::chrono::microseconds(usec));
}

/*BOOL WINAPI CtrlHandler(DWORD fdwCtrlType)
{
    switch (fdwCtrlType)
    {
        // Handle the CTRL-C signal. 
    case CTRL_C_EVENT:
        cerr << "Ctrl-C event" << std::endl;
        signal_handler(1);
        return TRUE;

        // CTRL-CLOSE: confirm that the user wants to exit. 
    case CTRL_CLOSE_EVENT:
        cerr << "Ctrl-Close event" << std::endl;
        signal_handler(1);
        return TRUE;

        // Pass other signals to the next handler. 
    case CTRL_BREAK_EVENT:
        cerr << "Ctrl-Break event" << std::endl;
        signal_handler(1);
        return FALSE;

    case CTRL_LOGOFF_EVENT:
        cerr << "Ctrl-Logoff event" << std::endl;
        signal_handler(1);
        return FALSE;

    case CTRL_SHUTDOWN_EVENT:
        cerr << "Ctrl-Shutdown event" << std::endl;
        signal_handler(1);
        return FALSE;

    default:
        cerr << "Ctrl-DEFAULT event????" << std::endl;
        return FALSE;
    }
}

LRESULT CALLBACK CtrlHandlerHook (int nCode, WPARAM wParam, LPARAM lParam)
{
  cerr << "Ctrl-WM_CLOSE detected!" << std::endl;
  return NULL;
}*/


#endif

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

static void dvd_DvisEst_display_rotate_mat(cv::Mat& src, cv::Mat& dst, double angle)
{
  cv::Point2f ptCp(src.cols, src.rows*0.5);
  cv::Mat M = cv::getRotationMatrix2D(ptCp, angle, 1.0);
  cv::warpAffine(src, dst, M, src.size(), cv::INTER_CUBIC); //Nearest is too rough, 
}

static void dvd_DvisEst_display_text(const std::string * text_to_show, const int num_strings, const int time_s, const float hyzer_rad, const float pitch_rad, const float VEC3(lin_vel_xyz))
{
  int res_x;
  int res_y;
  dvd_DvisEst_get_desktop_resolution(&res_x, &res_y);
  cerr << "Res: " << res_x << ", " << res_y << endl;

  // build stats display matrix
  // set background to light grey
  const int base_colour = 240;

  cv::Mat throw_stats(cv::Size(res_x, res_y), CV_8UC3, cv::Scalar(base_colour,base_colour,base_colour));

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

  // add ellipse behind text to denote disc angle (hyzer and pitch)
  cv::Mat disc_ellipse(cv::Size(res_x, res_y), CV_8UC3, cv::Scalar(255,255,255));

  // main disc body
  cv::ellipse(disc_ellipse,cv::Point(res_x/2, res_y/2), cv::Size(res_x/3, res_x/3 * max(abs(sin(pitch_rad)), (float)0.04)), 0 + RAD_TO_DEG(hyzer_rad), 0, 360, cv::Scalar(base_colour-30,base_colour-30,base_colour-30), -1, cv::FILLED, 0);
  // lippy drop shadow
  cv::ellipse(disc_ellipse,cv::Point(res_x/2, res_y/2), cv::Size(res_x/3, res_x/3 * max(abs(sin(pitch_rad)), (float)0.06)), 180 + RAD_TO_DEG(hyzer_rad), 180, 360, cv::Scalar(base_colour-60,base_colour-60,base_colour-60), -1, 8, 0);
  
  if(pitch_rad > 0)
  {
    cv::ellipse(disc_ellipse,cv::Point(res_x/2, res_y/2), cv::Size(res_x/3, res_x/3 * (max(abs(sin(pitch_rad)), (float)0.04) - 0.04)), 180 + RAD_TO_DEG(hyzer_rad), 180, 360, cv::Scalar(base_colour-30,base_colour-30,base_colour-30), -1, 8, 0);
  }
  else
  {
    cv::ellipse(disc_ellipse,cv::Point(res_x/2, res_y/2), cv::Size(res_x/3, res_x/3 * (max(abs(sin(pitch_rad)), (float)0.04) - 0.04)), 180 + RAD_TO_DEG(hyzer_rad), 0, 180, cv::Scalar(base_colour-60,base_colour-60,base_colour-60), -1, 8, 0);
  }

  // darken with ellipse
  throw_stats = min(throw_stats, disc_ellipse);
  // end disc ellipse

  // add an arrow to denote the linear throw vector
  const float vel_mag = sqrt(lin_vel_xyz[0]*lin_vel_xyz[0] + lin_vel_xyz[1]*lin_vel_xyz[1] + lin_vel_xyz[2]*lin_vel_xyz[2]);
  const float base_arrow_length_3d = res_y;
  const float arrow_length_camx_wfy = lin_vel_xyz[1] / vel_mag * base_arrow_length_3d;
  const float arrow_length_camy_wfz = -lin_vel_xyz[2] / vel_mag * base_arrow_length_3d;
  const float max_speed = 30.0; // m/s base speed to adjust colours to
  const float factor = max(max_speed - vel_mag, (float)0.0) / max_speed;
  const int red  = (int)((1.0 - factor) * 255.0);
  const int blue = (int)(factor * 255.0);


  // aspect the arrowhead size based on how forward the shot is
  const float arrowhead_tip_scale = sqrt(lin_vel_xyz[1]*lin_vel_xyz[1] + lin_vel_xyz[2]*lin_vel_xyz[2]) / MAX(fabs(lin_vel_xyz[0]), CLOSE_TO_ZERO);

  cv::arrowedLine(throw_stats, cv::Point(res_x/2, res_y/2), cv::Point(res_x/2 + arrow_length_camx_wfy, res_y/2 + arrow_length_camy_wfz), CV_RGB(red, 0, blue), 3, 8, 0, arrowhead_tip_scale);
  // end throw vector arrow

  // add a pointless catchphrase behind the output text for no reason
  srand(time(NULL));
  const int catchphrase_lotto = rand() % 9;
  const std::string catchphrases[9] = 
  {
    " MONDO COOL",
    " SIC THRO BRO",
    " ACE RUN BOYZZ",
    " YO, RAD DISC'N",
    " MANDO COOL",
    " GIT SUM!",
    " TREE-MENDOUS!",
    " FROLF IT UP SON",
    " GIT'N PRINGLED!"
  };
  const std::string catchphrase = catchphrases[catchphrase_lotto];
  //cerr << "catch: " << catchphrase_lotto << " -> " << catchphrase << endl;
  const float catchphrase_line_length = base_letter_width * catchphrase.length();
  const float catchphrase_width_mult = (float)res_x / catchphrase_line_length * 0.95;// * height_scale;
  // Create and rotate the text
  cv::Mat catchphrase_text(cv::Size(res_x, res_y), CV_8UC3, cv::Scalar(0,0,0));
  #if defined(IS_WINDOWS)
  cv::putText(catchphrase_text, catchphrase, cv::Point(0, throw_stats.cols*2/3), cv::FONT_HERSHEY_PLAIN, text_size * catchphrase_width_mult, cv::Scalar(base_colour+10,base_colour+10,base_colour+10), 2.0 * text_thickness * catchphrase_width_mult);
  #else
  cv::putText(catchphrase_text, catchphrase, cv::Point(0, throw_stats.cols/2), cv::FONT_HERSHEY_PLAIN, text_size * catchphrase_width_mult, cv::Scalar(base_colour+10,base_colour+10,base_colour+10), 2.0 * text_thickness * catchphrase_width_mult);
  #endif
  //const int rot_dir = (rand() % 2)*2-1;
  dvd_DvisEst_display_rotate_mat(catchphrase_text, catchphrase_text, -45.0 * (float)res_y/(float)res_x);
  // Sum the images (add the text to the original img)
  // brighten with text
  throw_stats = max(throw_stats, catchphrase_text);
  // end pointless catchphrase

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

static std::string get_datestring(void)
{
  // get date string for logging
  std::time_t now = std::time(NULL);
  std::tm * ptm = std::localtime(&now);
  string datestring;// = std::ctime(&end_time);
  char buffer[32];
  std::strftime(buffer, 32, "%Y-%m-%d_%H-%M-%S", ptm);
  datestring = buffer;
  return datestring;
}

static void set_logging_dir(void)
{
  cv::String executable_path = get_executable_path();
  std::string datestring = get_datestring();
  #if defined(IS_WINDOWS)
  std::string log_path = executable_path + "\\logs\\";
  #else
  std::string log_path = executable_path + "/logs/";
  #endif
  //replace_slashes_linux_to_windows(&log_path);
  cv::utils::fs::createDirectory(log_path);
  #if defined(IS_WINDOWS)
  std::string log_debug_path = log_path + datestring + "_log_data\\";
  #else
  std::string log_debug_path = log_path + datestring + "_log_data/";
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
    "{contrun cr     |false      | Keep outputting values, and resetting the estimate thread states, until a SIGINT is recieved}"
    "{randmeas rm    |0          | If contrun is enabled, this can also optionally be enabled to provide random dummy throws every N seconds, 0 is disabled}"
    "{nocam nc       |false      | If you want to run contrun and randmeas without a camera connected at all, enable thisn option}"
    ;

  cv::CommandLineParser parser(argc, argv, keys);
  if (parser.has("help"))
  {
    parser.printMessage();
    return 0;
  }

  // flag init condition on the stdout commandline
  cout << "init:0," << endl;

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
  const bool        contrun     = parser.get<bool>("contrun");
  const int         randmeas    = parser.get<int>("randmeas");
  const bool        nocam       = parser.get<bool>("nocam");

  // register signal SIGINT and signal handler if we need to de-init camera
  /*#ifdef IS_WINDOWS
  SetConsoleCtrlHandler(CtrlHandler, TRUE);
  //SetWindowsHookEx(WM_CLOSE, CtrlHandlerHook, TRUE);
  HINSTANCE hInstance = GetModuleHandle(NULL);
  SetWindowsHookEx(WM_CLOSE, CtrlHandlerHook, hInstance, NULL);
  SetWindowsHookEx(WM_QUIT, CtrlHandlerHook, hInstance, NULL);
  SetWindowsHookEx(WM_DESTROY, CtrlHandlerHook, hInstance, NULL);
  #else*/
  signal(SIGINT, signal_handler);
  //#endif

  gv_handle_camera_on_sigint = camera_src && !nocam;

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

  if(randmeas > 0)
  {
    cerr << "Using randmeas period of " << randmeas << " seconds!" << endl;
  }

  if(nocam)
  {
    cerr << "NOCAM option is enabled!" << endl;
  }

  // Set up directories
  string datestring = get_datestring();
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

  if(camera_src && !nocam)
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
    camera_cal = executable_path + "\\camera_calibrations/19508898.yaml";
    #else
    camera_cal = executable_path + "/camera_calibrations/19508898.yaml";
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
      while(!gv_force_complete_threads)
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

  // Init remaining calls and loop if required
  if(debug)
  {
    set_logging_dir();
  }

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
  if(camera_src && !nocam)
  {
    // Not running a test from image files?
    // Then we are actively capturing from our capture thread
    // and queueing frames when available
    // Note: This thread only joins when our KF thread indicates that
    // the initial disc states have stabilized

    // start image capture thread
    dvd_DvisEst_image_capture_start();
  }
  else
  {
    // fake out init
    cout << "init:1," << endl;
  }
  
  if(contrun)
  {
    cerr  << endl << endl << endl << endl << endl << "CONTINUOUS RUN IS ENABLED!!!!" << endl << endl << endl << endl << endl;
    // force threads to keep running in continuous mode
    gv_force_continuous_mode = true;
  }

  while(!gv_force_complete_threads)
  {
    if(contrun && !set_gnd_plane)
    {
      // re-allocate measurement slots and prep Kalman Filter initial states
      if(!dvd_DvisEst_estimate_init(debug))
      {
        cerr << "Can't read ground plane, exiting...!" << endl;
        return 1;
      }
    }

    // We should be calling reverse-order thread join/destroy functions here
    // Once the filter thread returns, we know we can wrap everything up
    cerr << "Waiting Until threads join..." << endl;
    bool got_output = false;
    char output_cmd[512] = {0};
    dvd_DvisEst_kf_state_t kf_state;

    if(!set_gnd_plane)
    {
      // wait here until estimate thread is complete
      if(contrun)
      {
        uint32_t last_randmeas_time_ms = NS_TO_MS(uptime_get_ns());
        bool ready_rm = true;
        while(!gv_force_complete_threads)
        {
          // we're holding threads open for continuous mode, so just check the state of the estimate before continuing
          if(dvd_DvisEst_estimate_complete())
          {
            cerr << "ESTIMATECOMPLETE!!!!" << endl;
            break;
          }

          // if random debug outputs are enabled, generate one every randmeas seconds
          // (resets time after a real throw to give it some space!)
          if(randmeas > 0)
          {
            uint32_t now_ms  = NS_TO_MS(uptime_get_ns());
            // pre-empt the 'ready' flag a bit, just so the UI can actually do something with it
            if(now_ms > last_randmeas_time_ms + S_TO_MS(randmeas) - 500 && ready_rm)
            {
              ready_rm = false;
              cout << "ready:0," << endl;
            }

            if(now_ms > last_randmeas_time_ms + S_TO_MS(randmeas))
            {
              srand(time(NULL));
              const int base = rand() % 40;
              const float s1 = ((float)(base)) - 40.0/2.0 + 60.0;
              const float signer = ((float)(rand() % 2))*2.0-1.0;
              const float spin_d = s1 * signer;       
              // output a random throw, along with a ready flag on either side
              sprintf(output_cmd, "posx:%0.3f,posy:%0.3f,posz:%0.3f,velx:%0.3f,vely:%0.3f,velz:%0.3f,hyzer:%0.5f,pitch:%0.5f,spin_d:%0.5f,wobble:%0.3f,discmold:%d,",
                MM_TO_M((float)(rand() % 500) - 500/2),
                MM_TO_M((float)(rand() % 500) - 500/2),
                MM_TO_M((float)(rand() % 300) - 300/2) + 1.5, // standing up off the ground by 1.5 default
                MM_TO_M((float)(rand() % 10000) - 10000/2) + 18.0, //xvel
                MM_TO_M((float)(rand() % 8000) - 8000/2),
                MM_TO_M((float)(rand() % 6000) - 6000/3), // bias up for zvel
                DEG_TO_RAD((float)(rand() % 60) - 60/2), // +- 30 deg hyzer
                DEG_TO_RAD((float)(rand() % 30) - 30/2 + 10), // +- 15 deg pitch, biased up by 10 deg
                spin_d, // 60 +- 20 rad/s either positive or negative
                ((float)(rand() % 600)) * 0.001,
                (rand() % 10 + 3)
                );
              // stdout!
              cout << output_cmd << endl;
              cout << "ready:1," << endl;
              ready_rm = true;

              last_randmeas_time_ms = now_ms;
            }
          }

          // sleep for a bit so we don't busy poll
          usleep(2000);
        }
      }
      else
      {
        // we are not holding this therad open if in single-run mode, so just wait for the join
        dvd_DvisEst_estimate_end_filter();
      }

      // get final output state
      got_output = dvd_DvisEst_estimate_get_ideal_output_state(&kf_state);

      // gate output reporting on minimum speed of 10kph
      const float lin_vel_mag_kph = 
        sqrt(
          kf_state.lin_xyz[0].vel*kf_state.lin_xyz[0].vel +
          kf_state.lin_xyz[1].vel*kf_state.lin_xyz[1].vel +
          kf_state.lin_xyz[2].vel*kf_state.lin_xyz[2].vel
          ) * 3.6;
      if(got_output)
      {
        if(lin_vel_mag_kph < 5.0)
        {
          got_output = false;
        }
      }

      if(got_output)
      {
        // we should print this state ASAP for consumption by Mike's dvd_DfisX
        sprintf(output_cmd, "posx:%0.3f,posy:%0.3f,posz:%0.3f,velx:%0.3f,vely:%0.3f,velz:%0.3f,hyzer:%0.5f,pitch:%0.5f,spin_d:%0.5f,wobble:%0.3f,discmold:%d,",
          kf_state.lin_xyz[0].pos,
          kf_state.lin_xyz[1].pos,
          kf_state.lin_xyz[2].pos,
          kf_state.lin_xyz[0].vel,
          kf_state.lin_xyz[1].vel,
          kf_state.lin_xyz[2].vel,
          kf_state.ang_hps[0].pos,
          kf_state.ang_hps[1].pos,
          kf_state.ang_hps[2].vel,
          kf_state.wobble_mag,
          (int)kf_state.disc_index
          );
        // stdout!
        cout << output_cmd << endl;
      }
      else
      {
        //cout << "NO OUTPUT!" << endl << endl;
      }

      // also pipe this into the metadata file
      if(debug)
      {
        std::stringstream ss_metadata;
        ss_metadata << "FILEPATH=$(ls " << log_debug_path << "metadata*); echo \"" << output_cmd << "\" >> " << "$FILEPATH";
        std::system(ss_metadata.str().c_str());
      }

      cerr << endl << endl;
    }

    if(!contrun)
    {
      dvd_DvisEst_apriltag_end();
    }

    if(!set_gnd_plane)
    {
      if(!got_output && !gianttext)
      {
        cerr << ("No ouput state! Sorry!\n") << endl;
      }

      // plotting with matlab!
      if(matlab && debug && !dfisx && !contrun && got_output)
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

      if(got_output)
      {
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
          kf_state.wobble_mag,
          (int)kf_state.disc_index
          );

        cerr << "Output String: " << output_cmd << endl;
      }

      // run Skinner's stuff
      if(dfisx && !contrun && got_output)
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

      if(gianttext && got_output)
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

        const float VEC3(lin_vel_xyz) = 
        {
          (float)kf_state.lin_xyz[0].vel,
          (float)kf_state.lin_xyz[1].vel,
          (float)kf_state.lin_xyz[2].vel
        };
        dvd_DvisEst_display_text(text_to_show, num_strings, 10, kf_state.ang_hps[0].pos, kf_state.ang_hps[1].pos, lin_vel_xyz);
      }
    }

    // do this at the end since it takes a while and we could be showing off text...
    if(!contrun)
    {
      dvd_DvisEst_image_capture_stop(camera_src);
    }

    // don't look for ground plane, or for non-continueous running
    if(set_gnd_plane || !contrun)
    {
      break;
    }
  }

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

