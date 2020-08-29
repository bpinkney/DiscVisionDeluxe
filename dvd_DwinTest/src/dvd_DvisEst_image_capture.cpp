#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(WIN64) || defined(_WIN64) || defined(__WIN64))
#define IS_WINDOWS
#endif

#if (defined(IS_WINDOWS) && !defined(SPINNAKER_ALLOWED))
// not available in mingw64 for windows! (sad)
// I'm starting to think spinnaker and apriltag are never meant to
// be together on windows...
#else
#define SPINNAKER_ALLOWED
#endif

// OpenCV stuff
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <stdio.h>
#include <math.h>
#include <string>
#include <iostream>
#include <sstream>

#if defined(SPINNAKER_ALLOWED)
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#endif

// Timer stuff
#if !defined(IS_WINDOWS)
#include <unistd.h>
#endif
#include <chrono>
#define _BSD_SOURCE
#if !defined(IS_WINDOWS)
#include <sys/time.h>
#endif
#include <stack>
#include <ctime>

// threading stuff
#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <atomic>

#include <dvd_DvisEst_image_capture.hpp>
#include <dvd_DvisEst_apriltag.hpp>

#if defined(SPINNAKER_ALLOWED)
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
#endif

using namespace std;
using namespace cv;

// external functions
bool dvd_DvisEst_image_capture_test(void)
{
#if defined(SPINNAKER_ALLOWED)
  // Retrieve singleton reference to system object
  SystemPtr system = System::GetInstance();

  // Print out current library version
  const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();

  // Retrieve list of cameras from the system
  CameraList camList = system->GetCameras();

  unsigned int numCameras = camList.GetSize();

  return true;
#else
  return false;
#endif
}
