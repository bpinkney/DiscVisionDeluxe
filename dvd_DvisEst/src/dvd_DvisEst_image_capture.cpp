#if defined(WIN32) || defined(_WIN32) || defined(__WIN32)
// not available in mingw64 for windows! (sad)
// I'm starting to think spinnaker and apriltag are never meant to
// be together on windows...
#else
#define SPINNAKER_ALLOWED
#endif

// OpenCV stuff
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>

#if defined(SPINNAKER_ALLOWED)
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#endif

// threading stuff
//#include <thread>
//#include <future>
//#include <queue>
//#include <mutex>
//#include <atomic>

#include <dvd_DvisEst_image_capture.hpp>

#if defined(SPINNAKER_ALLOWED)
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
#endif

using namespace std;
using namespace cv;


bool dvd_DvisEst_image_capture_test(void)
{
#if defined(SPINNAKER_ALLOWED)
  // Retrieve singleton reference to system object
  SystemPtr system = System::GetInstance();

  // Print out current library version
  const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
  /*cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
     << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
     << endl;*/

  // Retrieve list of cameras from the system
  CameraList camList = system->GetCameras();

  unsigned int numCameras = camList.GetSize();

  //cout << "Number of cameras detected: " << numCameras << endl << endl;

  return true;
#else
  return false;
#endif
}
