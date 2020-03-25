// OpenCV stuff
#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <sstream>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"

// threading stuff
//#include <thread>
//#include <future>
//#include <queue>
//#include <mutex>
//#include <atomic>

#include <dvd_DvisEst_image_capture.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;

bool dvd_DvisEst_image_capture_test(void)
{
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
}
