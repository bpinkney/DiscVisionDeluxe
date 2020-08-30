#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(WIN64) || defined(_WIN64) || defined(__WIN64))
#define IS_WINDOWS
#endif

//#if (defined(IS_WINDOWS) && !defined(SPINNAKER_ALLOWED))
// not available in mingw64 for windows! (sad)
// I'm starting to think spinnaker and apriltag are never meant to
// be together on windows...
//#else
#define SPINNAKER_ALLOWED
//#endif

#include <string>
#include <iostream>
#if !defined(IS_WINDOWS)
#include <unistd.h>
#endif
#include <iomanip>
#include <chrono>
#include <ctime>

#include <atomic>

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

using namespace std;
using namespace cv;

int main(int argc, char** argv )
{
  // grab commandline args
  const bool        helloworld  = true;

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

  return 0;
}