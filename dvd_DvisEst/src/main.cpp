#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32)) && !defined(SPINNAKER_ALLOWED)
// not available in mingw64 for windows! (sad)
// I'm starting to think spinnaker and apriltag are never meant to
// be together on windows...
#else
#define SPINNAKER_ALLOWED
#endif

#include <string>
#include <iostream>

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
    "{dt             |(1.0/522.0)| Nominal value of dt (in seconds) between frames.\n"
                  "                (only used when fromimgsdir is populated and dt is unknown)}"
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
  const double      dt_src      = parser.get<double>("dt");

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

  // check for test images directory
  if(!imgdir_src.empty())
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


    // try to read some images for fun (temporary to checking)
    cerr << "Got images, now show 'em" << endl;
    image_capture_t image_capture;
    bool got_one;
    while(1)
    {
      got_one = dvd_DvisEst_image_capture_get_next_image_capture(&image_capture);

      if(got_one)
      {
        imshow("Image View", image_capture.image_data);
        if(1)
        {
          char c = (char)cv::waitKey();
          if( c == 27 || c == 'q' || c == 'Q' )
              break;
        }
      }
      else
      {
        cerr << "Done imreading queue!" << endl;
        return 0;
      }
    }    

  }
  else
  {
    // Not running a test from image files?
    // Then we are actively capturing from our capture thread
    // and queueing frames when available
    // Note: This thread only returns when our KF thread indicates that
    // the initial disc states have stabilized
  }

  //waitKey(0);

  return 0;
}