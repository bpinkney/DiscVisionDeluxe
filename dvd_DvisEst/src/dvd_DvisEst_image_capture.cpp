#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32)) && !defined(SPINNAKER_ALLOWED)
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
#include <string>
#include <iostream>
#include <sstream>

#if defined(SPINNAKER_ALLOWED)
#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#endif

// threading stuff
#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <atomic>

#include <dvd_DvisEst_image_capture.hpp>

#if defined(SPINNAKER_ALLOWED)
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
#endif

using namespace std;
using namespace cv;

// declare static local structures
std::queue<image_capture_t> image_capture_queue;
std::mutex image_capture_mutex;
//std::atomic<bool> ai_thread_ready (false);

static bool image_queue_empty()
{
  bool empty;
  // mutex on
  image_capture_mutex.lock();  
  empty = image_capture_queue.empty();
  // mutex off
  image_capture_mutex.unlock();

  return empty;
}

static void image_queue_push(image_capture_t * image_capture)
{
  // mutex on
  image_capture_mutex.lock();
  // no deep copy required here?
  image_capture_queue.push(*image_capture);

  //cerr << "Queue size: " << image_capture_queue.size() << endl;
  // mutex off
  image_capture_mutex.unlock();
}

static bool image_queue_pull(image_capture_t * image_capture)
{   
  // mutex on
  image_capture_mutex.lock();
  if(image_capture_queue.empty())
  {
    // mutex off
    image_capture_mutex.unlock();
    image_capture->timestamp_ns = 0;
    image_capture->frame_id     = 0;
    return false;
  }
  else
  {
    // no deep copy required here I think
    (*image_capture) = image_capture_queue.front();
    image_capture_queue.pop();
    // mutex off
    image_capture_mutex.unlock();
    return true;
  }
}

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

// load test images into the capture queue and return
bool dvd_DvisEst_image_capture_load_test_queue(const cv::String imgdir_src, const double dt)
{
  // get list of images (for now, just jpgs)
  vector<cv::String> img_filenames;
  glob(imgdir_src + "/*.jpg", img_filenames);

  if(img_filenames.size() < 1)
  {
    return false;
  }

  cerr << "Found " << img_filenames.size() << " image files!" << endl;

  // Loop through the *jpg files found, and add them to the image_capture_queue
  uint64_t timestamp_ns = 0;
  uint32_t frame_id = 0;

  uint32_t i;
  cv::Mat image;
  for(i = 0; i < (int)img_filenames.size(); i++)
  {
    cerr << "Loading " << img_filenames[i] << " into image queue..." << endl;
    
    image = imread(img_filenames[i], 1);

    if(image.empty())
    {
      cerr << "For some reason " << img_filenames[i] << " wasn't read correctly!" << endl;
      continue;
    }

    image_capture_t image_capture = image_capture_t
    (
      image,
      timestamp_ns,
      frame_id
    );

    image_queue_push(&image_capture);
  }

  return true;
}

// Return the next captured image from the front of the queue
bool dvd_DvisEst_image_capture_get_next_image_capture(image_capture_t * image_capture)
{
  return image_queue_pull(image_capture);
}
