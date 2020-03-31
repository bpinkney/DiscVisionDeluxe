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
std::queue<image_capture_t> sv_image_capture_queue;
std::mutex                  sv_image_capture_mutex;
//std::atomic<bool> ai_thread_ready (false);

// start thread stuff
static bool image_queue_empty()
{
  bool empty;
  // mutex on
  sv_image_capture_mutex.lock();  
  empty = sv_image_capture_queue.empty();
  // mutex off
  sv_image_capture_mutex.unlock();

  return empty;
}

static void image_queue_push(image_capture_t * image_capture)
{
  // mutex on
  sv_image_capture_mutex.lock();
  // no deep copy required here?
  sv_image_capture_queue.push(*image_capture);

  //cerr << "Queue size: " << sv_image_capture_queue.size() << endl;
  // mutex off
  sv_image_capture_mutex.unlock();
}

static bool image_queue_pull(image_capture_t * image_capture)
{   
  // mutex on
  sv_image_capture_mutex.lock();
  if(sv_image_capture_queue.empty())
  {
    // mutex off
    sv_image_capture_mutex.unlock();
    image_capture->timestamp_ns = 0;
    image_capture->frame_id     = 0;
    return false;
  }
  else
  {
    // no deep copy required here I think
    (*image_capture) = sv_image_capture_queue.front();
    sv_image_capture_queue.pop();
    // mutex off
    sv_image_capture_mutex.unlock();
    return true;
  }
}
//end thread stuff

// convert Spinnaker frame to OpenCV Mat
// this requires a deep copy since the Mat constructor doesn't copy the
// input data
#if defined(SPINNAKER_ALLOWED)
void spinnaker_image_to_opencv_mat(ImagePtr spinnaker_image, cv::Mat * opencv_image)
{
  try
  {
    const auto XPadding = spinnaker_image->GetXPadding();
    const auto YPadding = spinnaker_image->GetYPadding();
    const auto rowsize = spinnaker_image->GetWidth();
    const auto colsize = spinnaker_image->GetHeight();
    
    // spinnaker_image is either grayscale or BayerRG pending camera selection
    cv::Mat image_local =
      cv::Mat
      (
        (int)(colsize + YPadding), 
        (int)(rowsize + XPadding), 
        CV_8UC1, 
        spinnaker_image->GetData(),
        spinnaker_image->GetStride()
      );

    image_local.copyTo(*opencv_image);
    image_local.release();
  }
  catch (const std::exception& e)
  {
    cerr << endl << endl << "*** Could not convert Spinnaker Image Pointer to OpenCV MAT! ***" << endl << endl;
  }
}
#endif

#if defined(SPINNAKER_ALLOWED)
// Disable everything preventing us from achieving the full frame rate
double camera_settings_configure(INodeMap& nodeMap, const double exposure_scale, const double gain_scale)
{
  double acquisitionResultingFrameRate = 0;

  cerr << endl << endl << "*** CONFIGURING EXPOSURE, GAIN, WHITE BALANCE, and ADC bitness ***" << endl << endl;
  try
  {
    // Set ADC bitness to 8-bits
    CEnumerationPtr ptrAdcBitDepth = nodeMap.GetNode("AdcBitDepth");
    if (!IsAvailable(ptrAdcBitDepth) || !IsWritable(ptrAdcBitDepth))
    {
      cerr << "Unable to set adc bit depth to 8-bits (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrAdcBitDepthBit8 = ptrAdcBitDepth->GetEntryByName("Bit8");
    if (!IsAvailable(ptrAdcBitDepthBit8) && !IsReadable(ptrAdcBitDepthBit8))
    {
      cerr << "Unable to set adc bit depth to 8-bits (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrAdcBitDepth->SetIntValue(ptrAdcBitDepthBit8->GetValue());
    cerr << "ADC bitness set to 8-bits..." << endl;

    // Turn off automatic exposure mode
    CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
    {
      cerr << "Unable to disable automatic exposure (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
    {
      cerr << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
    cerr << "Automatic exposure disabled..." << endl;

    // We know that the output framerate of the camera is intrinsically limited 
    // by the exposure setting
    // Try to calculate this explicitly
    double des_frame_rate = 522.0;
    double max_exp_time_s = 1.0 / des_frame_rate;

    // Since the frame capture is composed of more than just the shutter speeed
    // add a de-rate factor here to account for overhead
    // e.g. 1.1 assumes that 1/10 of the exposure time is required for frame capture and admin
    // tested experimentally at 522fps to be between 6-7%
    const double frame_capture_overhead_factor = 1.07;
    // Exposure scale (0-1) reduces the exposure further for faster capture
    // TODO: We should determine a way to set this explicitly with an init function later!
    const double des_exposure_us = max_exp_time_s * 1000000.0 * exposure_scale / frame_capture_overhead_factor;

    CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
    {
      cerr << "Unable to set exposure time. Aborting..." << endl << endl;
      return -1;
    }

    // Ensure desired exposure time does not exceed the maximum
    const double exposureTimeMax = ptrExposureTime->GetMax();
    const double exposureTimeMin = ptrExposureTime->GetMin();
    double exposureTimeToSet = des_exposure_us;

    if (exposureTimeToSet > exposureTimeMax)
    {
      exposureTimeToSet = exposureTimeMax;
    }
    if (exposureTimeToSet < exposureTimeMin)
    {
      exposureTimeToSet = exposureTimeMin;
    }

    ptrExposureTime->SetValue(exposureTimeToSet);

    // How can we verify that the gain is actually changing here to make up for the low fixed exposure?
    // Perhaps read the gain values out and wait for convergence? (likely)
    // We can now proceed with disabling the remaining frame-rate inhibiting features
    // Turn off automatic gain
    CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
    if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
    {
      cerr << "Unable to disable automatic gain (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrGainAutoOff) && !IsReadable(ptrGainAutoOff))
    {
      cerr << "Unable to disable automatic gain (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());
    cerr << "Automatic gain disabled..." << endl;

    // set gain
    CFloatPtr ptrGain = nodeMap.GetNode("Gain");
    if (!IsAvailable(ptrGain) || !IsReadable(ptrGain))
    {
      cerr << "Unable to get/set gain. Aborting..." << endl << endl;
      return -1;
    }

    double des_gain = 1.0 * gain_scale;    
    const double gain_max = ptrGain->GetMax();
    const double gain_min = ptrGain->GetMin();
    if (des_gain > gain_max)
    {
      des_gain = gain_max;
    }
    if (des_gain < gain_min)
    {
      des_gain = gain_min;
    }
    ptrGain->SetValue(des_gain);

    // Check gain value        
    const float gain = static_cast<float>(ptrGain->GetValue());
    cerr << "Gain: " << gain << endl;

    // Turn off automatic white balance
    CEnumerationPtr ptrBalanceWhiteAuto = nodeMap.GetNode("BalanceWhiteAuto");
    if (!IsAvailable(ptrBalanceWhiteAuto) || !IsWritable(ptrBalanceWhiteAuto))
    {
      cerr << "Unable to disable automatic white balance (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrBalanceWhiteAutoOff = ptrBalanceWhiteAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrBalanceWhiteAutoOff) && !IsReadable(ptrBalanceWhiteAutoOff))
    {
      cerr << "Unable to disable automatic white balance (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrBalanceWhiteAuto->SetIntValue(ptrBalanceWhiteAutoOff->GetValue());
    cerr << "Automatic white balance disabled..." << endl;

    // Check possible frame rate now that we have disabled these features
    CFloatPtr ptrAcquisitionResultingFrameRate = nodeMap.GetNode("AcquisitionResultingFrameRate");
    if (!IsAvailable(ptrAcquisitionResultingFrameRate) || !IsReadable(ptrAcquisitionResultingFrameRate))
    {
      cerr << "Unable to retrieve frame rate. Aborting..." << endl << endl;
      return -1;
    }

    acquisitionResultingFrameRate = static_cast<double>(ptrAcquisitionResultingFrameRate->GetValue());
    cerr << "Max Acquisition Frame Rate: " << acquisitionResultingFrameRate << endl;
    
  }
  catch (Spinnaker::Exception& e)
  {
    cerr << "Error: " << e.what() << endl;
    return -1;
  }

  return acquisitionResultingFrameRate;
}
#endif


#if defined(SPINNAKER_ALLOWED)
// This function returns the camera to its default state by re-enabling automatic
// exposure.
int camera_settings_reset(INodeMap& nodeMap)
{
  int result = 0;
  try
  {
    // Turn automatic exposure back on
    CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
    {
      cerr << "Unable to enable automatic exposure (node retrieval). Non-fatal error..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous))
    {
      cerr << "Unable to enable automatic exposure (enum entry retrieval). Non-fatal error..." << endl << endl;
      return -1;
    }

    ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());
    cerr << "Automatic exposure enabled..." << endl;

    // Turn automatic gain back on
    CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
    if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
    {
      cerr << "Unable to enable automatic gain (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrGainAutoContinuous = ptrGainAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrGainAutoContinuous) && !IsReadable(ptrGainAutoContinuous))
    {
      cerr << "Unable to enable automatic gain (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrGainAuto->SetIntValue(ptrGainAutoContinuous->GetValue());
    cerr << "Automatic gain enabled..." << endl;


    // Turn on automatic white balance
    CEnumerationPtr ptrBalanceWhiteAuto = nodeMap.GetNode("BalanceWhiteAuto");
    if (!IsAvailable(ptrBalanceWhiteAuto) || !IsWritable(ptrBalanceWhiteAuto))
    {
      cerr << "Unable to disable automatic white balance (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrBalanceWhiteAutoContinuous = ptrBalanceWhiteAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrBalanceWhiteAutoContinuous) && !IsReadable(ptrBalanceWhiteAutoContinuous))
    {
      cerr << "Unable to disable automatic white balance (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrBalanceWhiteAuto->SetIntValue(ptrBalanceWhiteAutoContinuous->GetValue());
    cerr << "Automatic white balance enabled..." << endl;


    // Set ADC bitness to 10-bits
    CEnumerationPtr ptrAdcBitDepth = nodeMap.GetNode("AdcBitDepth");
    if (!IsAvailable(ptrAdcBitDepth) || !IsWritable(ptrAdcBitDepth))
    {
      cerr << "Unable to set adc bit depth to 10-bits (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrAdcBitDepthBit10 = ptrAdcBitDepth->GetEntryByName("Bit10");
    if (!IsAvailable(ptrAdcBitDepthBit10) && !IsReadable(ptrAdcBitDepthBit10))
    {
      cerr << "Unable to set adc bit depth to 10-bits (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrAdcBitDepth->SetIntValue(ptrAdcBitDepthBit10->GetValue());
    cerr << "ADC bitness set to 10-bits..." << endl;
  }
  catch (Spinnaker::Exception& e)
  {
    cerr << "Error: " << e.what() << endl;
    result = -1;
  }

  return result;
}
#endif

// TODO: we need to actually implement all the spinnaker functions and thread handling here

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

void dvd_DvisEst_image_capture_init(void)
{
  #if defined(SPINNAKER_ALLOWED)

  // set camera params
  cerr << "Call dvd_DvisEst_image_capture_init" << endl;

  #endif
}

// Start collecting frames (this also purges remaining frames in the queue)
void dvd_DvisEst_image_capture_start(void)
{
  #if defined(SPINNAKER_ALLOWED)

  // start a capture thread

  #endif
}

// Stop collecting frames
void dvd_DvisEst_image_capture_stop(void)
{
  #if defined(SPINNAKER_ALLOWED)

  // stop a capture thread (purge frames here?)

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

  // Loop through the *jpg files found, and add them to the sv_image_capture_queue
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

    timestamp_ns += (uint64_t)(dt * 1000000000.0);
    frame_id     += 1;
  }

  return true;
}

// Return the next captured image from the front of the queue
bool dvd_DvisEst_image_capture_get_next_image_capture(image_capture_t * image_capture)
{
  if(FRAME_SKIP_TEST_N > 0)
  {
    int i;
    for(i=0;i<FRAME_SKIP_TEST_N-1;i++)
    {
      image_queue_pull(image_capture);
    }
  }

  return image_queue_pull(image_capture);
}

// Return the next captured image from the front of the queue
bool dvd_DvisEst_image_capture_image_capture_queue_empty(void)
{
  return image_queue_empty();
}
