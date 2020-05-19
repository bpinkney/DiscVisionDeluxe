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
#include <math.h>
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
#include <dvd_DvisEst_apriltag.hpp>
#include <dvd_DvisEst_estimate.hpp>

#if defined(SPINNAKER_ALLOWED)
using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
#endif

using namespace std;
using namespace cv;

// declare static local structures
std::deque<image_capture_t> sv_image_capture_queue;
std::mutex                  sv_image_capture_mutex;
std::mutex                  sv_image_scerr_mutex;

std::thread                 capture_thread;

// this is only used for test images for now (since we never expect the camera to stop rolling until told to)
std::atomic<bool> capture_thread_ready (false);

std::atomic<uint32_t> sv_image_capture_frame_rate (0);
std::atomic<double>   sv_exposure_us (2100);
std::atomic<double>   sv_gain (30.0);

// static funcs
static int dvd_DvisEst_image_capture_apply_exposure_gain(CameraPtr pCam);

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

static uint16_t image_queue_size()
{
  uint16_t size;
  // mutex on
  sv_image_capture_mutex.lock();  
  size = sv_image_capture_queue.size();
  // mutex off
  sv_image_capture_mutex.unlock();

  return size;
}

static void image_queue_push(image_capture_t * image_capture)
{
  // mutex on
  sv_image_capture_mutex.lock();
  // no deep copy required here?
  sv_image_capture_queue.push_back(*image_capture);

  //cerr << "Queue size: " << sv_image_capture_queue.size() << endl;
  // mutex off
  sv_image_capture_mutex.unlock();
}

static bool image_queue_pull(image_capture_t * image_capture, const uint16_t front_offset, const bool pop_frame)
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
    const uint16_t front_offset_idx = max(0, min((int)front_offset, ((int)sv_image_capture_queue.size())-1));
    (*image_capture) = sv_image_capture_queue[front_offset_idx]; // 0 is the front of the queue
    // only pop frames when reading normally for measurements
    if(pop_frame)
    {
      sv_image_capture_queue.pop_front();
    }
    // mutex off
    sv_image_capture_mutex.unlock();
    return true;
  }
}

// pop N frames from the front of the queue
// always leave one entry
static uint16_t image_queue_purge(const uint16_t front_offset)
{
  if(image_queue_empty() || front_offset == 0)
  {
    return 0;
  } 

  // mutex on
  sv_image_capture_mutex.lock();
  int i;
  int purge_count = 0;
  for(i = 0; i < front_offset; i++)
  {
    if(sv_image_capture_queue.size() < 1)
    {
      break;
    }

    purge_count++;
    sv_image_capture_queue.pop_front();
  }
  // mutex off
  sv_image_capture_mutex.unlock();
  return (purge_count);
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
static double camera_settings_configure(CameraPtr pCam)
{
  // Retrieve GenICam nodemaps
  INodeMap& nodeMap = pCam->GetNodeMap();
  INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
  INodeMap& nodeMapTLStream = pCam->GetTLStreamNodeMap();

  //PrintDeviceInfo(nodeMapTLDevice);

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

    // set exposure and gain
    dvd_DvisEst_image_capture_apply_exposure_gain(pCam);

    // Check gain value
    CFloatPtr ptrGain = nodeMap.GetNode("Gain");
    if (!IsAvailable(ptrGain) || !IsReadable(ptrGain))
    {
      cerr << "Unable to get/set gain. Aborting..." << endl << endl;
      return -1;
    }

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
    sv_image_capture_frame_rate = acquisitionResultingFrameRate;



    //// Set acquisition mode to continuous
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
      cerr << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
      cerr << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting..." << endl
         << endl;
      return -1;
    }

    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    cerr << "Acquisition mode set to continuous..." << endl;

    // try to avoid skipping frames with this hack
    // maybe I need to query the buffers constantly? ehhh
    // Sounds like we'll just lose frames every time there isn't enough CPU...
    CEnumerationPtr ptrStreamBufferCountMode = nodeMapTLStream.GetNode("StreamBufferCountMode");
    if (!IsAvailable(ptrStreamBufferCountMode) || !IsWritable(ptrStreamBufferCountMode))
    {
      cerr << IsAvailable(ptrStreamBufferCountMode) << ", " << IsWritable(ptrStreamBufferCountMode) << "Unable to set StreamBufferCountMode (node retrieval). Aborting..." << endl << endl;
      return -1;
    }
    CEnumEntryPtr ptrStreamBufferCountMode_Manual = ptrStreamBufferCountMode->GetEntryByName("Manual");
    if (!IsAvailable(ptrStreamBufferCountMode_Manual) || !IsReadable(ptrStreamBufferCountMode_Manual))
    {
      cerr << "Unable to set StreamBufferCountMode (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }
    ptrStreamBufferCountMode->SetIntValue(ptrStreamBufferCountMode_Manual->GetValue());

    CIntegerPtr ptrStreamBufferCountManual = nodeMapTLStream.GetNode("StreamBufferCountManual");
    if (!IsAvailable(ptrStreamBufferCountManual) || !IsWritable(ptrStreamBufferCountManual))
    {
      cerr << IsAvailable(ptrStreamBufferCountManual) << "," << IsWritable(ptrStreamBufferCountManual) << "Unable to set StreamBufferCountManual (node retrieval). Aborting..." << endl << endl;
      return -1;
    }
    cerr << "Maximum Buffer Count: " << ptrStreamBufferCountManual->GetMax() << endl;
    ptrStreamBufferCountManual->SetValue(ptrStreamBufferCountManual->GetMax());
    

    cerr << "Camera Stream Buffers Jacked Up..." << endl;

    // Retrieve device serial number for filename
    gcstring deviceSerialNumber("");

    CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
    if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
    {
      deviceSerialNumber = ptrStringSerial->GetValue();

      cerr << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
    }
    cerr << endl;
    
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
int camera_settings_reset(CameraPtr pCam)
{
  // Retrieve GenICam nodemaps
  INodeMap& nodeMap = pCam->GetNodeMap();
  INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
  INodeMap& nodeMapTLStream = pCam->GetTLStreamNodeMap();

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

// use pixel centroid from apriltag sample to set new exposure and gain values
// hard code filters for now
void dvd_DvisEst_image_capture_calculate_exposure_gain(const double centroid)
{
  const double des_centroid = 0.5;
  // Under the current assumption that this only gets called during the ground plane setting
  // set the capture thread to complete once the centroid is around 0.5 
  // (simple bi-modal histogram for now, falsely assumes a balance of BW pixels)
  if(abs(des_centroid - centroid) < 0.01)
  {
    capture_thread_ready = true;
    return;
  }

  double set_exposure_us = sv_exposure_us;
  double set_gain        = sv_gain;

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

  const double max_exposure_us = max_exp_time_s * 1000000.0 / frame_capture_overhead_factor;

  // don't bother checking the limits on the other end
  // just define them for now
  const double max_gain = 100;

  const double centroid_err = des_centroid - centroid;
  const double gain_p       = 0.2;
  const double exposure_p   = 1.0;

  // NOTE: This is not linear at all! We can fix this later if required

  // if things are too dark (centroid_err +ve), try decreasing the exposure
  // if we hit the minimum, try increasing the gain
  // if things are too light, always decrease the exposure time first
  if(set_exposure_us < max_exposure_us - 0.01 || centroid_err < 0)
  {
    set_exposure_us += centroid_err * exposure_p;
    set_exposure_us = min(set_exposure_us, max_exposure_us);
    set_gain -= centroid_err * gain_p * 0.25;
    set_gain = max(1.0, set_gain);
  }
  else
  {
    set_gain += centroid_err * gain_p;
    set_exposure_us -= centroid_err * exposure_p * 0.25;
    set_exposure_us = min(set_exposure_us, max_exposure_us);
  }

  dvd_DvisEst_image_capture_set_exposure_gain(set_exposure_us, set_gain);

  cerr << "Centroid = " << centroid << ", EXP = " << set_exposure_us << ", GAIN = " << set_gain << endl;
}

void dvd_DvisEst_image_capture_set_exposure_gain(const double exposure_us, const double gain)
{
  sv_exposure_us = exposure_us;
  sv_gain = gain;
}

void dvd_DvisEst_image_capture_get_exposure_gain(double * exposure_us, double * gain)
{
  *exposure_us = sv_exposure_us;
  *gain        = sv_gain;
}

static int dvd_DvisEst_image_capture_apply_exposure_gain(CameraPtr pCam)
{
  #if defined(SPINNAKER_ALLOWED)
  // Retrieve GenICam nodemaps
  INodeMap& nodeMap = pCam->GetNodeMap();

  try
  {
    // Set exposure
    CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
    if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
    {
      cerr << "Unable to set exposure time. Aborting..." << endl << endl;
      return -1;
    }

    // Ensure desired exposure time does not exceed the maximum
    const double exposureTimeMax = ptrExposureTime->GetMax();
    const double exposureTimeMin = ptrExposureTime->GetMin();
    double des_exposure = sv_exposure_us;

    if (des_exposure > exposureTimeMax)
    {
      des_exposure = exposureTimeMax;
    }
    if (des_exposure < exposureTimeMin)
    {
      des_exposure = exposureTimeMin;
    }

    ptrExposureTime->SetValue(des_exposure);


    // Set gain
    CFloatPtr ptrGain = nodeMap.GetNode("Gain");
    if (!IsAvailable(ptrGain) || !IsReadable(ptrGain))
    {
      cerr << "Unable to get/set gain. Aborting..." << endl << endl;
      return -1;
    }

    const double gain_max = ptrGain->GetMax();
    const double gain_min = ptrGain->GetMin();
    double des_gain = sv_gain;
    if (des_gain > gain_max)
    {
      des_gain = gain_max;
    }
    if (des_gain < gain_min)
    {
      des_gain = gain_min;
    }

    ptrGain->SetValue(des_gain);    
  }
  catch (Spinnaker::Exception& e)
  {
    cerr << "Error: " << e.what() << endl;
    return -1;
  }
  #endif
}

int dvd_DvisEst_image_capture_thread()
{            
  int result = 0;
  int cam_err = 0;

  // Get camera pointer
  // Retrieve singleton reference to system object
  SystemPtr system = System::GetInstance();

  // Print out current library version
  const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
  cerr << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
     << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
     << endl;

  // Retrieve list of cameras from the system
  CameraList camList = system->GetCameras();
  unsigned int numCameras = camList.GetSize();
  cerr << "Number of cameras detected: " << numCameras << endl << endl;

  CameraPtr pCam;

  try
  {
    // Finish if there are no cameras
    if(numCameras == 0)
    {
      capture_thread_ready = true;
      cerr << "Not enough cameras! Exiting..." << endl;
    }
    else
    {
      // continue camera init
      // just take first camera (why are there 2 plugged in? what?)
      pCam = camList.GetByIndex(0);

      // Initialize camera
      pCam->Init();

      // Configure exposure
      // TODO: Load groundplane auto exposure and gain settings
      cam_err = camera_settings_configure(pCam);
      if (cam_err < 0)
      {
        result = cam_err;
      }

      // Print image format info
      cerr << endl << "Pixel Format Enum: " << pCam->PixelFormat.GetValue() << " == " << PixelFormat_BayerRG8 << endl;

      cerr << "Acquiring images..." << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      cerr << "*************** GOGOGOGO! ******************" << endl << endl;
      // Begin acquiring images
      pCam->BeginAcquisition();
    }

    uint64_t image_count = 0;
    uint64_t timestamp_ns;
    uint64_t init_timestamp_ns = 0;
    uint64_t expgain_adj_timestamp_ns = 0;
    double   last_exposure_us = sv_exposure_us;
    double   last_gain        = sv_gain;
    ImagePtr imagePtr;
    cv::Mat  imageMat;

    while(dvd_DvisEst_get_estimate_stage() < KF_EST_STAGE_PRIME && !capture_thread_ready && result == 0)
    {
      // capture frames       
      // Retrieve next received image and ensure image completion
      imagePtr = pCam->GetNextImage();

      if (imagePtr->IsIncomplete())
      {
        cerr << "Image incomplete with image status " << imagePtr->GetImageStatus() << "..." << endl
           << endl;
      }
      else
      {
        if(init_timestamp_ns == 0)
        {
          init_timestamp_ns = imagePtr->GetTimeStamp();
        }
        timestamp_ns = (imagePtr->GetTimeStamp() - init_timestamp_ns);

        // Convert to OpenCV matrix
        spinnaker_image_to_opencv_mat(imagePtr, &imageMat);
        // push to IO thread so we can keep grabbing frames
        uint32_t frame_id = imagePtr->GetFrameID();
        image_capture_t image_capture = image_capture_t
        (
          imageMat,
          timestamp_ns,
          frame_id
        );

        image_queue_push(&image_capture);

        image_count++;

        if(timestamp_ns > expgain_adj_timestamp_ns + S_TO_NS(0.5))
        {
          expgain_adj_timestamp_ns = timestamp_ns;
          // see if exposure and gain need to be updated
          if(abs(sv_exposure_us - last_exposure_us) > 0.001 || abs(sv_gain - last_gain) > 0.001)
          {
            last_exposure_us = sv_exposure_us;
            last_gain        = sv_gain;
            dvd_DvisEst_image_capture_apply_exposure_gain(pCam);
          }
        }
      }
      // Free up image memory after OpenCV conversion                
      imagePtr->Release();
    }
    if(numCameras > 0)
    {
      // End acquisition
      cerr << "---> Spinnaker End Acquisition" << endl;
      // ************* TODO: Why is this taking so long? What?
      pCam->EndAcquisition();
      cerr << "---> Spinnaker End Acquisition, Complete!" << endl;

      // Reset exposure (why bother?)
      //camera_settings_reset(pCam);

      // wait here until camera references are gone (empty queue)
      cerr << "---> Spinnaker Purge Stale Image Queue" << endl;
      bool ready_to_deint_cam = false;
      while(!ready_to_deint_cam)
      {
        ready_to_deint_cam = image_queue_empty();
        if(!ready_to_deint_cam && (dvd_DvisEst_get_estimate_stage() >= KF_EST_STAGE_PRIME || result != 0))
        {
          // looks like we have some left-over frames to purge before we can de-init the camera
          image_queue_purge(1);
          //cerr << "Purge old Spinnaker Image! Queue Size: " << (int)image_queue_size() << endl;          
        }
        else
        {
          usleep(1000);
        }
      }

      // Deinitialize camera
      cerr << "---> Spinnaker Camera De-Init" << endl;
      pCam->DeInit();

      cerr << "---> Spinnaker Camera De-Init, Complete!" << endl;
    }

    // Clear camera list before releasing system
    // This dumb line is required first
    pCam = nullptr; 
    camList.Clear();
    cerr << "---> Done Spinnaker Camlist Clear!" << endl;

    // Release system
    system->ReleaseInstance();
    cerr << "---> Done Spinnaker release system instance!" << endl;
  }
  catch (Spinnaker::Exception& e)
  {
    cerr << "Error: " << e.what() << endl;
    result = -1;
  }
  catch (...)
  {
    cerr << "Unexpected Error in IMAGE CAPTURE!" << endl;
  }

  return result;
}

double dvd_DvisEst_image_capture_get_fps()
{
  return sv_image_capture_frame_rate;
}

bool dvd_DvisEst_image_capture_thread_ready()
{
  return capture_thread_ready;
}

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
    
  capture_thread = std::thread(dvd_DvisEst_image_capture_thread);

  /*// jack up thread priority for capture (does this actually help?)
  #ifdef linux
  sched_param sch;
  int policy; 
  pthread_getschedparam(capture_thread.native_handle(), &policy, &sch);
  sch.sched_priority = 90;
  if (pthread_setschedparam(capture_thread.native_handle(), SCHED_FIFO, &sch)) 
  {
      cerr << "Failed to setschedparam: " << std::strerror(errno) << '\n';
  }
  #endif
  #ifdef _WIN32
      // figure out how to do thread priority on Windows...
  #endif*/

  #endif
}

// Stop collecting frames
void dvd_DvisEst_image_capture_stop(const bool camera_src)
{
  #if defined(SPINNAKER_ALLOWED)
  if(camera_src)
  {


    // purge frames here perhaps? Only matters if this is included as a library

    capture_thread.join();

  }
  #endif
}

void dvd_DvisEst_image_capture_load_test_queue_threaded(const cv::String imgdir_src, const double dt)
{
  //capture_thread = std::thread(dvd_DvisEst_image_capture_load_test_queue, imgdir_src, dt);
}

// load test images into the capture queue and return
bool dvd_DvisEst_image_capture_load_test_queue(const cv::String imgdir_src, const double dt)
{
  sv_image_capture_frame_rate = 1.0 / max(CLOSE_TO_ZERO, dt);

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
    //cerr << "Loading " << img_filenames[i] << " into image queue..." << endl;
    
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

  capture_thread_ready = true;
  cerr << "Capture Test Thread Complete." << endl;

  return true;
}

uint32_t dvd_DvisEst_image_capture_get_image_capture_queue_size()
{
  return image_queue_size();
}

// Return the next captured image from the front of the queue
// TODO: THERE IS SOMETHING WRONG WITH THIS IMPLEMENTATION WHERE INITIAL APRILTAG DETECTIONS ARE LOST!
// FIX IT!
#define MAX_FRAME_SKIP_COUNT (10)
bool dvd_DvisEst_image_capture_get_next_image_capture(image_capture_t * image_capture, uint16_t * skipped_frames, std::atomic<uint8_t> * at_thread_mode, uint8_t thread_id)
{
  bool got_frame = true;
  static int32_t scerr_index = 0;

  // If any tags have been detected, force subsequent threads out of scerr mode
  if(dvd_DvisEst_estimate_get_tags_detected() && *at_thread_mode == AT_DETECTION_THREAD_MODE_SCOUT)
  {
    cerr << "Force Thread #" << (int)thread_id << " into MEAS mode!" << endl; 
  }

  // If we're using test images, might want to keep processing until the end of the queue....
  // (fake out meas mode for the last 100 entries for now)
  if(*at_thread_mode == AT_DETECTION_THREAD_MODE_SCOUT)
  {
    // TODO: clean this up
    // we'll mutex this operation since we need the scerr index updated before the next thread tries to scerr ahead
    sv_image_scerr_mutex.lock();

    const bool wrap_things_up_for_test_mode = dvd_DvisEst_image_capture_thread_ready();
    const int32_t queue_size = image_queue_size();

    // This is the block our scerr threads try to stay within
    // If the queue has less than AT_THREAD_COUNT * MAX_FRAME_SKIP_COUNT members, start divying up what's left
    const int32_t scerr_block_max = AT_THREAD_COUNT * MAX_FRAME_SKIP_COUNT; 
    const int32_t scerr_size = min(queue_size, scerr_block_max);
    int32_t scerr_block = ceil(scerr_size / AT_THREAD_COUNT); // always skip at least 1 frame if one is present

    scerr_index += scerr_block;

    // Note how purging will only begin once the scerr index surpasses 'scerr_block_max'
    int32_t purge_count = max(scerr_index - (AT_THREAD_COUNT * MAX_FRAME_SKIP_COUNT), 0); // fix this

    // purge extra frames, scerr index will move along from the front as a result
    *skipped_frames = image_queue_purge(purge_count);
    scerr_index -= purge_count; // Now that the queue has been shortened, move the scerr index as appropriate

    // pull the scerr frame, but don't pop it from the queue
    got_frame = image_queue_pull(image_capture, scerr_index, 0);

    // test check:
    //image_capture_t ic_front;
    //image_queue_pull(&ic_front, 0, 0);

    //cerr << "Scerr index is now " << scerr_index << " for a queue size of " << queue_size - purge_count <<
    //", FID for scerr thread is " << (int)image_capture->frame_id << ", and front is " << (int)ic_front.frame_id << endl;

    static uint32_t image_capture_last_print_time_ms = 0;
    if(NS_TO_MS(image_capture->timestamp_ns) > image_capture_last_print_time_ms + 1000)
    {
      image_capture_last_print_time_ms = NS_TO_MS(image_capture->timestamp_ns);
      cerr << "Image Queue Size: " << (int)queue_size << endl;
    }

    // test mode only to empty end of queue
    if(queue_size <= scerr_block_max && wrap_things_up_for_test_mode && purge_count == 0 && dvd_DvisEst_get_estimate_stage() < KF_EST_STAGE_PRIME)
    {
      cerr << "WRAP PURGE!" << dvd_DvisEst_get_estimate_stage() << endl;
      *skipped_frames += image_queue_purge(1);
      sv_image_scerr_mutex.unlock();
      return false;
    }
    
    sv_image_scerr_mutex.unlock();
  }
  else
  {
    *skipped_frames = 0;
    // reset scerr index
    scerr_index = 0;
    got_frame = image_queue_pull(image_capture, 0, 1);
    //cerr << "Meas read at FID " << (int)image_capture->frame_id << endl;
  }  

  return got_frame;
}

// Return the next captured image from the front of the queue
bool dvd_DvisEst_image_capture_image_capture_queue_empty(void)
{
  return image_queue_empty();
}
