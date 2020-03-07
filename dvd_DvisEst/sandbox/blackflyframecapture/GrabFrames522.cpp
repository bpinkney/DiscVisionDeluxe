// Read from

#include <unistd.h>
#include <chrono>
#define _BSD_SOURCE
#include <sys/time.h>
#include <stdio.h>
#include <string.h>

#include "Spinnaker.h"
#include "SpinGenApi/SpinnakerGenApi.h"
#include <iostream>
#include <sstream>

// threading stuff
#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <atomic>

// Include the standard OpenCV headers
#include <opencv2/opencv.hpp>

using namespace Spinnaker;
using namespace Spinnaker::GenApi;
using namespace Spinnaker::GenICam;
using namespace std;
using namespace cv;

// declare global queue for mutex access

std::queue<cv::Mat> imageMatQueue;
std::queue<uint64_t> timestampnsQueue;
std::queue<uint64_t> frameIDQueue;
std::mutex imageMatMutex;
std::atomic<bool> ai_thread_ready (false);


void image_queue_push(cv::Mat imageMat, uint64_t timestamp_ns, uint64_t frame_id)
{
  // mutex on
  imageMatMutex.lock();
  
  imageMatQueue.push(imageMat);
  timestampnsQueue.push(timestamp_ns);
  frameIDQueue.push(frame_id);

  // mutex off
  imageMatMutex.unlock();
}

bool image_queue_pull(cv::Mat * imageMat, uint64_t * timestamp_ns, uint64_t * frame_id)
{   
  // mutex on
  imageMatMutex.lock();
  if(imageMatQueue.empty())
  {
    // mutex off
    imageMatMutex.unlock();
    *imageMat = cv::Mat();
    *timestamp_ns = 0;
    *frame_id     = 0;
    return false;
  }
  else
  {    
    *imageMat     = imageMatQueue.front();
    *timestamp_ns = timestampnsQueue.front();
    *frame_id     = frameIDQueue.front();
    
    imageMatQueue.pop();
    timestampnsQueue.pop();
    frameIDQueue.pop();
    // mutex off
    imageMatMutex.unlock();
    return true;
  }

}

// Get time stamp in milliseconds.
uint64_t millis()
{
  uint64_t ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::
          now().time_since_epoch()).count();
  return ms; 
}

// Get time stamp in microseconds.
uint64_t micros()
{
  uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
          now().time_since_epoch()).count();
  return us; 
}

// Get time stamp in nanoseconds.
uint64_t nanos()
{
  uint64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::high_resolution_clock::
          now().time_since_epoch()).count();
  return ns; 
}

double get_uptime_ms()
{    
  static uint64_t base_ns = 0;
  uint64_t time_ns = nanos();
  if(base_ns == 0)
  {
    base_ns = time_ns;
    return 0;
  }

  return ((double)(time_ns - base_ns)*0.000001);
}

// OpenCV stuff
cv::Mat spinnakerWrapperToCvMat(ImagePtr imagePtr)
{
  try
  {
    //ImagePtr convertedImagePtr = imagePtr->Convert(PixelFormat_BGR8, NEAREST_NEIGHBOR);
    ImagePtr convertedImagePtr = imagePtr;//->Convert(PixelFormat_Mono8, HQ_LINEAR);

    const auto XPadding = convertedImagePtr->GetXPadding();
    const auto YPadding = convertedImagePtr->GetYPadding();
    const auto rowsize = convertedImagePtr->GetWidth();
    const auto colsize = convertedImagePtr->GetHeight();

    cv::Mat openCVOutputImage  = 
      cv::Mat
      (
        (int)(colsize + YPadding), 
        (int)(rowsize + XPadding), 
        CV_8UC1, 
        convertedImagePtr->GetData(),
        convertedImagePtr->GetStride()
      );

    // debug
    bool show_me = false;
    if(show_me)
    {
      namedWindow( "Display window", WINDOW_AUTOSIZE );// Create a window for display.
      imshow( "Display window",openCVOutputImage );    // Show our image inside it.
      waitKey(0);                                      // Wait for a keystroke in the window
    }    

    return openCVOutputImage;
  }
  catch (const std::exception& e)
  {
    cout << endl << endl << "*** Could not convert Spinnaker Image Pointer to OpenCV MAT! ***" << endl << endl;
    return cv::Mat();
  }
}


// Disbale everything preventing us from achieving the full frame rate
int ConfigureExposureGainWhiteBalanceADC(INodeMap& nodeMap)
{
  int result = 0;

  cout << endl << endl << "*** CONFIGURING EXPOSURE, GAIN, WHITE BALANCE, and ADC bitness ***" << endl << endl;

  try
  {
    // Set ADC bitness to 8-bits
    CEnumerationPtr ptrAdcBitDepth = nodeMap.GetNode("AdcBitDepth");
    if (!IsAvailable(ptrAdcBitDepth) || !IsWritable(ptrAdcBitDepth))
    {
      cout << "Unable to set adc bit depth to 8-bits (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrAdcBitDepthBit8 = ptrAdcBitDepth->GetEntryByName("Bit8");
    if (!IsAvailable(ptrAdcBitDepthBit8) && !IsReadable(ptrAdcBitDepthBit8))
    {
      cout << "Unable to set adc bit depth to 8-bits (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrAdcBitDepth->SetIntValue(ptrAdcBitDepthBit8->GetValue());
    cout << "ADC bitness set to 8-bits..." << endl;

    // Turn off automatic exposure mode
    CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
    {
      cout << "Unable to disable automatic exposure (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrExposureAutoOff = ptrExposureAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrExposureAutoOff) || !IsReadable(ptrExposureAutoOff))
    {
      cout << "Unable to disable automatic exposure (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrExposureAuto->SetIntValue(ptrExposureAutoOff->GetValue());
    cout << "Automatic exposure disabled..." << endl;


    if(1)
    {
      // We know that the output framerate of the camera is intrinsically limited 
      // by the exposure setting
      // Try to calculate this explicitly
      double des_frame_rate = 522.0;
      double max_exp_time_s = 1.0 / des_frame_rate;

      // Since the frame capture is composed of more than just the shutter speeed
      // add a de-rate factor here to account for overhead
      // e.g. 1.1 assumes that 1/10 of the exposure time is required for frame capture and admin
      // tested experimentally at 522fps to be between 6-7%
      double frame_capture_overhead_factor = 1.07 * 1.0;
      double des_exposure_us = max_exp_time_s * 1000000.0 / frame_capture_overhead_factor;

      CFloatPtr ptrExposureTime = nodeMap.GetNode("ExposureTime");
      if (!IsAvailable(ptrExposureTime) || !IsWritable(ptrExposureTime))
      {
        cout << "Unable to set exposure time. Aborting..." << endl << endl;
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
    }

    // How can we verify that the gain is actually changing here to make up for the low fixed exposure?
    // Perhaps read the gain values out and wait for convergence? (likely)
    //cout << "Waiting for camera to adapt to new exposure and bitness..." << endl;
    //usleep(1000000);

    // We can now proceed with disabling the remaining frame-rate inhibiting features
    // Turn off automatic gain
    CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
    if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
    {
      cout << "Unable to disable automatic gain (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrGainAutoOff = ptrGainAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrGainAutoOff) && !IsReadable(ptrGainAutoOff))
    {
      cout << "Unable to disable automatic gain (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrGainAuto->SetIntValue(ptrGainAutoOff->GetValue());
    cout << "Automatic gain disabled..." << endl;

    // set gain
    CFloatPtr ptrGain = nodeMap.GetNode("Gain");
    if (!IsAvailable(ptrGain) || !IsReadable(ptrGain))
    {
      cout << "Unable to get/set gain. Aborting..." << endl << endl;
      return -1;
    }
    if(1)
    {
      double des_gain = 10.0;
      
      const double gainMax = ptrGain->GetMax();
      const double gainMin = ptrGain->GetMin();

      if (des_gain > gainMax)
      {
        des_gain = gainMax;
      }
      if (des_gain < gainMin)
      {
        des_gain = gainMin;
      }

      ptrGain->SetValue(des_gain);
    }

    // Check gain value        
    float gain = static_cast<float>(ptrGain->GetValue());
    cout << "Gain: " << gain << endl;

    // Turn off automatic white balance
    CEnumerationPtr ptrBalanceWhiteAuto = nodeMap.GetNode("BalanceWhiteAuto");
    if (!IsAvailable(ptrBalanceWhiteAuto) || !IsWritable(ptrBalanceWhiteAuto))
    {
      cout << "Unable to disable automatic white balance (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrBalanceWhiteAutoOff = ptrBalanceWhiteAuto->GetEntryByName("Off");
    if (!IsAvailable(ptrBalanceWhiteAutoOff) && !IsReadable(ptrBalanceWhiteAutoOff))
    {
      cout << "Unable to disable automatic white balance (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrBalanceWhiteAuto->SetIntValue(ptrBalanceWhiteAutoOff->GetValue());
    cout << "Automatic white balance disabled..." << endl;

    // Edit: nevermind! this limits the framerate to 290 due to the onboard ISP! D'oh!
    /*if(0)
    {
      CEnumerationPtr ptrPixelFormat = nodeMap.GetNode("PixelFormat");
      if (IsAvailable(ptrPixelFormat) && IsWritable(ptrPixelFormat))
      {
        // Retrieve the desired entry node from the enumeration node
        CEnumEntryPtr ptrPixelFormatMono8 = ptrPixelFormat->GetEntryByName("Mono8");
        if (IsAvailable(ptrPixelFormatMono8) && IsReadable(ptrPixelFormatMono8))
        {
          // Retrieve the integer value from the entry node
          int64_t pixelFormatMono8 = ptrPixelFormatMono8->GetValue();
          // Set integer as new value for enumeration node
          ptrPixelFormat->SetIntValue(pixelFormatMono8);
          cout << "Pixel format set to " << ptrPixelFormat->GetCurrentEntry()->GetSymbolic() << "..." << endl;
        }
        else
        {
          cout << "Pixel format mono 8 not available..." << endl;
        }
      }
      else
      {
        cout << "Pixel format not available..." << endl;
      }
    }*/
    
    // Check possible frame rate now that we have disabled these features
    CFloatPtr ptrAcquisitionResultingFrameRate = nodeMap.GetNode("AcquisitionResultingFrameRate");
    if (!IsAvailable(ptrAcquisitionResultingFrameRate) || !IsReadable(ptrAcquisitionResultingFrameRate))
    {
      cout << "Unable to retrieve frame rate. Aborting..." << endl << endl;
      return -1;
    }

    float acquisitionResultingFrameRate = static_cast<float>(ptrAcquisitionResultingFrameRate->GetValue());
    cout << "Max Acquisition Frame Rate: " << acquisitionResultingFrameRate << endl;
    
  }
  catch (Spinnaker::Exception& e)
  {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }

  return result;
}

// This function returns the camera to its default state by re-enabling automatic
// exposure.
int ResetExposureGainWhiteBalanceADC(INodeMap& nodeMap)
{
  int result = 0;

  try
  {
    // Turn automatic exposure back on
    CEnumerationPtr ptrExposureAuto = nodeMap.GetNode("ExposureAuto");
    if (!IsAvailable(ptrExposureAuto) || !IsWritable(ptrExposureAuto))
    {
      cout << "Unable to enable automatic exposure (node retrieval). Non-fatal error..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrExposureAutoContinuous = ptrExposureAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrExposureAutoContinuous) || !IsReadable(ptrExposureAutoContinuous))
    {
      cout << "Unable to enable automatic exposure (enum entry retrieval). Non-fatal error..." << endl << endl;
      return -1;
    }

    ptrExposureAuto->SetIntValue(ptrExposureAutoContinuous->GetValue());
    cout << "Automatic exposure enabled..." << endl;


    // Turn automatic gain back on
    CEnumerationPtr ptrGainAuto = nodeMap.GetNode("GainAuto");
    if (!IsAvailable(ptrGainAuto) || !IsWritable(ptrGainAuto))
    {
      cout << "Unable to enable automatic gain (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrGainAutoContinuous = ptrGainAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrGainAutoContinuous) && !IsReadable(ptrGainAutoContinuous))
    {
      cout << "Unable to enable automatic gain (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrGainAuto->SetIntValue(ptrGainAutoContinuous->GetValue());
    cout << "Automatic gain enabled..." << endl;


    // Turn on automatic white balance
    CEnumerationPtr ptrBalanceWhiteAuto = nodeMap.GetNode("BalanceWhiteAuto");
    if (!IsAvailable(ptrBalanceWhiteAuto) || !IsWritable(ptrBalanceWhiteAuto))
    {
      cout << "Unable to disable automatic white balance (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrBalanceWhiteAutoContinuous = ptrBalanceWhiteAuto->GetEntryByName("Continuous");
    if (!IsAvailable(ptrBalanceWhiteAutoContinuous) && !IsReadable(ptrBalanceWhiteAutoContinuous))
    {
      cout << "Unable to disable automatic white balance (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrBalanceWhiteAuto->SetIntValue(ptrBalanceWhiteAutoContinuous->GetValue());
    cout << "Automatic white balance enabled..." << endl;


    // Set ADC bitness to 10-bits
    CEnumerationPtr ptrAdcBitDepth = nodeMap.GetNode("AdcBitDepth");
    if (!IsAvailable(ptrAdcBitDepth) || !IsWritable(ptrAdcBitDepth))
    {
      cout << "Unable to set adc bit depth to 10-bits (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrAdcBitDepthBit10 = ptrAdcBitDepth->GetEntryByName("Bit10");
    if (!IsAvailable(ptrAdcBitDepthBit10) && !IsReadable(ptrAdcBitDepthBit10))
    {
      cout << "Unable to set adc bit depth to 10-bits (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }

    ptrAdcBitDepth->SetIntValue(ptrAdcBitDepthBit10->GetValue());
    cout << "ADC bitness set to 10-bits..." << endl;
  }
  catch (Spinnaker::Exception& e)
  {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }

  return result;
}

// This function prints the device information of the camera from the transport
// layer; please see NodeMapInfo example for more in-depth comments on printing
// device information from the nodemap.
int PrintDeviceInfo(INodeMap& nodeMap)
{
  int result = 0;

  cout << endl << "*** DEVICE INFORMATION ***" << endl << endl;

  try
  {
    FeatureList_t features;
    CCategoryPtr category = nodeMap.GetNode("DeviceInformation");
    if (IsAvailable(category) && IsReadable(category))
    {
      category->GetFeatures(features);

      FeatureList_t::const_iterator it;
      for (it = features.begin(); it != features.end(); ++it)
      {
        CNodePtr pfeatureNode = *it;
        cout << pfeatureNode->GetName() << " : ";
        CValuePtr pValue = (CValuePtr)pfeatureNode;
        cout << (IsReadable(pValue) ? pValue->ToString() : "Node not readable");
        cout << endl;
      }
    }
    else
    {
      cout << "Device control information not available." << endl;
    }
  }
  catch (Spinnaker::Exception& e)
  {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }

  return result;
}

// This function acquires and saves image_num images from a device; please see
// Acquisition example for more in-depth comments on acquiring images.
int AcquireImages(CameraPtr pCam, uint k_numImages, std::promise<int> && p)
{            
  int result = 0;

  
  // Retrieve GenICam nodemap
  INodeMap& nodeMap = pCam->GetNodeMap();
  INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();
  INodeMap& nodeMapTLStream = pCam->GetTLStreamNodeMap();

  cout << endl << "*** IMAGE ACQUISITION ***" << endl << endl;

  uint32_t imageCnt = 0;

  try
  {
    // Set acquisition mode to continuous
    CEnumerationPtr ptrAcquisitionMode = nodeMap.GetNode("AcquisitionMode");
    if (!IsAvailable(ptrAcquisitionMode) || !IsWritable(ptrAcquisitionMode))
    {
      cout << "Unable to set acquisition mode to continuous (node retrieval). Aborting..." << endl << endl;
      return -1;
    }

    CEnumEntryPtr ptrAcquisitionModeContinuous = ptrAcquisitionMode->GetEntryByName("Continuous");
    if (!IsAvailable(ptrAcquisitionModeContinuous) || !IsReadable(ptrAcquisitionModeContinuous))
    {
      cout << "Unable to set acquisition mode to continuous (entry 'continuous' retrieval). Aborting..." << endl
         << endl;
      return -1;
    }

    int64_t acquisitionModeContinuous = ptrAcquisitionModeContinuous->GetValue();

    ptrAcquisitionMode->SetIntValue(acquisitionModeContinuous);

    cout << "Acquisition mode set to continuous..." << endl;

    // try to avoid skipping frames with this hack
    // maybe I need to query the buffers constantly? ehhh
    // Sounds like we'll just lose frames every time there isn't enough CPU...
    CEnumerationPtr ptrStreamBufferCountMode = nodeMapTLStream.GetNode("StreamBufferCountMode");
    if (!IsAvailable(ptrStreamBufferCountMode) || !IsWritable(ptrStreamBufferCountMode))
    {
      cout << IsAvailable(ptrStreamBufferCountMode) << ", " << IsWritable(ptrStreamBufferCountMode) << "Unable to set StreamBufferCountMode (node retrieval). Aborting..." << endl << endl;
      return -1;
    }
    CEnumEntryPtr ptrStreamBufferCountMode_Manual = ptrStreamBufferCountMode->GetEntryByName("Manual");
    if (!IsAvailable(ptrStreamBufferCountMode_Manual) || !IsReadable(ptrStreamBufferCountMode_Manual))
    {
      cout << "Unable to set StreamBufferCountMode (enum entry retrieval). Aborting..." << endl << endl;
      return -1;
    }
    ptrStreamBufferCountMode->SetIntValue(ptrStreamBufferCountMode_Manual->GetValue());

    CIntegerPtr ptrStreamBufferCountManual = nodeMapTLStream.GetNode("StreamBufferCountManual");
    if (!IsAvailable(ptrStreamBufferCountManual) || !IsWritable(ptrStreamBufferCountManual))
    {
      cout << IsAvailable(ptrStreamBufferCountManual) << "," << IsWritable(ptrStreamBufferCountManual) << "Unable to set StreamBufferCountManual (node retrieval). Aborting..." << endl << endl;
      return -1;
    }
    cout << "Maximum Buffer Count: " << ptrStreamBufferCountManual->GetMax() << endl;
    ptrStreamBufferCountManual->SetValue(ptrStreamBufferCountManual->GetMax());
    

    cout << "Camera Stream Buffers Jacked Up..." << endl;

    // Retrieve device serial number for filename
    gcstring deviceSerialNumber("");

    CStringPtr ptrStringSerial = nodeMapTLDevice.GetNode("DeviceSerialNumber");
    if (IsAvailable(ptrStringSerial) && IsReadable(ptrStringSerial))
    {
      deviceSerialNumber = ptrStringSerial->GetValue();

      cout << "Device serial number retrieved as " << deviceSerialNumber << "..." << endl;
    }
    cout << endl;

    // Retrieve images and pipe them into threadsafe queue       
    ImagePtr imagePtr;
    ImagePtr convertedImagePtr;
    cv::Mat  imageMat;
    uint64_t timestamp_ns;

    // Begin acquiring images
    pCam->BeginAcquisition();

    cout << "Acquiring images..." << endl;
    cout << "*************** GOGOGOGO! ******************" << endl << endl;

    uint64_t init_timestamp_ns = 0;
    for (imageCnt = 0; imageCnt < k_numImages; imageCnt++)
    {
      try
      {
        // Retrieve next received image and ensure image completion
        imagePtr = pCam->GetNextImage();

        if (imagePtr->IsIncomplete())
        {
          cout << "Image incomplete with image status " << imagePtr->GetImageStatus() << "..." << endl
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
          imageMat = spinnakerWrapperToCvMat(imagePtr);
          // push to IO thread so we can keep grabbing frames
          image_queue_push(imageMat, timestamp_ns, imagePtr->GetFrameID());
        }

        // Free up image memory after OpenCV conversion                
        imagePtr->Release();
      }
      catch (Spinnaker::Exception& e)
      {
        cout << "Error: " << e.what() << endl;
        result = -1;
      }
      catch (...)
      {
        cout << "Unexpected Error in Acquire Images inner loop!" << endl;
      }
    }

    // End acquisition
    pCam->EndAcquisition();
  }
  catch (Spinnaker::Exception& e)
  {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
  catch (...) 
  {
    cout << "Unexpected Exception in Acquire Images!" << endl;
  }

  cout << "*************** STOP! ******************" << endl << endl;

  p.set_value(result);
  ai_thread_ready = true;
  return result;
}

int ProcessImages(void)
{    
  // cycle through buffered images
  
  uint64_t imageCnt           = 0;
  uint64_t timestamp_ns       = 0;
  uint64_t last_timestamp_ns  = 0;
  uint64_t frame_id           = 0;
  uint64_t last_frame_id      = 0;
  cv::Mat imageBayerMat   = cv::Mat();
  cv::Mat imageMat        = cv::Mat();
  bool empty_queue = false;

  if(0)
  {
    // due to CPU issues, just wait here and log all images to RAM before writing to disk
    while(!ai_thread_ready)
    {
      usleep(10000);
    }
  }

  // Continue until our capture thread is done, and our queue is empty
  while(!ai_thread_ready || !empty_queue)
  {
    // Pull the raw image out of the queue
    if(!image_queue_pull(&imageBayerMat, &timestamp_ns, &frame_id))
    {
      //cout << endl << "Empty Queue!" << endl;
      empty_queue = true;
      continue;
    }
    empty_queue = false;

    // Colour option
    cvtColor(imageBayerMat, imageMat, cv::COLOR_BayerRG2RGB); // starts as PixelFormat_BayerRG8

    // Greyscale option
    //cvtColor(imageBayerMat, imageMat, cv::COLOR_BayerRG2GRAY); // starts as PixelFormat_BayerRG8

    uint64_t frame_diff = (frame_id - last_frame_id);

    bool print_writes = false;
    if(print_writes)
    {
      // print info
      cv::Size s = imageMat.size();

      cout 
      << imageCnt << " : " 
      << (double)(timestamp_ns - last_timestamp_ns)*0.000001 
      << " ms : Grabbed images " 
      << imageCnt << ", width = " << s.width
      << ", height = " << s.height << endl;
    }
    
    if(frame_diff > 1)
    {
      cout << "Frames lost! " << imageCnt << endl;
    }

    last_timestamp_ns = timestamp_ns;
    last_frame_id     = frame_id;

    // Create a unique filename
    ostringstream filename;

    filename << "imgs/" << "BFS-";
    filename << imageCnt << ".jpg";
    // Save image
    cv::imwrite(filename.str(), imageMat);

    imageCnt++;
      
  }
  cout << "Grabbed images " << imageCnt << endl;
  
  return 0;
}

// This function acts as the body of the example; please see NodeMapInfo example
// for more in-depth comments on setting up cameras.
int RunSingleCamera(CameraPtr pCam, uint32_t k_numImages)
{
  int result = 0;
  int err = 0;
  try
  {
    // Retrieve TL device nodemap and print device information
    INodeMap& nodeMapTLDevice = pCam->GetTLDeviceNodeMap();

    result = PrintDeviceInfo(nodeMapTLDevice);

    // Initialize camera
    pCam->Init();

    // Retrieve GenICam nodemap
    INodeMap& nodeMap = pCam->GetNodeMap();

    // Configure exposure
    err = ConfigureExposureGainWhiteBalanceADC(nodeMap);
    if (err < 0)
    {
      return err;
    }

    // Print image format info
    cout << endl << "Pixel Format Enum: " << pCam->PixelFormat.GetValue() << " == " << PixelFormat_BayerRG8 << endl;

    // Acquire images in one thread
    // standard non-threaded method for reference
    // result = result | AcquireImages(pCam);
    std::promise<int> p;
    auto f = p.get_future();
    
    std::thread aiThread(AcquireImages, pCam, k_numImages, std::move(p));

    // jack up thread priority for capture (does this actually help?)
    #ifdef linux
    sched_param sch;
    int policy; 
    pthread_getschedparam(aiThread.native_handle(), &policy, &sch);
    sch.sched_priority = 90;
    if (pthread_setschedparam(aiThread.native_handle(), SCHED_FIFO, &sch)) 
    {
        cout << "Failed to setschedparam: " << std::strerror(errno) << '\n';
    }
    #endif
    #ifdef _WIN32
        // figure out how to do thread priority on Windows...
    #endif

    // Save/process them in another (main thread with an atomic watch)
    ProcessImages();

    // get result of acquisition thread
    aiThread.join();
    int i = f.get();
    result = result | i;

    // Reset exposure
    result = result | ResetExposureGainWhiteBalanceADC(nodeMap);

    // Deinitialize camera
    pCam->DeInit();
  }
  catch (Spinnaker::Exception& e)
  {
    cout << "Error: " << e.what() << endl;
    result = -1;
  }
  catch (...)
  {
    cout << "Unexpected Error in RunSingleCamera!" << endl;
  }

  return result;
}

// Example entry point; please see Enumeration example for additional
// comments on the steps in this function.
int main(int /*argc*/, char** /*argv*/)
{
  // Since this application saves images in the current folder
  // we must ensure that we have permission to write to this folder.
  // If we do not have permission, fail right away.
  FILE* tempFile = fopen("test.txt", "w+");
  if (tempFile == nullptr)
  {
    cout << "Failed to create file in current folder.  Please check "
        "permissions."
       << endl;
    cout << "Press Enter to exit..." << endl;
    getchar();
    return -1;
  }
  fclose(tempFile);
  remove("test.txt");

  int result = 0;

  // Print application build information
  cout << "Application build date: " << __DATE__ << " " << __TIME__ << endl << endl;

  // Retrieve singleton reference to system object
  SystemPtr system = System::GetInstance();

  // Print out current library version
  const LibraryVersion spinnakerLibraryVersion = system->GetLibraryVersion();
  cout << "Spinnaker library version: " << spinnakerLibraryVersion.major << "." << spinnakerLibraryVersion.minor
     << "." << spinnakerLibraryVersion.type << "." << spinnakerLibraryVersion.build << endl
     << endl;

  // Retrieve list of cameras from the system
  CameraList camList = system->GetCameras();

  unsigned int numCameras = camList.GetSize();

  cout << "Number of cameras detected: " << numCameras << endl << endl;

  // Finish if there are no cameras
  if (numCameras == 0)
  {
    // Clear camera list before releasing system
    camList.Clear();

    // Release system
    system->ReleaseInstance();

    cout << "Not enough cameras!" << endl;
    cout << "Done! Press Enter to exit..." << endl;
    getchar();

    return -1;
  }

  // Run example on first camera
  unsigned int i = 0;
  cout << endl << "Running example for camera " << i << "..." << endl;

  result = result | RunSingleCamera(camList.GetByIndex(i), 10000);

  cout << "Camera " << i << " example complete..." << endl << endl;

  // Clear camera list before releasing system
  camList.Clear();

  // Release system
  system->ReleaseInstance();

  cout << endl << "Done!" << endl;

  return result;
}