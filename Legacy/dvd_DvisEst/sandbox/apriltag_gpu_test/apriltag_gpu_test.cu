#include<iostream>
#include<limits>
#include <chrono>
#include <unordered_map>
#include <string>

// CUDA runtime
#include <cuda_runtime.h>

// helper functions and utilities to work with CUDA
#include <helper_functions.h>
#include <helper_cuda.h>

// GPU version
//#include "nvAprilTags.h"
//#include "apriltag_gpu.h"

// CPU version
#include "apriltag.h"
#include "tag36h11.h"

// opencv stuff
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

using namespace std;
using namespace cv;

static nvAprilTagsHandle hApriltags = nullptr;

// camera matrix is 3x3:
//  fx   0    cx
//  0    fy   cy
//  0    0    1 

// [ 1.5283289799775675e+03, 0., 3.5971937738154656e+02, 
//   0., 1.4032086558666635e+03, 2.6986410577371180e+02, 
//   0., 0., 1. ]

static float sFx = 1.5283289799775675e+03;
static float sFy = 1.4032086558666635e+03;
static float sCx = 3.5971937738154656e+02;
static float sCy = 2.6986410577371180e+02;
static float sTagDimension = 0.20;
static uint8_t* buffer_cpu = nullptr;
static uint8_t* buffer_gpu = nullptr;

nvAprilTagsImageInput_st sNvData;


void allocate_uint8_buffer(uint8_t *tempPoint, const int width, const int height, const int color_comp_per_pixel)
{
  //uint8_t *tempPoint;// = nullptr;
  const uint32_t gps_malloc_size = width*height*color_comp_per_pixel*sizeof(uint8_t);

  // Why does this keep segfaulting?
  checkCudaErrors(cudaMalloc((void **)&tempPoint, gps_malloc_size));
  //return tempPoint;
}

int release_gpu_buffer (void * buffer)
{
  checkCudaErrors(cudaFree(buffer));
  return 0;
}


int upload_data_to_GPU (void * dst, void * src, int width, int height, int color_comp_per_pixel)
{
  int size = width * height * color_comp_per_pixel;    
  checkCudaErrors(cudaMemcpy(dst,src,size,cudaMemcpyHostToDevice));
  return 0;
}

int download_data_from_GPU (void * dst, void * src, int width, int height, int color_comp_per_pixel)
{
  int size = width * height * color_comp_per_pixel;        
  checkCudaErrors(cudaMemcpy(dst,src,size,cudaMemcpyDeviceToHost));
  return 0;
}

void upload_data_to_uchar4_gpu(uint8_t *data, int pitch, int width, int height) 
{

  uchar4* buf = (uchar4 *)buffer_cpu;
  int i = 0, j = 0;
  if(IMAGE_C == 4) {
    while(i < pitch * height * IMAGE_C) {
      buf[j].x = data[i++];
      buf[j].y = data[i++];
      buf[j].z = data[i++];
      buf[j].w = data[i++];
      j++;
    }
  } else {
    while(i < pitch * height * IMAGE_C) {
      buf[j].x = data[i];
      buf[j].y = data[i];
      buf[j].z = data[i];
      buf[j].w = 0xFF; //data[i];
      j++; i++;
    }
  }
  upload_data_to_GPU(buffer_gpu, buffer_cpu, pitch, height, sizeof(uchar4));
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
int nv_init_apriltag_detector() 
{
  if(hApriltags) return -1;
  buffer_cpu = (uint8_t*)malloc(sizeof(uchar4) * IMAGE_W * IMAGE_H);
  //buffer_gpu = (uint8_t*)malloc(sizeof(uchar4) * IMAGE_W * IMAGE_H);
  allocate_uint8_buffer(buffer_gpu, IMAGE_W, IMAGE_H, sizeof(uchar4));
  return 0;
}

int nv_create_apriltag_detector(int width, int height) 
{
  if(hApriltags) nv_destroy_apriltag_detector();

  nvAprilTagsCameraIntrinsics_t cam = {sFx, sFy, sCx, sCy };
  sNvData.dev_ptr = (uchar4*)buffer_gpu;
  sNvData.pitch   = width * sizeof(uchar4);
  sNvData.width   = width;
  sNvData.height  = height;
  return nvCreateAprilTagsDetector(&hApriltags, width, height, NVAT_TAG36H11, &cam, sTagDimension);
}

static unordered_map<int, int> m;

int nv_apriltag_detector_detect (uint8_t *data, int pitch, int width, int height) 
{
  nv_create_apriltag_detector(width, height);

  if(!hApriltags) return -1;
  
  upload_data_to_uchar4_gpu(data, pitch, width, height);

  uint32_t num_tags = 0;
  std::vector<nvAprilTagsID_t> tags;

  auto prev = std::chrono::high_resolution_clock::now();

  sNvData.pitch   = pitch * sizeof(uchar4);
  sNvData.width   = width;
  sNvData.height  = height;

  // for(int i = 0; i < 1000; i++) 
  {
    tags.clear();
    tags.resize(20);
    nvAprilTagsDetect(hApriltags, &(sNvData), tags.data(), &num_tags, tags.size(), nullptr);
  }
  auto now = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double, std::milli> elapse_ms = now - prev;

  // std::cout << "delay = " << elapse_ms.count() << " ms\n";

  for(int i = 0 ; i < num_tags; i++) {
    cout << "tag id " << tags[i].id << "\n";
    m[tags[i].id]++;
  }
  return num_tags;
}


void nv_destroy_apriltag_detector() 
{
  if(!hApriltags) return;
  nvAprilTagsDestroy(hApriltags);
}



void nv_print_stats() 
{
  vector<int> ids = {173, 319, 119, 219, 142};
  for(auto id : ids) 
  {
    std::cout << "id = " << id << " count " << m[id] << "\n";
  }
}

#define IMAGE_C (1.0)
int main( int argc, char** argv )
{
  std::cout << "TEST" << endl;

  // Load image
  std::string folderpath = "undistorted_imgs/*.jpg";
  // get list of images
  vector<std::string> filenames;
  glob(folderpath, filenames);

  Mat view, viewGray = imread(filenames[0], 1);
  Size imageSize = view.size();

  // aprilTag Stuff
  const int total_size = IMAGE_W * IMAGE_H * IMAGE_C;
  uint8_t *image_buffer = new uint8_t[total_size];

  //nv_init_apriltag_detector();

  int total_detections = 0;

  int i;
  for( i = 0; i < (int)filenames.size(); i++ )
  {
      view = imread(filenames[i], 1);
      if(view.empty())
          continue;

      int im_height = view.rows;
      int im_width  = view.cols;

      // convert to greyscale
      cvtColor(view, viewGray, cv::COLOR_RGB2GRAY);

      std::memcpy(image_buffer, viewGray.data, viewGray.total() * sizeof(uint8_t));
      
      int det1 = nv_apriltag_detector_detect(image_buffer, im_width, im_width, im_height);
      total_detections += det1;

      imshow("Image View", viewGray);

      char c = (char)waitKey();
      if( c == 27 || c == 'q' || c == 'Q' )
          break;
  }

  nv_destroy_apriltag_detector();  
}
