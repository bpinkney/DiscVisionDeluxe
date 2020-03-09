#include<iostream>
#include<limits>
#include <chrono>
#include <unordered_map>
#include "nvAprilTags.h"

//#include "apriltag_gpu.h"

using namespace std;

static nvAprilTagsHandle hApriltags = nullptr;

static float sFx = 205;
static float sFy = 205;
static float sCx = 320;
static float sCy = 240;
static float sTagDimension = 0.224;
static uint8_t* buffer_cpu = nullptr;
static uint8_t* buffer_gpu = nullptr;

nvAprilTagsImageInput_st sNvData;


uint8_t *allocate_uint8_buffer(int width, int height, int color_comp_per_pixel)
{
    uint8_t *tempPoint = nullptr;
    checkCudaErrors(cudaMalloc((void **)&tempPoint, width*height*color_comp_per_pixel*sizeof(uint8_t) ));
    return tempPoint;
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

void upload_data_to_uchar4_gpu(uint8_t *data, int pitch, int width, int height) {

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
int nv_init_apriltag_detector() {
    if(hApriltags) return -1;
    buffer_cpu = (uint8_t*)malloc(sizeof(uchar4) * IMAGE_W * IMAGE_H);
    buffer_gpu = allocate_uint8_buffer(IMAGE_W, IMAGE_H, sizeof(uchar4));
    return 0;
}

int nv_create_apriltag_detector(int width, int height) {
    if(hApriltags) nv_destroy_apriltag_detector();

    nvAprilTagsCameraIntrinsics_t cam = {sFx, sFy, sCx, sCy };
    sNvData.dev_ptr = (uchar4*)buffer_gpu;
    sNvData.pitch   = width * sizeof(uchar4);
    sNvData.width   = width;
    sNvData.height  = height;
    return nvCreateAprilTagsDetector(&hApriltags, width, height, NVAT_TAG36H11, &cam, sTagDimension);
}

static unordered_map<int, int> m;

int nv_apriltag_detector_detect (uint8_t *data, int pitch, int width, int height) {
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


void nv_destroy_apriltag_detector() {
    if(!hApriltags) return;
    nvAprilTagsDestroy(hApriltags);
}



void nv_print_stats() {
    vector<int> ids = {173, 319, 119, 219, 142};
    for(auto id : ids) {
        std::cout << "id = " << id << " count " << m[id] << "\n";
    }
}
