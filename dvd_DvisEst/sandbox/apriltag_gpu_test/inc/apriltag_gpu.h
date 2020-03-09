#pragma once
#include<cstdint>

static int IMAGE_W = 640;
static int IMAGE_H = 480;
static int IMAGE_C = 1;

int nv_create_apriltag_detector();
int nv_apriltag_detector_detect (uint8_t *data, int pitch, int width, int height);
void nv_destroy_apriltag_detector();
int nv_init_apriltag_detector();
void nv_print_stats();
