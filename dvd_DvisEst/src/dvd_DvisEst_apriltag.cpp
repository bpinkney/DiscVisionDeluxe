#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>

// CPU AprilTags
#include "apriltag.h"
#include <tag36h11.h>
#include "common/homography.h"

// Disc Stuff
#include <dvd_DvisEst_apriltag.hpp>
#include <dvd_DvisEst_image_capture.hpp>
#include <dvd_DvisEst_estimate.hpp>
#include <disc_layouts.hpp>

using namespace std;

// Test a simple call to the apriltag lib
bool dvd_DvisEst_apriltag_test(void)
{
  apriltag_family_t *tf = tag36h11_create();
  tag36h11_destroy(tf);

  return true;
}

// Initialize apriltag thread pool
void dvd_DvisEst_apriltag_init(void)
{
  cerr << "Call dvd_DvisEst_apriltag_init" << endl;
}

// Start apriltag thread with new image_capture_t
