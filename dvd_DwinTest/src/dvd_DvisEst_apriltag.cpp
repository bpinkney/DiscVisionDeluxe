#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <iomanip>
#include <iostream>

// CPU AprilTags
#include "apriltag.h"
#include <tag36h11.h>
#include "common/homography.h"

// threading stuff
#include <thread>
#include <future>
#include <queue>
#include <mutex>
#include <atomic>

#include <exception>
#include <typeinfo>
#include <stdexcept>

// Disc Stuff
#include <dvd_DvisEst_apriltag.hpp>
#include <dvd_DvisEst_image_capture.hpp>
#include <dvd_DvisEst_image_processing.hpp>
#include <disc_layouts.hpp>

using namespace std;

// Test a simple call to the apriltag lib
bool dvd_DvisEst_apriltag_test(void)
{
  apriltag_family_t *tf = tag36h11_create();
  tag36h11_destroy(tf);

  return true;
}

