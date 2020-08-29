#ifndef DVD_DVISEST_IMAGE_CAPTURE_HPP
#define DVD_DVISEST_IMAGE_CAPTURE_HPP

#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32)) && !defined(SPINNAKER_ALLOWED)
// not available in mingw64 for windows! (sad)
// I'm starting to think spinnaker and apriltag are never meant to
// be together on windows...
#else
#define SPINNAKER_ALLOWED
#endif

#include <string>
#include <iostream>

#include <atomic>

// OpenCV stuff
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"


bool dvd_DvisEst_image_capture_test(void);

#endif // DVD_DVISEST_IMAGE_CAPTURE_HPP