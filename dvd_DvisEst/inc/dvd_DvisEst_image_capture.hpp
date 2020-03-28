#ifndef DVD_DVISEST_IMAGE_CAPTURE_HPP
#define DVD_DVISEST_IMAGE_CAPTURE_HPP

#include <string>
#include <iostream>

// OpenCV stuff
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// Structures
// Define struct to hold image matrices, timestamps, and frame IDs generated by the camera
struct image_capture_t
{
    image_capture_t()
        : image_data(cv::Mat()), timestamp_ns(0), frame_id(0) {}
    image_capture_t(cv::Mat _image_data, uint64_t _timestamp_ns, uint32_t _frame_id)
        : image_data(_image_data), timestamp_ns(_timestamp_ns), frame_id(_frame_id) {}
    cv::Mat     image_data;   // Image data from the camera capture, converted to cv::Mat
    uint64_t    timestamp_ns; // Timestamp reported by camera, nanoseconds, don't leave your program running for more than 500 years at a time!
    uint32_t    frame_id;     // Frame ID reported by camera, sequential, don't leave your program running for more than 90 days at a time!
};

// Functions
bool dvd_DvisEst_image_capture_test(void);
// load test images into the capture queue and return
bool dvd_DvisEst_image_capture_load_test_queue(const cv::String imgdir_src, const double dt);
// Return the next captured image from the front of the queue
bool dvd_DvisEst_image_capture_get_next_image_capture(image_capture_t * image_capture);





#endif // DVD_DVISEST_IMAGE_CAPTURE_HPP