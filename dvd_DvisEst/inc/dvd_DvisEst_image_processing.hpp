#ifndef DVD_DVISEST_IMAGE_PROCESSING_HPP
#define DVD_DVISEST_IMAGE_PROCESSING_HPP

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// simple helloworld openccv test
bool dvd_DvisEst_image_processing_test(void);
// Get camera cal and init local statics with chosen image scale
bool dvd_DvisEst_image_processing_init(const cv::String camera_cal_file, const double image_scale);
// get scaled camera params
bool dvd_DvisEst_image_processing_get_camera_params(double * Fx, double * Fy, double * Cx, double * Cy);
// Undistort opencv mat image, and output using the same pointer
void dvd_DvisEst_image_processing_undistort_image(cv::Mat * image_in, cv::Mat * image_out);

#endif // DVD_DVISEST_IMAGE_PROCESSING_HPP