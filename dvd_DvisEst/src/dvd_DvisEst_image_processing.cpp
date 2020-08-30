#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(WIN64) || defined(_WIN64) || defined(__WIN64))
#define IS_WINDOWS
#endif

#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>

using namespace cv;
using namespace std;

// define local statics
double  sv_image_scale = 1.0;
cv::Mat sv_camera_matrix;
cv::Mat sv_distortion_coefficients;

// Transformations used for undistortion
cv::Mat sv_undistort_map0;
cv::Mat sv_undistort_map1;

bool dvd_DvisEst_image_processing_test(void)
{
  Mat *view = new Mat();

  return true;
}

static void dvd_DvisEst_image_processing_load_calib(const cv::String camera_cal_file)
{
  // get camera cal parameters from .yaml file
  FileStorage fs;
  fs.open(camera_cal_file, FileStorage::READ);
  if (!fs.isOpened())
  {
      cerr << "Failed to open camera cal: " << camera_cal_file << endl;
  }
  else
  {
    FileNode camera_matrix                    = fs["camera_matrix"];
    FileNode distortion_coefficients          = fs["distortion_coefficients"];
    FileNode image_width                      = fs["image_width"];
    FileNode image_height                     = fs["image_height"];
    //FileNode avg_reprojection_error           = fs["avg_reprojection_error"];
    //FileNode per_view_reprojection_errors     = fs["per_view_reprojection_errors"];
    //FileNode extrinsic_parameters             = fs["extrinsic_parameters"];

    sv_camera_matrix           = camera_matrix.mat();
    sv_distortion_coefficients = distortion_coefficients.mat();

    // Compute optimal undistort tranformation mappings
    Size image_size     = Size((double)image_width, (double)image_height);
    Size image_size_out = Size(sv_image_scale * image_size.width, sv_image_scale * image_size.height);

    // Populate map0 and map1 for undistortion
    const bool fisheye = true;
    if(fisheye)
    {
      Mat new_cam_mat;
      fisheye::estimateNewCameraMatrixForUndistortRectify(sv_camera_matrix, sv_distortion_coefficients, image_size,
                                                          Matx33d::eye(), new_cam_mat, 1.0, image_size_out, 1.0);
      fisheye::initUndistortRectifyMap(sv_camera_matrix, sv_distortion_coefficients, Matx33d::eye(),
                            new_cam_mat,
                            image_size_out, CV_16SC2, sv_undistort_map0, sv_undistort_map1);
    }
    else
    {
    initUndistortRectifyMap(sv_camera_matrix, sv_distortion_coefficients, Mat(),
                          getOptimalNewCameraMatrix(sv_camera_matrix, sv_distortion_coefficients, image_size, 1, image_size_out, 0),
                          image_size_out, CV_8UC1, sv_undistort_map0, sv_undistort_map1);
    }

  }
  fs.release();
}

bool dvd_DvisEst_image_processing_init(const cv::String camera_cal_file, const double image_scale)
{
  cerr << "Call dvd_DvisEst_image_processing_init" << endl;

  sv_image_scale = image_scale;

  dvd_DvisEst_image_processing_load_calib(camera_cal_file);

  return true;
}

// These parameters are scaled linearly by sv_image_scale to reflect the resize step performed during the undistort
bool dvd_DvisEst_image_processing_get_camera_params(double * Fx, double * Fy, double * Cx, double * Cy)
{
  if(sv_camera_matrix.empty())
  {
    cerr << "Can't retrieve camera intrinsics, camera cal was never loaded!" << endl;
    return false;
  }

  cerr << "Camera Calibration Pre Parameters: Fx' = " << 
  sv_camera_matrix.at<double>(0,0) << ", Fy' = " << 
  sv_camera_matrix.at<double>(1,1) << ", Cx' = " << 
  sv_camera_matrix.at<double>(0,2) << ", Cy' = " << 
  sv_camera_matrix.at<double>(1,2) << endl;

  *Fx = sv_camera_matrix.at<double>(0,0) * sv_image_scale;
  *Fy = sv_camera_matrix.at<double>(1,1) * sv_image_scale;
  *Cx = sv_camera_matrix.at<double>(0,2) * sv_image_scale;
  *Cy = sv_camera_matrix.at<double>(1,2) * sv_image_scale;

  return true;
}

// Undistort opencv mat image, and output using the same pointer
void dvd_DvisEst_image_processing_undistort_image(cv::Mat * image_in, cv::Mat * image_out)
{
  // No cal loaded? return early
  if(sv_camera_matrix.empty())
  {
    cerr << "Can't undistort image, camera cal was never loaded!" << endl;
    return;
  }

  Mat d_image, ud_image;
  d_image = *image_in;

  // use re-map to undistort, linear interpolation
  remap(d_image, ud_image, sv_undistort_map0, sv_undistort_map1, INTER_LINEAR);

  // point to undistorted image, free memory from extra mat
  *image_out = ud_image;
  d_image.release();
}
