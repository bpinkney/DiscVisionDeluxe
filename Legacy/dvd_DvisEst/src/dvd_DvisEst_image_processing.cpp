#if (defined(WIN32) || defined(_WIN32) || defined(__WIN32) || defined(WIN64) || defined(_WIN64) || defined(__WIN64)) && !defined(IS_WINDOWS)
#define IS_WINDOWS

#include <windows.h>
// option to disable all warnings (does this work? NOPE)
#pragma warning(push, 0)

// fix for garbage MSVC c++17 support (c'mon guys, sweet christ)
#define _HAS_STD_BYTE 0
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
  cv::Mat *view = new cv::Mat();

  return true;
}

static void dvd_DvisEst_image_processing_load_calib(const cv::String camera_cal_file)
{
  // get camera cal parameters from .yaml file
  cv::FileStorage fs;
  fs.open(camera_cal_file, cv::FileStorage::READ);
  if (!fs.isOpened())
  {
      cerr << "Failed to open camera cal: " << camera_cal_file << endl;
  }
  else
  {
    cv::FileNode camera_matrix                    = fs["camera_matrix"];
    cv::FileNode distortion_coefficients          = fs["distortion_coefficients"];
    cv::FileNode image_width                      = fs["image_width"];
    cv::FileNode image_height                     = fs["image_height"];
    //FileNode avg_reprojection_error           = fs["avg_reprojection_error"];
    //FileNode per_view_reprojection_errors     = fs["per_view_reprojection_errors"];
    //FileNode extrinsic_parameters             = fs["extrinsic_parameters"];

    sv_camera_matrix           = camera_matrix.mat();
    sv_distortion_coefficients = distortion_coefficients.mat();

    // Compute optimal undistort tranformation mappings
    cv::Size image_size     = cv::Size((double)image_width, (double)image_height);
    cv::Size image_size_out = cv::Size(sv_image_scale * image_size.width, sv_image_scale * image_size.height);

    // Populate map0 and map1 for undistortion
    const bool fisheye = true;
    if(fisheye)
    {
      cv::Mat new_cam_mat;
      cv::fisheye::estimateNewCameraMatrixForUndistortRectify(sv_camera_matrix, sv_distortion_coefficients, image_size,
                                                          cv::Matx33d::eye(), new_cam_mat, 1.0, image_size_out, 1.0);
      cv::fisheye::initUndistortRectifyMap(sv_camera_matrix, sv_distortion_coefficients, cv::Matx33d::eye(),
                            new_cam_mat,
                            image_size_out, CV_16SC2, sv_undistort_map0, sv_undistort_map1);
    }
    else
    {
    cv::initUndistortRectifyMap(sv_camera_matrix, sv_distortion_coefficients, cv::Mat(),
                          cv::getOptimalNewCameraMatrix(sv_camera_matrix, sv_distortion_coefficients, image_size, 1, image_size_out, 0),
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

  cv::Mat d_image, ud_image;
  d_image = *image_in;

  // use re-map to undistort, linear interpolation
  cv::remap(d_image, ud_image, sv_undistort_map0, sv_undistort_map1, cv::INTER_LINEAR);

  // point to undistorted image, free memory from extra mat
  *image_out = ud_image;
  d_image.release();
}
