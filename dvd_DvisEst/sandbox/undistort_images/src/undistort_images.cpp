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

int main( int argc, char** argv )
{
  // params 
  string filename = "camera_cal_sample.yaml";
  string folderpath = "distorted_imgs/*.jpg";

  // get list of images
  vector<String> filenames;
  glob(folderpath, filenames);

  // get camera cal parameters from .yaml file
  FileStorage fs;
  fs.open(filename, FileStorage::READ);
  if (!fs.isOpened())
  {
      cerr << "Failed to open " << filename << endl;
      return 1;
  }

  FileNode camera_matrix                    = fs["camera_matrix"];
  FileNode distortion_coefficients          = fs["distortion_coefficients"];
  FileNode avg_reprojection_error           = fs["avg_reprojection_error"];
  FileNode per_view_reprojection_errors     = fs["per_view_reprojection_errors"];
  FileNode extrinsic_parameters             = fs["extrinsic_parameters"];
  //image_points

  cout << "Camera Matrix: " << endl << camera_matrix.mat() << endl;

  cout << "First Image File: " << endl << filenames[0] << endl;

  Mat view, rview, map1, map2;

  // read in first image (use for size ref)
  view = imread(filenames[0], 1);
  Size imageSize = view.size();

  // undistort  
  initUndistortRectifyMap(camera_matrix.mat(), distortion_coefficients.mat(), Mat(),
                          getOptimalNewCameraMatrix(camera_matrix.mat(), distortion_coefficients.mat(), imageSize, 1, imageSize, 0),
                          imageSize, CV_16SC2, map1, map2);
  int i;
  for( i = 0; i < (int)filenames.size(); i++ )
  {
      view = imread(filenames[i], 1);
      if(view.empty())
          continue;

      //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
      remap(view, rview, map1, map2, INTER_LINEAR);
      imshow("Image View", rview);
      char c = (char)waitKey();
      if( c == 27 || c == 'q' || c == 'Q' )
          break;
  }
}