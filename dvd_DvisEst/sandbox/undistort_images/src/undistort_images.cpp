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

// Test inclusion of disc defs
#include <disc_layouts.hpp>

using namespace cv;
using namespace std;

template<typename T/*, typename = std::enable_if_t<std::is_integral_v<T>>*/>
std::string to_string_with_zero_padding(const T& value, std::size_t total_length)
{
    auto str = std::to_string(value);
    if (str.length() < total_length)
        str.insert(str.front() == '-' ? 1 : 0, total_length - str.length(), '0');
    return str;
}

int main( int argc, char** argv )
{
  // Quick, un-related test for disc def headers
  uint16_t apriltag_id = 109;
  map<uint16_t, disc_layout_t>::const_iterator dl_lookup = disc_layout_by_id.find(apriltag_id);
  disc_layout_t dl = dl_lookup->second;
  cout << "Look up disc index for tag id " << to_string(apriltag_id) <<  " : " << to_string(dl.disc_index) << endl;

  // params 
  string filename = "camera_cals/camera_cal800.yaml";
  string folderpath = "distorted_imgs/*.jpg";
  string folderpathsave = "undistorted_imgs/";

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
  // Increase image size by ~2x pixels to maintain the detail of centre pixels
  // during the de-warp
  // keep same size for initial spriltag test
  Size imageSizeOut = imageSize;//Size(1040.0/720.0 * imageSize.width, 1040.0/720.0 * imageSize.height);
  initUndistortRectifyMap(camera_matrix.mat(), distortion_coefficients.mat(), Mat(),
                          getOptimalNewCameraMatrix(camera_matrix.mat(), distortion_coefficients.mat(), imageSize, 1, imageSizeOut, 0),
                          imageSizeOut, CV_8UC1, map1, map2);
  int i;
  for( i = 0; i < (int)filenames.size(); i++ )
  {
      view = imread(filenames[i], 1);
      if(view.empty())
          continue;

      //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
      remap(view, rview, map1, map2, INTER_LINEAR);
      imshow("Image View", rview);

      // write to file  
      string outfile = folderpathsave + "undistort-" + to_string_with_zero_padding(i, 5) + ".jpg";
      cout << outfile << endl;
      imwrite(outfile, rview);

      if(0)
      {
        char c = (char)waitKey();
        if( c == 27 || c == 'q' || c == 'Q' )
            break;
      }
  }
}