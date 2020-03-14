#include <cctype>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <iostream>
#include <fstream>

// OpenCV stuff
#include "opencv2/core.hpp"
#include <opencv2/core/utility.hpp>
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include "opencv2/highgui.hpp"

// CPU AprilTags
#include "apriltag.h"
#include "tag36h11.h"
#include "tag36h11.h"
#include "common/homography.h"

// Disc Stuff
#include <disc_layouts.hpp>

using namespace cv;
using namespace std;

std::ofstream csvlog;

template<typename T/*, typename = std::enable_if_t<std::is_integral_v<T>>*/>
std::string to_string_with_zero_padding(const T& value, std::size_t total_length)
{
    auto str = std::to_string(value);
    if (str.length() < total_length)
        str.insert(str.front() == '-' ? 1 : 0, total_length - str.length(), '0');
    return str;
}

// Indexing functions for matrices
#define MAT3X3(x) x[9]
#define MAT4X4(x) x[16]

#define VEC2(x)   x[2]
#define VEC3(x)   x[3]
#define VEC4(x)   x[4]

// Row-major
#define i3x3(r,c)   (3*(r)+(c))
#define i4x4(r,c)   (4*(r)+(c))
void R2Q( float MAT3X3(R), float VEC4(Q) )
{
  // Note the reverse ordering
  const float m00 = R[i3x3(0,0)];
  const float m10 = R[i3x3(0,1)];
  const float m20 = R[i3x3(0,2)];
  const float m01 = R[i3x3(1,0)];
  const float m11 = R[i3x3(1,1)];
  const float m21 = R[i3x3(1,2)];
  const float m02 = R[i3x3(2,0)];
  const float m12 = R[i3x3(2,1)];
  const float m22 = R[i3x3(2,2)];
  
  float trace1 = 1.0 + m00 - m11 - m22;
  float trace2 = 1.0 - m00 + m11 - m22;
  float trace3 = 1.0 - m00 - m11 + m22;

  if( (trace1 > trace2) && (trace1 > trace3) )
  {
    if( trace1 < 0.0 ) trace1 = 0.0;
    
    const float s = 0.5 / sqrt(trace1);
    Q[0] = (m12 - m21) * s;
    Q[1] = 0.25 / s;
    Q[2] = (m01 + m10) * s; 
    Q[3] = (m02 + m20) * s; 
  }
  else if( (trace2 > trace1) && (trace2 > trace3) )
  {
    if( trace2 < 0.0 ) trace2 = 0.0;
    
    const float s = 0.5 / sqrt(trace2);
    Q[0] = (m20 - m02) * s;
    Q[1] = (m01 + m10) * s; 
    Q[2] = 0.25 / s;
    Q[3] = (m12 + m21) * s; 
  }
  else
  {
    if( trace3 < 0.0 ) trace3 = 0.0;
  
    const float s = 0.5 / sqrt(trace3);
    Q[0] = (m01 - m10) * s;
    Q[1] = (m02 + m20) * s;
    Q[2] = (m12 + m21) * s;
    Q[3] = 0.25 / s;
  }
}

void norm_quat(float quat[4]) 
{
  uint8_t i;
  float mag = 0;
  for(i=0; i<4; i++)
    mag += quat[i] * quat[i];
  mag = sqrt( mag );
  for(i=0; i<4; i++)
    quat[i] /= mag;
}

static void csv_log_open()
{
  csvlog.open("csvlog.csv", std::ios_base::trunc);// discard old file contents each run
  csvlog << "t_ms, x_m, y_m, z_m, qw, qx, qy, qz" << endl;
}

static void csv_log_close()
{
  csvlog.close();
}

static void csv_log_write(float time_ms, float x_m, float y_m, float z_m, float qw, float qx, float qy, float qz)
{
  char out[128];
  sprintf(
      out, "%0.6f, %0.3f, %0.3f, %0.3f, %0.5f, %0.5f, %0.5f, %0.5f",
      time_ms, x_m, y_m, z_m, qw, qx, qy, qz);

  csvlog << out << endl;
}

int main( int argc, char** argv )
{
  // Quick, un-related test for disc def headers
  uint16_t apriltag_id = 109;
  map<uint16_t, disc_layout_t>::const_iterator dl_lookup = disc_layout_by_id.find(apriltag_id);
  disc_layout_t dl = dl_lookup->second;
  cout << "Look up disc index for tag id " << to_string(apriltag_id) <<  " : " << to_string(dl.disc_index) << endl;
  
  // params 
  string cal_filename = "camera_cals/cal2.yaml";
  string folderpath = "../../../resources/test_throws/0_slow_pitch_putt/*.jpg";
  string folderpathsave = "overlay_images/";

  // define local variables
  float MAT3X3(R_out);
  float VEC4(Q_out); 

  // get list of images
  vector<String> filenames;
  glob(folderpath, filenames);

  // get camera cal parameters from .yaml file
  FileStorage fs;
  fs.open(cal_filename, FileStorage::READ);
  if (!fs.isOpened())
  {
      cerr << "Failed to open cal file" << cal_filename << endl;
      return 1;
  }

  FileNode camera_matrix                    = fs["camera_matrix"];
  //FileNode distortion_coefficients          = fs["distortion_coefficients"];
  //FileNode avg_reprojection_error           = fs["avg_reprojection_error"];
  //FileNode per_view_reprojection_errors     = fs["per_view_reprojection_errors"];
  //FileNode extrinsic_parameters             = fs["extrinsic_parameters"];
  //image_points

  // camera matrix is 3x3:
  //  fx   0    cx
  //  0    fy   cy
  //  0    0    1 
  cout << "Camera Matrix: " << endl << camera_matrix.mat() << endl;
  const float sFx = camera_matrix.mat().at<double>(0,0);
  const float sFy = camera_matrix.mat().at<double>(1,1);
  const float sCx = camera_matrix.mat().at<double>(0,2);
  const float sCy = camera_matrix.mat().at<double>(1,2);

  cout << "Fx: " << sFx << endl;
  cout << "Fy: " << sFy << endl;
  cout << "Cx: " << sCx << endl;
  cout << "Cy: " << sCy << endl;

  Mat view, viewGray, viewOverlay;

  // read in first image (use for size ref)
  view = imread(filenames[0], 1);
  //Size imageSize = view.size();

  // Init AprilTag Stuff
  // tag parameters used to convert the transformation matrix translations
  // to actual measurements in meters
  const float tag_size_mm = 108;//52; //width of tag
  // Tag size is the size of the tag in your desired units. I.e., if
  // your tag measures 0.25m along the side, your tag size is
  // 0.25. (The homography is computed in terms of *half* the tag
  // size, i.e., that a tag is 2 units wide as it spans from -1 to +1
  const float homography_to_m_z  = tag_size_mm / 2.0 *0.001 * sqrt(2.0); //this seems to be wrong? by sqrt(2)?
  const float homography_to_m_xy = tag_size_mm / 2.0 *0.001;

  apriltag_family_t *tf = tag36h11_create();

  apriltag_detector_t *td = apriltag_detector_create();
  apriltag_detector_add_family_bits(td, tf, 1);
  td->quad_decimate   = 2.0;
  td->quad_sigma      = 0.0;
  td->nthreads        = 1;
  td->debug           = 0;
  td->refine_edges    = 0; // might want to disable this for 522fps

  const int quiet     = 1;
  //const int maxiters  = 1;

  // Assume at we got all frames at our desired framerate for test purposes
  float time_ms = 0.0;
  float dt_ms = 1.0/522.0; 

  // open logfile
  csv_log_open();

  int i;
  int throttle = (int)filenames.size();
  for( i = 0; i < throttle; i++ )
  {
    // get opencv image matrix
    //int im_height = view.rows;
    //int im_width  = view.cols;
    //cout << "Read Image: " << filenames[i] << endl;
    view = imread(filenames[i], 1);


    // convert to greyscale
    cvtColor(view, viewGray, cv::COLOR_RGB2GRAY);
    // go back to RBG land
    cvtColor(viewGray, view, cv::COLOR_GRAY2RGB);

    //cout << "Cols: " << viewGray.cols << ", Rows: " << viewGray.rows << endl;

    //format opencv Mat into apriltag wrapper
    image_u8_t img_header = 
    {     
      .width  = viewGray.cols,
      .height = viewGray.rows,
      .stride = viewGray.cols,
      .buf    = viewGray.data
    };

    // detect those apriltags
    zarray_t *detections = apriltag_detector_detect(td, &img_header);

    const int detect_num = zarray_size(detections);

    cout << "Tags Detected: " << to_string(detect_num) << " in Image #" << to_string(i) << endl;

    if(detect_num > 0)
    {
      view.copyTo(viewOverlay);
    }

    // snap to first tag_id;
    int16_t track_tag_id = -1;

    for (int tag = 0; tag < detect_num; tag++) 
    {

      // store XYZ translations for tags in m
      std::vector<double> x_m(detect_num), y_m(detect_num), z_m(detect_num);
      std::vector<int> tag_id(detect_num);

      apriltag_detection_t *det;
      zarray_get(detections, tag, &det);

      // determine actual pose (rotation and translation) from Homography matrix and camera intrinsics
      // This seems to get the depth correct, but not the translations in X and Y
      // might need better projection from the sensor through to the camera frame?
      // For homography_to_pose, you have to pass in a negative fx parameter.
      // This is again due to the OpenCV convention of having z negative.
      matd_t * T = homography_to_pose(det->H, -sFx, sFy, sCx, sCy);

      // store translation points for plotting
      tag_id[tag] = det->id;

      if(track_tag_id < 0)
      {
        //first tag is only tag!
        track_tag_id = tag_id[tag];
      }

      x_m[tag] = MATD_EL(T, 0, 3)*homography_to_m_xy;
      y_m[tag] = MATD_EL(T, 1, 3)*homography_to_m_xy;
      z_m[tag] = MATD_EL(T, 2, 3)*homography_to_m_z;

      // convert rotation matrix to quaternion
      R_out[i3x3(0,0)] = MATD_EL(T, 0, 0);
      R_out[i3x3(0,1)] = MATD_EL(T, 0, 1);
      R_out[i3x3(0,2)] = MATD_EL(T, 0, 2);
      R_out[i3x3(1,0)] = MATD_EL(T, 1, 0);
      R_out[i3x3(1,1)] = MATD_EL(T, 1, 1);
      R_out[i3x3(1,2)] = MATD_EL(T, 1, 2);
      R_out[i3x3(2,0)] = MATD_EL(T, 2, 0);
      R_out[i3x3(2,1)] = MATD_EL(T, 2, 1);
      R_out[i3x3(2,2)] = MATD_EL(T, 2, 2);
      R2Q(R_out, Q_out);
      norm_quat(Q_out); // make sure this a unit quaternion

      // Print to log file
      if(tag_id[tag] == track_tag_id)
      {
        csv_log_write(time_ms, x_m[tag], y_m[tag], z_m[tag], Q_out[0], Q_out[1], Q_out[2], Q_out[3]);
      }

      if (!quiet)
      {
        printf("\ndetection %3d: id (%2dx%2d)-%-4d, hamming %d, margin %8.3f\n",
             tag, det->family->nbits, det->family->h, det->id, det->hamming, det->decision_margin);

        printf("Homography Matrix:\n[");
        matd_print(det->H, " %f,");
        printf("]\n");

        printf("Transformation Matrix:\n[");
        matd_print(T, " %f,");
        printf("]\n");

        printf("Tag Position in meters?:[XYZ] [%lf, %lf, %lf]\n", 
        x_m[tag], //why are these /2???
        y_m[tag],
        z_m[tag]);

        cout << "Corners: " << endl;
        for (int xx = 0; xx < 4; xx++)
        {
          for (int yy = 0; yy < 2; yy++)
          {
            cout << det->p[xx][yy] << ' ';
          }
          cout << endl;
        }
      }

      // only overlay great detections
      const bool show_overlay = (det->hamming <= 1);
      if(show_overlay)
      {
        // overlay square at tag corners
        // Draw on Image
        Point corner_points[1][4];
        for (int c = 0; c < 4; c++)
        {
          corner_points[0][c] = Point( det->p[c][0], det->p[c][1] );
        }
        
        const Point* ppt[1] = { corner_points[0] };
        int npt[] = { 4 };
        int lineType = 8;
        fillPoly( viewOverlay,
              ppt,
              npt,
              1,
              Scalar( 0, 255, 0 ),
              lineType );
      }
    }

    apriltag_detections_destroy(detections);

    if(detect_num > 0)
    {
      // Blend in overlay
      const float alpha = 0.5;
      cv::addWeighted(viewOverlay, alpha, view, 1 - alpha, 0, view);
    }

    imshow("Image View", view);

    string outfile = folderpathsave + "overlay-" + to_string_with_zero_padding(i, 5) + ".jpg";
    //cout << outfile << endl;
    imwrite(outfile, view);

    // pause for detects
    if(0 && detect_num > 0)
    {
      char c = (char)waitKey();
      if( c == 27 || c == 'q' || c == 'Q' )
        break;
    }

    time_ms += dt_ms;    
  }

  // close log file
  csv_log_close();

  apriltag_detector_destroy(td);
  tag36h11_destroy(tf);
}