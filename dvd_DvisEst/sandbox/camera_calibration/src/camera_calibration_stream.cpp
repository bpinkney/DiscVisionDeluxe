
// This is the old version of opencv!

// std headers
#include <vector>
#include <fstream>
#include <ostream>
#include <iostream>

// Include the standard OpenCV headers
#include <cv.hpp>
#include <imgcodecs.hpp>
#include <highgui.hpp>

// We'll include the OpenCV namespace so we can use its functions directly (without a cv:: everytime): 
using namespace cv;
using namespace std;

// define funcs
static bool saveCameraCalibration(string fname, Mat cameraMatrix, Mat distorceCoefiientes);

// Now the main function. We create some variables. The number of boards you want to capture, the number of internal corners horizontally and the number of internal corners vertically (That's just how the algorithm works)
int main()
{
  // List camera devices
  cv::VideoCapture camera;
  int device_counts = 0;
  while ( true ) 
  {
    if ( !camera.open(device_counts++) ) 
    {
      break;
    }
  }
  camera.release();
  cout << "Camera Devices: " << device_counts - 1 << std::endl;

  int camDevice = 1;
  int numBoards = 40;
  int numCornersHor = 7;
  int numCornersVer = 9;

  if(0)
  {
    //Then, we get these values from the user:
    printf("Enter which capture device to use: ");
    scanf("%d", &camDevice);

    printf("Enter number of corners along width: ");
    scanf("%d", &numCornersHor);

    printf("Enter number of corners along height: ");
    scanf("%d", &numCornersVer);

    printf("Enter number of boards: ");
    scanf("%d", &numBoards);
  }

  //We also create some additional variables that we'll be using later on. 
  int numSquares = numCornersHor * numCornersVer;
  Size board_sz = Size(numCornersHor, numCornersVer);

  //See the Size? That's OpenCV in C++. Next, we create a camera capture. We want live feed for out calibration! 
  VideoCapture capture = VideoCapture(camDevice);

  //Next, we'll create a list of objectpoints and imagepoints. 
  vector< vector<Point3f> > object_points;
  vector< vector<Point2f> > image_points;

  //What do these mean? For those unfamiliar with C++, a "vector" is a list. This list contains items of the type mentioned within the angular brackets < > (it's called generic programming). So, we're creating a list of list of 3D points (Point3f) and a list of list of 2D points (Point2f).
  //object_points is the physical position of the corners (in 3D space). This has to be measured by us. [write relationg between each list item and list's eh you get the point]
  //image_points is the location of the corners on in the image (in 2 dimensions). Once the program has actual physical locations and locations on the image, it can calculate the relation between the two.
  //And because we'll use a chessboard, these points have a definite relations between them (they lie on straight lines and on squares). So the "expected" - "actual" relation can be used to correct the distortions in the image.
  //Next, we create a list of corners. This will temporarily hold the current snapshot's chessboard corners. We also declare a variable that will keep a track of successfully capturing a chessboard and saving it into the lists we declared above. 
  vector<Point2f> corners;
  int successes=0;

  //Then we create two images and get the first snapshot from the camera: 
  Mat image;
  Mat gray_image;
  capture >> image;

  //Next, we do a little hack with object_points. Ideally, it should contain the physical position of each corner. The most intuitive way would be to measure distances "from" the camera lens. That is, the camera is the origin and the chessboard has been displaced. 
  //Usually, it's done the other way round. The chessboard is considered the origin of the world. So, it is the camera that is moving around, taking different shots of the camera. So, you can set the chessboard on some place (like the XY plane, of ir you like, the XZ plane). 
  //Mathematically, it makes no difference which convention you choose. But it's easier for us and computationally faster in the second case. We just assign a constant position to each vertex.
  vector<Point3f> obj;
  for(int j=0;j<numSquares;j++)
    obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

  printf("Press 'Space' to collect calibration data");

  while(successes<numBoards)
  {
    cvtColor(image, gray_image, CV_BGR2GRAY);
    bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

    if(found)
    {
        cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        drawChessboardCorners(gray_image, board_sz, corners, found);
    }

    //The findChessboardCorners does exactly what it says. It looks for board_sz sized corners in image. If it detects such a pattern, their pixel locations are stored in corners and found becomes non-zero. The flags in the last parameter are used to improve the chances of detecting the corners. Check the OpenCV documentation for details about the three flags that can be used.
    //If corners are detected, they are further refined. Subpixel corners are calculated from the grayscale image. This is an iterative process, so you need to provide a termination criteria (number of iterations, amount of error allowed, etc). 
    //Also, if corners are detected, they're drawn onto the screen using the handy _drawChessboardCorners _function
    //Next we update the display the images and grab another frame. We also try to capture a key:
    imshow("win1", image);
    imshow("win2", gray_image);
    
    capture >> image;

    //bool capture_and_process = false;// change this to capture constantly later

    int key = waitKey(1);

    //if(key > 0)
    //  printf("KeyPressed: %d\n", key);

    //If escape is pressed, we quit. No questions asked. If corners were found and space bar was pressed, we store the results into the lists. And if we reach the required number of snaps, we break the while loop too: 
    if(key==27)
      return 0;

    if(key==' ' && found!=0)
    {
      image_points.push_back(corners);
      object_points.push_back(obj);

      printf("Snap stored! %d/%d\n", successes+1, numBoards);

      successes++;

      if(successes>=numBoards)
        break;
    }
  }

  //Next, we get ready to do the calibration. We declare variables that will hold the unknowns: 
  Mat intrinsic = Mat(3, 3, CV_32FC1);
  Mat distCoeffs;
  vector<Mat> rvecs;
  vector<Mat> tvecs;

  //We modify the intrinsic matrix with whatever we do know. 
  //The camera's aspect ratio is 1 (that's usually the case... If not, change it as required). 
  //aspect ratio = fx/fy
  // this should be right for 1920/1080
  intrinsic.ptr<float>(0)[0] = 1;       //fx
  intrinsic.ptr<float>(1)[1] = 0.5625; //fy

  //intrinsic elements (0,0) and (1,1) are the focal lengths along the X and Y axis.
  //intrinsic elements (2,0) and (2,1) are the centres along the X and Y axis.
  calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

  cout << "Camera Intrinsics: " << intrinsic << std::endl;
  cout << "Camera Distortion Coeffs: " << distCoeffs << std::endl;

  // Save the camera cal to a file
  saveCameraCalibration("camera_cal.txt", intrinsic, distCoeffs);

  //Undistort
  //After this statement, you'll have the intrinsic matrix, distortion coefficients and the rotation+translation vectors. The intrinsic matrix and distortion coefficients are a property of the camera and lens. So as long as you use the same lens (ie you don't change it, or change its focal length, like in zoom lenses etc) you can reuse them. In fact, you can save them to a file if you want and skip the entire chessboard circus!
  //Note: The calibrateCamera function converts all matrices into 64F format even if you initialize it to 32F. Thanks to Michael Koval!
  Mat imageUndistorted;
  while(1)
  {
      capture >> image;
      undistort(image, imageUndistorted, intrinsic, distCoeffs);

      imshow("win1", image);
      imshow("win2", imageUndistorted);
      waitKey(1);
  }

  capture.release();

  return 0;
}

static bool saveCameraCalibration(string fname, Mat cameraMatrix, Mat distorceCoefiientes) 
{
  ofstream outStream(fname);
  if (outStream) 
  {
    uint16_t rows = cameraMatrix.rows;
    uint16_t cols = cameraMatrix.cols;

    for (int r = 0; r < rows; r++) 
    {
      for (int c = 0; c < cols; c++) 
      {
        double value = cameraMatrix.at<double>(r, c);
        outStream << value << endl;
      }
    }

    rows = distorceCoefiientes.rows;
    cols = distorceCoefiientes.cols;

    for (int r = 0; r < rows; r++) 
    {
      for (int c = 0; c < cols; c++) 
      {
        double value = distorceCoefiientes.at<double>(r, c);
        outStream << value << endl;
      }
    }
    outStream.close();
    return true;
  }
  return false;
}



