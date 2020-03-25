#include <iostream>

// disc includes
#include <dvd_DvisEst_image_capture.hpp>
#include <dvd_DvisEst_image_processing.hpp>
#include <dvd_DvisEst_apriltag.hpp>

using namespace std;

int main(int argc, char** argv )
{
    cout << endl << "AprilTag function test?  " << dvd_DvisEst_apriltag_test() << endl;
    cout << endl << "OpenCV function test?    " << dvd_DvisEst_image_processing_test() << endl;
    cout << endl << "Spinnaker function test? " << dvd_DvisEst_image_capture_test() << endl;
    cout << endl;
    cout << "dvd_DvisEst.out";
    cout << endl;

    //waitKey(0);

    return 0;
}