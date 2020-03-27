# DiscVisionDeluxe Vision Capture and Initial State Estimator

Sequence of dvd_visEst routine:
- Observe high-fps video feed
- Run AprilTag detector on each frame
- Upon detection, use single/multiple tag detections to produce a single position and orientation measurement
- Use pos/orient measurements to produce a KF-style disc state estimate
- Update state estimate throughout disc flight to obtain confident states and derivatives
- Serve finalized initial state to dvd_DfisX and dvd_DgrafX

To build with CMAKE:

Linux:
<add lib deps and build steps here, this will be updated shortly as it is less challenging than WIN>

To build using CMake:
``` bash
cd build/
cmake ..
make
```
You can just make install apriltag and opencv, easy!


Windows:

1. Install OpenCV
- you can just the binaries distributed here: 
  https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html
  http://sourceforge.net/projects/opencvlibrary/files/opencv-win/
- Use the installer to install to C:/opencv
- Add the following entry to your environment variables"
    OpenCV_DIR C:/opencv/build/x64/vc15/lib
- Edit the system PATH, and add the entry:
    %OPENCV_DIR%../bin

2. Install CMAKE
cmake.org/download/

3. Install MSYS2 (and MINGW64 through it)
- grab the installer from https://www.msys2.org/
- in mysy2 prompt:
``` bash
pacman -Syu a couple times (close and re-open shell when prompted, then run it again)
pacman -S mingw-w64-x86_64-toolchain
pacman -S mingw-w64-x86_64-python-numpy
pacman -S mingw-w64-x86_64-cmake
```

3. Install apriltags (On WINDOWS, wooooo)
- check out apriltag from github to C:\apriltag
  (cd C:\ ; git clone https://github.com/AprilRobotics/apriltag.git;)
- Open a MSYS2 mingw64 prompt, and navigate to C:\apriltag
``` bash
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make.exe
```
- (You should have a libapriltag.so file in C:\apriltag now)

4. Build the dvd_DvisEst test app
- cd to dvd_DvisEst in your git checkout
``` bash
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make.exe
```
- copy the libapriltag.so file to your .exe bin dir (cp C:\apriltag\libapriltag.so DiscVisionDeluxe\bin\)

5. Run dvd_DvisEst.exe! woot!


There might have to be some visual studio stuff soon:
VS stuff (darn you Spinnaker...):
In Visual Studio
Project -> Manage Nuget Packages
In browse Tab search for 'pthread'
Select Install for the pthreads lib

OR

vcpkg.exe install pthread?
vcpkg.exe integrate install?

This works for dvd_DvisEst, but not for apriltags, drat

