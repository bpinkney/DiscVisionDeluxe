# DiscVisionDeluxe Vision Capture and Initial State Estimator

Sequence of dvd_visEst routine:
- Observe high-fps video feed (Thread 1)
- Run AprilTag detector on each frame (Threads 2-N)
- Upon detection, use single/multiple tag detections to produce a single position and orientation measurement (Threads 2-N)
- Use pos/orient measurements to produce a KF-style disc state estimate (Thread N+1, although this runs after frame collection is complete technically)
- Update state estimate throughout disc flight to obtain confident states and derivatives (Thread N+1)
- Serve finalized initial state to dvd_DfisX and dvd_DgrafX (Main Thread)


## Linux:

You can just cmake install opencv, apriltag, and eigen; easy!

### 1. Install openCV
- 'cd' to the empty 'DiscVisionDeluxe/dvd_DvisEst/lib/opencv' directory
- get rid of placeholder file
``` bash
rm *
```
- clone repo
``` bash
git clone https://github.com/opencv/opencv.git
```
- build with cmake
``` bash
mkdir build; cd build
cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=YES -D CMAKE_INSTALL_PREFIX=/usr/local ..
```
- make and install (change jN to your 'number fo processors-1' if you still want to do other stuff)
``` bash
make -j7
sudo make install
```

### 2. Install apriltag
- cd to the empty 'DiscVisionDeluxe/dvd_DvisEst/lib/apriltag' directory
- get rid of placeholder file
``` bash
rm *
```
- clone repo
``` bash
git clone https://github.com/AprilRobotics/apriltag.git
```
- build with cmake
``` bash
mkdir build; cd build
cmake ..
```
- make and install (change jN to your 'number fo processors-1' if you still want to do other stuff)
``` bash
make -j7
sudo make install
```

### 3. Install eigen
- cd to the empty 'DiscVisionDeluxe/dvd_DvisEst/lib/eigen' directory
- get rid of placeholder file
``` bash
rm *
```
- clone repo
``` bash
git clone https://github.com/AprilRobotics/apriltag.git
```
- build with cmake
``` bash
mkdir build; cd build
cmake ..
```
- make and install (change jN to your 'number fo processors-1' if you still want to do other stuff)
``` bash
make -j7
sudo make install
```

### 4. Install the FLIR Spinnaker SDK
- Grab the SDK from https://www.flir.ca/products/spinnaker-sdk/
- Follow the instructions in the README 
(make sure to do the permanent steps in the 'USB RELATED NOTES' section)
(don't bother with the 'GIGE CAMERA SETUP' stuff since we are using the USB3 version)
- At this point you can try using the 'SpinView' application to check if your camera is working
You should see 'SuperSpeed' USB under the streaming parameters (2nd high-level accordion) if your USB port supports PCIE2.1 (required)

### 5. Build dvd_DvisEst using CMake
``` bash
cd build/
cmake ..
make
```
(These commands are in the 'DiscVisionDeluxe/dvd_DvisEst/dothemake' script for convenience)

### 6. Run dvd_DvisEst!
``` bash
../bin/dvd_DvisEst -h
```
e.g.
``` bash
../bin/dvd_DvisEst -d -chime -gt
```



## Windows:

### 1. Install OpenCV
- you can just the binaries distributed here: 
  https://docs.opencv.org/master/d3/d52/tutorial_windows_install.html
  http://sourceforge.net/projects/opencvlibrary/files/opencv-win/
- Use the installer to install to C:/opencv
- Add the following entry to your environment variables"
    OpenCV_DIR C:/opencv/build/x64/vc15/lib
- Edit the system PATH, and add the entry:
    %OPENCV_DIR%../bin

### 2. Install CMAKE
cmake.org/download/

### 3. Install MSYS2 (and MINGW64 through it) (THIS NEEDS TO BE CHANGED TO THE VS COMPILER)
- grab the installer from https://www.msys2.org/
- in mysy2 prompt:
``` bash
pacman -Syu a couple times (close and re-open shell when prompted, then run it again)
pacman -S mingw-w64-x86_64-toolchain
pacman -S mingw-w64-x86_64-python-numpy
pacman -S mingw-w64-x86_64-cmake
```

### 3. Install apriltags (On WINDOWS, wooooo) (THIS NEEDS TO BE CHANGED TO THE VS COMPILER)
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

### 4. Build the dvd_DvisEst test app (THIS NEEDS TO BE CHANGED TO THE VS COMPILER)
- cd to dvd_DvisEst in your git checkout
``` bash
mkdir build
cd build
cmake -G "MinGW Makefiles" ..
mingw32-make.exe
```
- copy the libapriltag.so file to your .exe bin dir (cp C:\apriltag\libapriltag.so DiscVisionDeluxe\bin\)

### 5. Run dvd_DvisEst.exe! woot!


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
WORK IN PROGRESS!

