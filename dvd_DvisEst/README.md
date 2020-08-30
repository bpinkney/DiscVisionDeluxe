# DiscVisionDeluxe Vision Capture and Initial State Estimator

Sequence of dvd_visEst routine:
- Observe high-fps video feed (Thread 1)
- Run AprilTag detector on each frame (Threads 2-N)
- Upon detection, use single/multiple tag detections to produce a single position and orientation measurement (Threads 2-N)
- Use pos/orient measurements to produce a KF-style disc state estimate (Thread N+1, although this runs after frame collection is complete technically)
- Update state estimate throughout disc flight to obtain confident states and derivatives (Thread N+1)
- Serve finalized initial state to dvd_DfisX and dvd_DgrafX (Main Thread)

## Setting up a Linux build env:

You can just cmake install opencv, apriltag, and eigen; easy!

### 0. Required Libs
``` bash
sudo apt-get install gcc-5 g++-5
sudo apt-get install cmake
sudo pip2 install numpy-stl
sudo apt-get install libgtkmm-2.4-dev
sudo apt-get install git
cd ~/
git clone https://github.com/bpinkney/DiscVisionDeluxe.git
cd DiscVisionDeluxe
```

### 1. Install openCV
- 'cd' to the 'DiscVisionDeluxe/dvd_DvisEst/lib/' directory
- get rid of placeholder file and folder
``` bash
rm -rf opencv
```
- clone repo
``` bash
git clone https://github.com/opencv/opencv.git
cd opencv
```
- build with cmake
``` bash
mkdir build; cd build
cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=YES -D CMAKE_INSTALL_PREFIX=/usr/local ..
```
- make and install (Set jN to as many processors as you can spare)
``` bash
make -j7
sudo make install
```

### 2. Install apriltag
- 'cd' to the 'DiscVisionDeluxe/dvd_DvisEst/lib/' directory
- get rid of placeholder file and folder
``` bash
rm -rf apriltag
```
- clone repo
``` bash
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
```
- build with cmake
``` bash
mkdir build; cd build
cmake ..
```
- make and install (Set jN to as many processors as you can spare)
``` bash
make -j7
sudo make install
```

### 3. Install eigen
- 'cd' to the 'DiscVisionDeluxe/dvd_DvisEst/lib/' directory
- get rid of placeholder file and folder
``` bash
rm -rf eigen
```
- clone repo
``` bash
git clone https://gitlab.com/libeigen/eigen.git
cd eigen
```
- build with cmake
``` bash
mkdir build; cd build
cmake ..
```
- make and install (Set jN to as many processors as you can spare)
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
You should see 'SuperSpeed' USB under the streaming parameters (2nd high-level accordion) if your USB port supports PCIE2.1 (this is required, sometimes machines have a mix of PCIE2.0 and PCIE2.1 USB ports, so try them all and label them)

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

#or for test images
../bin/dvd_DvisEst -gp="../bin/logs/test_throw_1/1_2020-05-21_09-01-37_ground_plane.yaml" -fi="../bin/logs/test_throw_1/images/" -cc="../bin/camera_calibrations/19508898.yaml"
```



## Setting up a Windows build env (This actually works now!):

Note: We are going to build everything here 64 bit!
"Visual Studio 16 2019" is 64-bit by default on 64 bit hosts, so you will need to change some of the paths here for 32 bit if you want it

### 0. Install Various Crap
- Install git from https://git-scm.com/download/win
- Install the FLIR Spinnaker SDK (including the VS support)
- Install Visual Studio 2019 Community or onward with C++ and Cmake (3.1.5) support

### 1. Install openCV
- 'cd' to the 'DiscVisionDeluxe/dvd_DvisEst/lib/' directory
- get rid of placeholder file and folder
``` bash
rm -rf opencv
```
- clone repo
``` bash
git clone https://github.com/opencv/opencv.git
cd opencv
```
- build with cmake (for some reason you need to force cmake to pick up the x64 libs here?)
``` bash
mkdir build; cd build
cmake  -G "Visual Studio 16 2019" -A x64 -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=YES -D CMAKE_INSTALL_PREFIX=/usr/local ..
```
- make and install
``` bash
cmake --build . --config Release
```
- verify the lib and dll files exists with
``` bash
dir /s *lib
dir /s *dll
```

### 2. Install apriltag
- 'cd' to the 'DiscVisionDeluxe/dvd_DvisEst/lib/' directory
- get rid of placeholder file and folder
``` bash
rm -rf apriltag
```
- clone repo
``` bash
git clone https://github.com/AprilRobotics/apriltag.git
cd apriltag
```
- build with cmake
This part is a real pain, but oh well

Option 1: (easier) Grab libpthread stuff from the DiscVisionDeluxe repo

Option 2: Create a new visual studio project, and use the nuget package manager to install 'pthreads', then grab the libpthread.so and include headers and move them somewhere you can reference for building apriltags (and eventually dvd_DvisEst)

- Open your copied pthread.h and add 
``` c
#define HAVE_STRUCT_TIMESPEC
```
- Open the apriltag/CMakeLists.txt file and replace the if(MSVC)andelse block with the following:
(replace with the pthread dir you made above if not using the checked in files)
```
    target_include_directories(${PROJECT_NAME} PUBLIC "C:/DiscVisionDeluxe/dvd_DvisEst/lib/winlibpthread64")

    set(PTHREAD_LIBRARIES "C:/DiscVisionDeluxe/dvd_DvisEst/lib/winlibpthread64/libpthread.lib")
    SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-allow-multiple-definition ")

    target_link_libraries(${PROJECT_NAME} ${PTHREAD_LIBRARIES} winmm)
```
- Change the add_library CMake command from:
```
add_library(${PROJECT_NAME} SHARED ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})
```
to
```
add_library(${PROJECT_NAME} STATIC ${APRILTAG_SRCS} ${COMMON_SRC} ${TAG_FILES})
```
- remove things which reference unistd.c in the examples folder, but the leave the file intact so it still builds a .lib
- I just removed the unistd.h include in apriltag_demo.c, and got rid of everything between the tag family constructors/detsructors

- Now actually make
``` bash
mkdir build; cd build
cmake  -G "Visual Studio 16 2019" -A x64 ..
```
- make and install
``` bash
cmake --build . --config Release 
```
- verify the lib and dll files exists with
``` bash
dir /s *lib
dir /s *dll
```

### 5. Build dvd_DvisEst using CMake
``` bash
mkdir build; cd build/
rm -rf *
cmake ..
cmake --build . --config Release
cd ..
```
(These commands are in the 'DiscVisionDeluxe/dvd_DvisEst/dothemakewin.bat' script for convenience)

### 6. Run dvd_DvisEst!
``` bash
..\bin\Release\dvd_DvisEst -h
```
e.g.
``` bash
..\bin\Release\dvd_DvisEst -d
```