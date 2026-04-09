cmake_minimum_required(VERSION 3.1) # 3.15 needed for nuGet pthreads on windows

# Add a bin directory at the top level for this project (ooh, special)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../bin)
set(THREADS_PREFER_PTHREAD_FLAG ON)

# set windows locations for opencv, apriltags
if (WIN32)
  set(APRILTAG_GIT_DIR "C:/DiscVisionDeluxe/dvd_DvisEst/lib/apriltag")
  set(OPENCV_GIT_DIR "C:/DiscVisionDeluxe/dvd_DvisEst/lib/opencv")
endif (WIN32)

#define project
project( dvd_DvisEst )

# add package deps
find_package( Threads REQUIRED)
if (UNIX)
  # this only works on linux, for windows, we'll set the INC and LIB directly
  find_package( OpenCV REQUIRED )
  find_package( apriltag REQUIRED )
endif (UNIX)

if(WIN32)

  set(OpenCV_DIR "${OPENCV_GIT_DIR}/build")

endif (WIN32)

if (UNIX)

  # find apriltag inc and lib stuff
  find_path(APRILTAG_INC NAMES apriltag.h PATH_SUFFIXES apriltag)
  find_library(APRILTAG_LIB apriltag)

endif (UNIX)
if (WIN32)

  # this path is set explicitly here on windows (so change it to your checkout location for apriltag)
  set(APRILTAG_INC "${APRILTAG_GIT_DIR}")
  set(APRILTAG_LIB "${APRILTAG_GIT_DIR}/build/lib/Release/apriltag.lib")

  # check external deps with debug messages
  message(INFO "APRILTAG_INC = ${APRILTAG_INC}" )
  message(INFO "APRILTAG_LIB = ${APRILTAG_LIB}" )

  # gosh, is there a good way to do multiple lines in a CMakeLists.txt?
  set(OpenCV_INC "${OPENCV_GIT_DIR}/modules/core/include" "${OPENCV_GIT_DIR}/modules/imgproc/include" "${OPENCV_GIT_DIR}/modules/calib3d/include" "${OPENCV_GIT_DIR}/modules/features2d/include" "${OPENCV_GIT_DIR}/modules/flann/include" "${OPENCV_GIT_DIR}/modules/imgcodecs/include" "${OPENCV_GIT_DIR}/modules/videoio/include" "${OPENCV_GIT_DIR}/modules/highgui/include" "${OPENCV_GIT_DIR}/build" "${OPENCV_GIT_DIR}/build/include" "${OPENCV_GIT_DIR}/build/include/opencv" "${OPENCV_GIT_DIR}/build/include/opencv/opencv2")
  set(OpenCV_LIB "${OPENCV_GIT_DIR}/build/lib/Release/opencv_core440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_imgproc440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_calib3d440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_features2d440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_flann440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_imgcodecs440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_videoio440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_highgui440.lib")
  set(OpenCV_DLL "${OPENCV_GIT_DIR}/build/bin/Release/opencv_core440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_imgproc440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_calib3d440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_features2d440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_flann440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_imgcodecs440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_videoio440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_highgui440.dll")
  
  # check external deps with debug messages
  message(INFO "OpenCV_INC = ${OpenCV_INC}")
  message(INFO "OpenCV_LIB = ${OpenCV_LIB}")
  message(INFO "OpenCV_DLL = ${OpenCV_DLL}")

  # spinnaker stuff
  set(SPINNAKER_LIB "C:/Program Files/FLIR Systems/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")

endif (WIN32)

# add project executable (must be done before local includes)
add_executable( dvd_DvisEst src/main.cpp )

#Windows includes
if (WIN32)
  #pthread stuff for windows
  target_include_directories(${PROJECT_NAME} PUBLIC "C:/DiscVisionDeluxe/dvd_DvisEst/lib/winlibpthread64")

  set(PTHREAD_LIBRARIES "C:/DiscVisionDeluxe/dvd_DvisEst/lib/winlibpthread64/libpthread.lib")
  #SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-allow-multiple-definition ")

  target_link_libraries(${PROJECT_NAME} ${PTHREAD_LIBRARIES} winmm)
  #end pthread stuff for windows

  target_include_directories(dvd_DvisEst PRIVATE "C:/Program Files/FLIR Systems/Spinnaker/include")
  # fix for janky windows pthreads (shame!)
  add_compile_definitions(HAVE_STRUCT_TIMESPEC)

  # use latest c++
  target_compile_features(dvd_DvisEst PRIVATE cxx_std_17)
  target_compile_options(dvd_DvisEst PRIVATE "/W4")
endif (WIN32)

#Linux includes
if (UNIX)
  # remember that Spinnaker only needs this include!
  target_include_directories(dvd_DvisEst PRIVATE "/usr/include/spinnaker/") 
  # new location
  target_include_directories(dvd_DvisEst PRIVATE "/opt/spinnaker/include/") 
endif (UNIX)

# add project local include files
target_include_directories(dvd_DvisEst PUBLIC "../common/inc/")
target_include_directories(dvd_DvisEst PUBLIC inc)
if (WIN32)
target_include_directories( dvd_DvisEst PRIVATE ${OpenCV_INC} )
endif (WIN32)
target_include_directories( dvd_DvisEst PRIVATE ${APRILTAG_INC} )

# add project local filenames
target_sources(dvd_DvisEst
    PRIVATE
      src/dvd_DvisEst_apriltag.cpp
      src/dvd_DvisEst_image_capture.cpp
      src/dvd_DvisEst_image_processing.cpp
      src/dvd_DvisEst_estimate.cpp
)

# link libs
if (UNIX)
# this is opencv's built in LIBS path, which works on linux
target_link_libraries( dvd_DvisEst ${OpenCV_LIBS} )
endif (UNIX)
if (WIN32)
# this is the manual path we must build for Windows (above)
target_link_libraries( dvd_DvisEst ${OpenCV_LIB} )
# we also need to copy .dll files (gosh Windows, I mean really)
file(COPY ${OpenCV_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
file(COPY "C:/DiscVisionDeluxe/dvd_DvisEst/lib/winlibpthread64/libpthread.dll" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
endif (WIN32)
target_link_libraries( dvd_DvisEst ${APRILTAG_LIB} )
target_link_libraries( dvd_DvisEst Threads::Threads )
if (WIN32)
target_link_libraries( dvd_DvisEst ${SPINNAKER_LIB} )
endif (WIN32)
if (UNIX)
#target_link_libraries( dvd_DvisEst Spinnaker )
target_link_libraries( dvd_DvisEst "/opt/spinnaker/lib/libSpinnaker.so" )
endif (UNIX)