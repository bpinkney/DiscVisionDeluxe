cmake_minimum_required(VERSION 3.1)

# Add a bin directory at the top level for this project (ooh, special)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../bin)
set(THREADS_PREFER_PTHREAD_FLAG ON)

#define project
project( dvd_DwinTest )

# add package deps
find_package( Threads REQUIRED)
if (UNIX)
  # this only works on linux, for windows, we'll set the INC and LIB directly
  find_package( OpenCV REQUIRED )
  find_package( apriltag REQUIRED )
endif (UNIX)

if (UNIX)

  # find apriltag inc and lib stuff
  find_path(APRILTAG_INC NAMES apriltag.h PATH_SUFFIXES apriltag)
  find_library(APRILTAG_LIB apriltag)

endif (UNIX)
if (WIN32)

  # this path is set explicitly here on windows (so change it to your checkout location for apriltag)
  set(APRILTAG_INC "C:/apriltag")
  set(APRILTAG_LIB "C:/apriltag/libapriltag.so")

  # check external deps with debug messages
  message(INFO "APRILTAG_INC = ${APRILTAG_INC}" )
  message(INFO "APRILTAG_LIB = ${APRILTAG_LIB}" )

  # opencv stuff (built with mingw) (this still doesn't work, gah)
  set(OpenCV_INC "C:/opencv/sources/modules/core/include" "C:/opencv/sources/modules/core/include" "C:/opencv/sources/modules/imgproc/include" "C:/opencv/sources/modules/calib3d/include" "C:/opencv/sources/modules/features2d/include" "C:/opencv/sources/modules/flann/include" "C:/opencv/sources/modules/imgcodecs/include" "C:/opencv/sources/modules/videoio/include" "C:/opencv/sources/modules/highgui/include" "C:/opencv/build")

  # check external deps with debug messages
  message(INFO "OpenCV_INC = ${OpenCV_INC}" )
  message(INFO "OpenCV_LIB = ${OpenCV_LIB}" )

  # spinnaker stuff
  #set(SPINNAKER_LIB "C:/Program Files/Point Grey Research/Spinnaker/lib64/vs2015/Spinnaker_v140.lib")

endif (WIN32)

# add project executable (must be done before local includes)
add_executable( dvd_DwinTest src/main.cpp )

#Windows includes
if (WIN32)
 #target_include_directories(dvd_DwinTest PRIVATE "C:/Program Files/Point Grey Research/Spinnaker/include")
 # fix for janky windows pthreads (shame!)
 #add_compile_definitions(HAVE_STRUCT_TIMESPEC)
endif (WIN32)

#Linux includes
if (UNIX)
  # remember that Spinnaker only needs this include!
  target_include_directories(dvd_DwinTest PRIVATE "/usr/include/spinnaker/")  
endif (UNIX)

# add project local include files
target_include_directories(dvd_DwinTest PUBLIC "../common/inc/")
target_include_directories(dvd_DwinTest PUBLIC inc)
if (WIN32)
target_include_directories( dvd_DwinTest PRIVATE ${OpenCV_INC} )
endif (WIN32)
target_include_directories( dvd_DwinTest PRIVATE ${APRILTAG_INC} )

# add project local filenames
target_sources(dvd_DwinTest
    PRIVATE
      src/dvd_DvisEst_apriltag.cpp
      src/dvd_DvisEst_image_capture.cpp
      src/dvd_DvisEst_image_processing.cpp
)

# link libs
target_link_libraries( dvd_DwinTest ${OpenCV_LIBS} )
target_link_libraries( dvd_DwinTest ${APRILTAG_LIB} )
target_link_libraries( dvd_DwinTest Threads::Threads )
if (WIN32)
#target_link_libraries( dvd_DwinTest ${SPINNAKER_LIB} )
endif (WIN32)
if (UNIX)
target_link_libraries( dvd_DwinTest Spinnaker )
endif (UNIX)