cmake_minimum_required(VERSION 2.8)

# Add a bin directory at the top level for this project (ooh, special)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../bin)

#define project
project( dvd_DvisEst )

# add package deps
find_package( OpenCV REQUIRED )
find_package( apriltag REQUIRED )
#[[find_package(Spinnaker)
if (NOT SPINNAKER_FOUND)
  message(FATAL_ERROR "Spinnaker not found. Either turn off the `WITH_FLIR_CAMERA` option or specify the path to
    the Spinnaker includes and libs.")
endif (NOT SPINNAKER_FOUND)]]
#find_package( Spinnaker REQUIRED )

# find apriltag inc and lib stuff
find_path(APRILTAG_INC NAMES apriltag.h PATH_SUFFIXES apriltag)
find_library(APRILTAG_LIB apriltag)

# find spinnaker inc and lib stuff
#find_path(SPINNAKER_INC NAMES Spinnaker.h PATH_SUFFIXES spinnaker)
#find_library(SPINNAKER_LIB Spinnaker)

# add project executable (must be done before local includes)
add_executable( dvd_DvisEst src/main.cpp )

# check external deps with debug messages
#message(WARNING "APRILTAG_INC = ${APRILTAG_INC}" )
#message(WARNING "APRILTAG_LIB = ${APRILTAG_LIB}" )

#Windows includes
if (WIN32)
    #do something
endif (WIN32)

#Linux includes
if (UNIX)
  # remember that Spinnaker only needs this include!
  target_include_directories(dvd_DvisEst PRIVATE "/usr/include/spinnaker/")  
endif (UNIX)

# add project local include files
target_include_directories(dvd_DvisEst PUBLIC "../common/inc/")
target_include_directories(dvd_DvisEst PUBLIC inc)
#target_include_directories(dvd_DvisEst PUBLIC ${apriltag_INCLUDE_DIRS})
target_include_directories( dvd_DvisEst PRIVATE ${APRILTAG_INC} )

# add project local filenames
target_sources(dvd_DvisEst
    PRIVATE
      src/dvd_DvisEst_apriltag.cpp
      src/dvd_DvisEst_image_capture.cpp
      src/dvd_DvisEst_image_processing.cpp
)

# link libs
target_link_libraries( dvd_DvisEst ${OpenCV_LIBS} )
target_link_libraries( dvd_DvisEst ${APRILTAG_LIB} )
target_link_libraries( dvd_DvisEst Spinnaker )