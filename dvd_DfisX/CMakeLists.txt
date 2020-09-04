cmake_minimum_required(VERSION 3.1) # 3.15 needed for nuGet pthreads on windows

# Add a bin directory at the top level for this project (ooh, special)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../bin)
#set(THREADS_PREFER_PTHREAD_FLAG ON)

# set windows locations for opencv, apriltags
if (WIN32)
  set(OPENCV_GIT_DIR "C:/DiscVisionDeluxe/dvd_DfisX/lib/opencv")
endif (WIN32)

#define project
project( dvd_DfisX )

# add package deps
find_package( Threads REQUIRED)
if (UNIX)
  # this only works on linux, for windows, we'll set the INC and LIB directly
  find_package( OpenCV REQUIRED )
  find_package (Eigen3 3.3 REQUIRED NO_MODULE)
endif (UNIX)

if(WIN32)

  set(OpenCV_DIR "${OPENCV_GIT_DIR}/build")

endif (WIN32)

if (WIN32)

  # gosh, is there a good way to do multiple lines in a CMakeLists.txt?
  set(OpenCV_INC "${OPENCV_GIT_DIR}/modules/core/include" "${OPENCV_GIT_DIR}/modules/imgproc/include" "${OPENCV_GIT_DIR}/modules/calib3d/include" "${OPENCV_GIT_DIR}/modules/features2d/include" "${OPENCV_GIT_DIR}/modules/flann/include" "${OPENCV_GIT_DIR}/modules/imgcodecs/include" "${OPENCV_GIT_DIR}/modules/videoio/include" "${OPENCV_GIT_DIR}/modules/highgui/include" "${OPENCV_GIT_DIR}/build" "${OPENCV_GIT_DIR}/build/include" "${OPENCV_GIT_DIR}/build/include/opencv" "${OPENCV_GIT_DIR}/build/include/opencv/opencv2")
  set(OpenCV_LIB "${OPENCV_GIT_DIR}/build/lib/Release/opencv_core440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_imgproc440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_calib3d440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_features2d440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_flann440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_imgcodecs440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_videoio440.lib" "${OPENCV_GIT_DIR}/build/lib/Release/opencv_highgui440.lib")
  set(OpenCV_DLL "${OPENCV_GIT_DIR}/build/bin/Release/opencv_core440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_imgproc440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_calib3d440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_features2d440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_flann440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_imgcodecs440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_videoio440.dll" "${OPENCV_GIT_DIR}/build/bin/Release/opencv_highgui440.dll")
  
  # check external deps with debug messages
  message(INFO "OpenCV_INC = ${OpenCV_INC}")
  message(INFO "OpenCV_LIB = ${OpenCV_LIB}")
  message(INFO "OpenCV_DLL = ${OpenCV_DLL}")

endif (WIN32)

# add project executable (must be done before local includes)
add_executable( dvd_DfisX main.cpp )

#Windows includes
if (WIN32)
  #pthread stuff for windows
  #target_include_directories(${PROJECT_NAME} PUBLIC "C:/DiscVisionDeluxe/dvd_DfisX/lib/winlibpthread64")

  #set(PTHREAD_LIBRARIES "C:/DiscVisionDeluxe/dvd_DfisX/lib/winlibpthread64/libpthread.lib")
  #SET(CMAKE_EXE_LINKER_FLAGS "-Wl,-allow-multiple-definition ")

  #target_link_libraries(${PROJECT_NAME} ${PTHREAD_LIBRARIES} winmm)
  #end pthread stuff for windows
  # fix for janky windows pthreads (shame!)
  #add_compile_definitions(HAVE_STRUCT_TIMESPEC)

  # use latest c++
  #target_compile_features(dvd_DfisX PRIVATE cxx_std_17)
  #target_compile_options(dvd_DfisX PRIVATE "/W4")
endif (WIN32)

# add project local include files
target_include_directories(dvd_DfisX PUBLIC "../common/inc/")
#target_include_directories(dvd_DfisX PUBLIC inc)
if (WIN32)
target_include_directories( dvd_DfisX PRIVATE ${OpenCV_INC} )
endif (WIN32)

# add project local filenames
target_sources(dvd_DfisX
    PRIVATE
      Dpropagate.cpp
      Dgyro.cpp
      Daero.cpp
      DfisX.cpp
)

# link libs
if (UNIX)
# this is opencv's built in LIBS path, which works on linux
target_link_libraries( dvd_DfisX ${OpenCV_LIBS} )
target_link_libraries( dvd_DfisX Eigen3::Eigen )
#target_link_libraries( dvd_DfisX ${Eigen_LIBS} )
endif (UNIX)
if (WIN32)
# this is the manual path we must build for Windows (above)
target_link_libraries( dvd_DfisX ${OpenCV_LIB} )
# we also need to copy .dll files (gosh Windows, I mean really)
file(COPY ${OpenCV_DLL} DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
#file(COPY "C:/DiscVisionDeluxe/dvd_DfisX/lib/winlibpthread64/libpthread.dll" DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Release)
endif (WIN32)
target_link_libraries( dvd_DfisX Threads::Threads )
