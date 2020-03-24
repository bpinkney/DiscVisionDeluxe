cmake_minimum_required(VERSION 2.8)

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../bin)

project( dvd_DvisEst )
find_package( OpenCV REQUIRED )
add_executable( dvd_DvisEst src/main.cpp )
target_link_libraries( dvd_DvisEst ${OpenCV_LIBS} )