cmake_minimum_required(VERSION 2.8)

# Add a bin directory at the top level for this project (ooh, special)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}../../../bin)

project( dvd_DvisEst )

find_package( OpenCV REQUIRED )
find_package( apriltag REQUIRED )

#Windows includes
if (WIN32)
    #do something
endif (WIN32)

#Linux includes
if (UNIX)
  #set(UNIX_APRILTAGS_INC /usr/local/include/apriltag/)
  #target_include_directories(test PRIVATE ${UNIX_APRILTAGS_INC})
endif (UNIX)

add_executable( dvd_DvisEst src/main.cpp )

target_link_libraries( dvd_DvisEst ${OpenCV_LIBS} )