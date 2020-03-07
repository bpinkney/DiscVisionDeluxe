CC = gcc
CXX = g++

CPPFLAGS = -I../../lib/matplotlib-cpp/ -I../../lib/opencv/include/opencv/ -I../../lib/opencv/modules/imgcodecs/include/opencv2/ -I../../lib/opencv/modules/core/include/opencv2/ -I../../lib/opencv/include/opencv2/ -I../../lib/opencv/modules/highgui/include/opencv2/ -I../../lib/apriltag/ `pkg-config --libs opencv` `pkg-config --libs apriltag`
CXXFLAGS = -Wall -O4 -std=c++11 -I/usr/include/python2.7 -lpython2.7
LDFLAGS = -lpthread -lm -g
VPATH=src

TARGETS := camera_calibration_from_jpegs

.PHONY: all
all: camera_calibration_from_jpegs

camera_calibration_from_jpegs: camera_calibration_from_jpegs.o 
		@echo "   [$@]"
		@$(CXX) -o $@ $^ $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS)

.PHONY: clean
clean:
		@rm -rf *.o $(TARGETS)