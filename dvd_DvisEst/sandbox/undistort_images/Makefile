CC = gcc
CXX = g++

CPPFLAGS = $(shell pkg-config --cflags opencv4)
# why does this choke on -L/usr/local/lib?
LIB += -lopencv_gapi -lopencv_stitching -lopencv_video -lopencv_highgui -lopencv_ml -lopencv_videoio -lopencv_photo -lopencv_dnn -lopencv_imgcodecs -lopencv_objdetect -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_imgproc -lopencv_core
#opencv stuff
#LIB += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio 
#LIB += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui
CXXFLAGS = -Wall -O4 -std=c++11
LDFLAGS = -lpthread -g
VPATH=src

TARGETS := undistort_images

.PHONY: all
all: undistort_images

undistort_images: undistort_images.o ${LIB}
		@echo "   [$@]"
		@$(CXX) -o $@ $^ $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) 

.PHONY: clean
clean:
		@rm -rf *.o $(TARGETS)