CC = gcc
CXX = g++

CPPFLAGS = $(shell pkg-config --cflags opencv4)
CPPFLAGS += -I/usr/local/include/apriltag/
#CPPFLAGS += $(shell pkg-config --cflags apriltag)
# why does this choke on -L/usr/local/lib?
LIB = -lopencv_stitching -lopencv_video -lopencv_highgui -lopencv_ml -lopencv_videoio -lopencv_photo -lopencv_dnn -lopencv_imgcodecs -lopencv_objdetect -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_imgproc -lopencv_core
LIB += -lapriltag

# add disc stuff
CXXFLAGS += -I../../../common/inc

CXXFLAGS = -Wall -O4 -std=c++11
LDFLAGS = -lpthread -g
VPATH=src

TARGETS := apriltag_cpu_test

.PHONY: all
all: apriltag_cpu_test

apriltag_cpu_test: apriltag_cpu_test.o ${LIB}
		@echo "   [$@]"
		@$(CXX) -o $@ $^ $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) 

.PHONY: clean
clean:
		@rm -rf *.o $(TARGETS)