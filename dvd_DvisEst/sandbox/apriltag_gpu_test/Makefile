CC = gcc
CXX = g++

CPPFLAGS = $(shell pkg-config --cflags opencv4)
# why does this choke on -L/usr/local/lib?
LIB += -lopencv_gapi -lopencv_stitching -lopencv_video -lopencv_highgui -lopencv_ml -lopencv_videoio -lopencv_photo -lopencv_dnn -lopencv_imgcodecs -lopencv_objdetect -lopencv_calib3d -lopencv_features2d -lopencv_flann -lopencv_imgproc -lopencv_core
#opencv stuff
#LIB += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_videoio 
#LIB += -L/usr/local/lib -lopencv_core -lopencv_imgcodecs -lopencv_highgui

#apriltag stuff
ISSAC_APRILTAG_INC = -I../../lib/apriltag_gpu/nvapriltags
CPPFLAGS += $(ISSAC_APRILTAG_INC)
ISSAC_APRILTAG_LIB = -L../../lib/apriltag_gpu/lib_x64 -lapril_tagging
LIB += $(ISSAC_APRILTAG_LIB)

CXXFLAGS = -Wall -O4 -std=c++11 -I/usr/include/python2.7 -lpython2.7
LDFLAGS = -lpthread -g
VPATH=src

TARGETS := apriltag_gpu_test

.PHONY: all
all: apriltag_gpu_test

apriltag_gpu_test: apriltag_gpu_test.o ${LIB}
		@echo "   [$@]"
		@$(CXX) -o $@ $^ $(CXXFLAGS) $(CPPFLAGS) $(LDFLAGS) 

.PHONY: clean
clean:
		@rm -rf *.o $(TARGETS)