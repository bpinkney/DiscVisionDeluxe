#!/bin/bash
#  int numCornersHor = 7;
#  int numCornersVer = 9;
make;
./random_cal_images.sh
./imagelist_creator image_list2.xml cal_imgs/*.jpg
./camera_calibration_from_jpegs -w=7 -h=9 -pt=chessboard -s=0.0196 -o=camera_cal2000.yaml -op -oe image_list2.xml

