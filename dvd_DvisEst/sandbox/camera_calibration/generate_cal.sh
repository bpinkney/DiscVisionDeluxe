#!/bin/bash
#  int numCornersHor = 7;
#  int numCornersVer = 9;
make;
./random_cal_images.sh
./imagelist_creator image_list2.xml cal_imgs/*.jpg
./camera_calibration_from_jpegs -w=9 -h=7 -pt=chessboard -s=0.01965 -o=camera_cal800_16mmlens.yaml -op -oe image_list2.xml

