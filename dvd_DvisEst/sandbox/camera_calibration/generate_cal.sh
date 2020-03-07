#!/bin/bash
#  int numCornersHor = 7;
#  int numCornersVer = 9;
make;
./imagelist_creator image_list.xml cal_imgs/*.pgm
./camera_calibration_from_jpegs -w=7 -h=9 -pt=chessboard -s=0.020 -o=camera_cal.txt -op -oe image_list.xml

