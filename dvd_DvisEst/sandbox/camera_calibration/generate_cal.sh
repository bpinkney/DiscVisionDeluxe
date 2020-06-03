#!/bin/bash
#  int numCornersHor = 7;
#  int numCornersVer = 9;
make;
./random_cal_images.sh
./imagelist_creator image_list.xml cal_imgs/*.jpg
./camera_calibration_from_jpegs -o=camera_cal500_fisheye_0_n2.yaml

