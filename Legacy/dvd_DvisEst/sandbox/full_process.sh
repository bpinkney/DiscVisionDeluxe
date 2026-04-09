IMG_DIR="$1"

[ -d "$IMG_DIR" ]
if [ -d "$IMG_DIR" ]
then
  echo "Directory $IMG_DIR exists."
else
  echo "Directory $IMG_DIR does not exist; exiting..."
  exit
fi

IMG_DIR_ABS=`realpath $IMG_DIR`

echo "Start process using $IMG_DIR_ABS"

#undistort
echo "Undistort"
rm undistort_images/distorted_imgs/*
rm undistort_images/undistorted_imgs/*
cp -r $IMG_DIR_ABS/* undistort_images/distorted_imgs/

cd undistort_images/
make
./undistort_images
cd ..

#apriltag detect
echo "Detect Apriltags"
rm apriltag_cpu_test/undistorted_imgs/*
cp undistort_images/undistorted_imgs/* apriltag_cpu_test/undistorted_imgs/

cd apriltag_cpu_test
make
rm overlay_images/*
./apriltag_cpu_test
cd ..

#clean up trash
rm -rf ~/.local/share/Trash/*

#copy matlab out over
cp apriltag_cpu_test/csvlog.csv $IMG_DIR_ABS
feh apriltag_cpu_test/overlay_images/ &

