mkdir build 
cd build
cmake .. || { echo 'cmake failed' ; exit 1; }
make -j8 || { echo 'make failed' ; exit 1; }

./process-sequence /kitti/tracking/training/ 0000
