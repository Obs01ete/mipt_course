cd /ws/lidar_course/
mkdir build
cd build
cmake ..
make -j8

./process-sequence /kitti/tracking/training/ 0000
