cd /ws/lidar_course/
mkdir build
cd build
cmake ..
make -j2

./process-sequence /kitti/tracking/training/ 0000
