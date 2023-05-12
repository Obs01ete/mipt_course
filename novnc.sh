docker run -p 6080:80 -v /dev/shm:/dev/shm -v ~/Desktop/p/phystech/lidar/:/ws -v ~/Desktop/p/phystech/lidar/kitti/:/kitti -e RESOLUTION=1600x900 dmitriikhizbullin/lidar_course:latest
