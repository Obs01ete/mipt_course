#include <iostream> 
#include "dbscan.h"

int main() {
    auto cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
    
    //! First claster
    cloud_ptr->push_back(pcl::PointXYZ(0, 0, 0));
    cloud_ptr->push_back(pcl::PointXYZ(0, 0.1, 0));
    cloud_ptr->push_back(pcl::PointXYZ(0, 0, 0.1));
    cloud_ptr->push_back(pcl::PointXYZ(0, 0, 0.2));
    cloud_ptr->push_back(pcl::PointXYZ(0, 0.2, 0));
    cloud_ptr->push_back(pcl::PointXYZ(0.1, 0, 0));
    cloud_ptr->push_back(pcl::PointXYZ(0.1, 0, 0.3));
    cloud_ptr->push_back(pcl::PointXYZ(0.2, 0.1, 0.3));
    cloud_ptr->push_back(pcl::PointXYZ(0.2, 0.1, 0.3));
    cloud_ptr->push_back(pcl::PointXYZ(0.3, 0.3, 0.4));
    cloud_ptr->push_back(pcl::PointXYZ(0.3, 0.3, 0.5));
    cloud_ptr->push_back(pcl::PointXYZ(0.3, 0.3, 0.6));
    cloud_ptr->push_back(pcl::PointXYZ(0.3, 0.3, 0.7));

    //! Second claster
    cloud_ptr->push_back(pcl::PointXYZ(2.3, 2.5, 2.5));
    cloud_ptr->push_back(pcl::PointXYZ(2.2, 2.6, 2.5));
    cloud_ptr->push_back(pcl::PointXYZ(2.1, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.0, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.1, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.2, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.3, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.4, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.5, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.6, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.7, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.8, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.9, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(3.0, 2.5, 2.6));
    cloud_ptr->push_back(pcl::PointXYZ(2.5, 2.6, 2.6));

    //! Third claster
    cloud_ptr->push_back(pcl::PointXYZ(30.0, 30.0, 30.0));
    cloud_ptr->push_back(pcl::PointXYZ(30.1, 30.3, 30.5));
    cloud_ptr->push_back(pcl::PointXYZ(30.2, 30.3, 30.4));
    cloud_ptr->push_back(pcl::PointXYZ(30.2, 30.3, 30.6));
    cloud_ptr->push_back(pcl::PointXYZ(30.3, 30.3, 30.7));
    cloud_ptr->push_back(pcl::PointXYZ(30.3, 30.3, 30.8));
    cloud_ptr->push_back(pcl::PointXYZ(30.3, 30.3, 30.9));
    cloud_ptr->push_back(pcl::PointXYZ(30.3, 30.5, 30.9));
    cloud_ptr->push_back(pcl::PointXYZ(30.4, 30.5, 30.9));
    cloud_ptr->push_back(pcl::PointXYZ(30.4, 30.6, 30.9));
    cloud_ptr->push_back(pcl::PointXYZ(30.5, 30.6, 30.9));
    cloud_ptr->push_back(pcl::PointXYZ(30.5, 30.6, 31.0));
    
    //! Noise
    cloud_ptr->push_back(pcl::PointXYZ(0, 1, 10.0));
    cloud_ptr->push_back(pcl::PointXYZ(1, 1.0, 0));
    cloud_ptr->push_back(pcl::PointXYZ(10.0, 10.0, 10.0));
    cloud_ptr->push_back(pcl::PointXYZ(0, 100, 10000));

    lidar_course::dbscan<pcl::PointXYZ> dbscan{cloud_ptr};
    auto dbscan_labeled_cloud_ptr = dbscan.run_segmentation();

    for (auto& point : *dbscan_labeled_cloud_ptr) {
        std::cout << "(" << point.x << ", " << point.y << ", " << point.z << "),  label=";
        
        if (point.label == 0)
            std::cout << "NOISE" << std::endl;
        else
            std::cout << point.label << std::endl;
    }
        
}