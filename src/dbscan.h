#include <Eigen/Geometry>
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/kdtree/kdtree.h>

#include <cstdlib>
#include <vector>
#include <deque>

namespace lidar_course {

template <typename PointT>
auto dbscan_segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr) {
    using PointLT = pcl::PointXYZL;
    const size_t UNDEFINED = 0;
    const size_t NOIZE = 1;
    const float eps = 1.5f;
    const size_t MinN = 5;

    pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud = std::make_shared<pcl::PointCloud<PointLT>>();
    pcl::copyPointCloud(*input_cloud_ptr, *labeled_cloud);
    for(PointLT& point : *labeled_cloud) point.label = UNDEFINED;

    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    kdtree.setInputCloud(input_cloud_ptr);

    size_t segnum = NOIZE;
    for(PointLT& point : *labeled_cloud) {
        if(point.label != UNDEFINED) continue;

        std::vector<pcl::index_t> Neighbors;
        std::vector<float> Dist;

        kdtree.radiusSearch(pcl::PointXYZ{point.x, point.y, point.z}, eps, Neighbors, Dist);
        
        if(Neighbors.size() < MinN) {
            point.label = NOIZE;
            continue;
        }

        segnum++;
        point.label = segnum;

        std::deque<PointLT*> point_queue;
        for(const auto& index : Neighbors) point_queue.push_back(&(labeled_cloud->points[index]));

        while(!point_queue.empty()) {
            PointLT* Npoint = *(point_queue.begin());
            point_queue.pop_front();

            if(Npoint->label == NOIZE) Npoint->label = segnum;
            if(Npoint->label != UNDEFINED) continue;
            Npoint->label = segnum;

            Neighbors.clear();
            Dist.clear();
            kdtree.radiusSearch(pcl::PointXYZ{Npoint->x, Npoint->y, Npoint->z}, eps, Neighbors, Dist);

            if(Neighbors.size() >= MinN) {
                for (const auto& index : Neighbors){
                    PointLT* NNpoint = &(labeled_cloud->points[index]);
                    if(Npoint->label <= NOIZE) point_queue.push_back(NNpoint);
                }
            }
            
        }
    }

    for(auto& point : *labeled_cloud) {
        if(point.label > 0) point.label--;
    }

    return labeled_cloud;
}
} //namespace lidar_course