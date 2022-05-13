#ifndef DBSCAN_H
#define DBSCAN_H

#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/segmentation/cpc_segmentation.h>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_cource {

    template <typename PointT>
    float CalcNormalize(PointT point) 
    {
        float normilize = point.getVector3fMap().norm();
        normilize = (normilize < 1.5) ? 0.0 : (normilize - 1.5) * 0.105;
        if (normilize > 4.0)
            normilize = 4.0;
            return normilize;
    }

    template <typename PointT>
    auto Dbscan(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
    {
        enum {
            UNDEFINED = 1,
            NOISE,
            FIRST_CLUSTER
        };

        const float Eps = 0.5;
        const size_t MinNumNeighbors = 10;

        pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
        pcl::copyPointCloud(*input_cloud_ptr, *labeled_point_cloud);
        for(auto& point : *labeled_point_cloud)
            point.label = UNDEFINED;

        pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
        kdtree.setInputCloud(labeled_point_cloud);

        // Addition types for search neighbor points
        std::queue<uint32_t> neighbor_queue;
        std::vector<int> point_neighbors_idx;
        std::vector<float> point_neighbors_distance;
        uint32_t label_cluster = FIRST_CLUSTER;

        for (auto &point : *labeled_point_cloud) {
            if (point.label != UNDEFINED) {
                continue;
            }
            int num_neighbors = kdtree.radiusSearch(point, Eps + CalcNormalize(point), point_neighbors_idx, point_neighbors_distance);
            if (num_neighbors < MinNumNeighbors) {
                point.label = NOISE;
                continue;
            }

            point.label = label_cluster;
            for (auto id: point_neighbors_idx) {
                neighbor_queue.push(id);
            }

            while (!neighbor_queue.empty()) {
                uint32_t id = neighbor_queue.front();
                neighbor_queue.pop();
                auto& temp_point = (*labeled_point_cloud)[id];
                if (temp_point.label == NOISE) {
                    temp_point.label = label_cluster;
                    continue;
                } else if (temp_point.label != UNDEFINED) {
                    continue;
                }
                temp_point.label = label_cluster;
                point_neighbors_idx.clear();
                point_neighbors_distance.clear();                
            
                int num_neighbors = kdtree.radiusSearch(point, Eps + CalcNormalize(temp_point), point_neighbors_idx, point_neighbors_distance);
                if (num_neighbors >= MinNumNeighbors) {
                    for (auto id: point_neighbors_idx) {
                        neighbor_queue.push(id);
                    }
                }
            }
            label_cluster++;
        }
        return labeled_point_cloud;
    }

}


#endif // DBSCAN_H
