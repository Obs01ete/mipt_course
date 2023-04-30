#pragma once

#include <deque>
#include <vector>

#include <pcl/kdtree/kdtree.h>


namespace lidar_course {


template <typename PointT>
auto dbscan_segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr) {
    // Define some DBSCAN specific constants
    const float epsilon = 1.0f;
    const int minPts = 10;

    enum point_label : int {
        UNDEFINED = -1,
        NOISE = 0
    };

    int cluster_idx = 0;
    
    pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_labeled_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
    
    // Collect all points and label them with "UNDEFINED"
    for(const auto& input_point : *input_cloud_ptr) {
        cloud_labeled_ptr->push_back(pcl::PointXYZL(input_point.x, input_point.y, input_point.z, point_label::UNDEFINED));
    }

    // Create KD-tree and initialize it with given point cloud
    pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
    kd_tree.setInputCloud(cloud_labeled_ptr);

    // Main loop over all points in cloud
    for(auto& cloud_point : *cloud_labeled_ptr) {
        if(cloud_point.label != point_label::UNDEFINED)
            continue;

        // Get all point's heighbors within epsilon distance
        std::vector<pcl::index_t> neighbors;
        std::vector<float> squared_distances;
        kd_tree.radiusSearch(cloud_point, epsilon, neighbors, squared_distances);

        // And if there is not enough points mark current one as "NOISE"
        if(neighbors.size() < minPts) {
            cloud_point.label = point_label::NOISE;
            continue;
        }

        // If there is enough neighbors, then current point is a core one,
        // so mark it with new cluster index. Start indexing from 1 because
        // index = 0 is reserved for "not clustered" points
        ++cluster_idx;
        cloud_point.label = cluster_idx;

        // deque has convenient constructor
        std::deque<pcl::index_t> points_to_label(neighbors.begin(), neighbors.end());

        // Process whole cluster with queue
        while(!points_to_label.empty()) {
            // Get current neighbor
            const pcl::index_t neigh_point_idx = points_to_label.front();
            points_to_label.pop_front();

            auto& curr_point = cloud_labeled_ptr->points[neigh_point_idx];

            // Promote "NOISE" point to current cluster
            if(curr_point.label == point_label::NOISE)
                curr_point.label = cluster_idx;

            if(curr_point.label != point_label::UNDEFINED)
                continue;

            curr_point.label = cluster_idx;

            // Get neighbors of the current neighbor
            neighbors.clear();
            squared_distances.clear();
            kd_tree.radiusSearch(curr_point, epsilon, neighbors, squared_distances);

            // If current neighbor is core point, then add its neighborhood to queue
            if(neighbors.size() >= minPts) {
                for(const auto& neigh : neighbors)
                    points_to_label.push_back(neigh);
            }
        }
    }
    return cloud_labeled_ptr;
}


} // namespace lidar_course
