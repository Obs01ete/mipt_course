#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <deque>
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_course::dbscan {

constexpr const float radius = 1.0f;
constexpr const int   min_pts = 10;
constexpr const int   undefined = -1;
constexpr const int   noise = 0;
  
int cluster= 0;

using point_cloud     =     typename  pcl::PointCloud<pcl::PointXYZ>;
using labeled_point_cloud = typename  pcl::PointCloud<pcl::PointXYZL>;

labeled_point_cloud::Ptr dbscan_segmentation(point_cloud::Ptr input_cloud_ptr)
{
    labeled_point_cloud::Ptr labeled_cloud_ptr(new labeled_point_cloud());
    copyPointCloud(*input_cloud_ptr, *labeled_cloud_ptr);
    for (auto &&point: *labeled_cloud_ptr) {
        point.label = undefined;
    }

    pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
    kd_tree.setInputCloud(labeled_cloud_ptr);

    for (auto &&point: *labeled_cloud_ptr) {
        if (point.label != undefined) {
            continue;
        }

        std::vector<pcl::index_t> neighbours;
        std::vector<float> sqr_distances;
        auto num_neighbour_pts = kd_tree.radiusSearch(point, radius, neighbours, sqr_distances);
        if (num_neighbour_pts < min_pts) {
            point.label = noise;
            continue;
        }

        point.label = ++cluster;

        std::deque<pcl::index_t> queue_of_neighbours(neighbours.begin(), neighbours.end());

        while (!queue_of_neighbours.empty()) {
            auto neighbour_idx = queue_of_neighbours.front();
            queue_of_neighbours.pop_front();

            auto &neighbour_point = labeled_cloud_ptr->points[neighbour_idx];
            if (neighbour_point.label == undefined) {
                neighbour_point.label = cluster;
                num_neighbour_pts = kd_tree.radiusSearch(neighbour_point, radius, neighbours, sqr_distances);
                if (num_neighbour_pts >= min_pts) {
                    queue_of_neighbours.insert(queue_of_neighbours.end(), neighbours.begin(), neighbours.end());
                }
            } else if (neighbour_point.label == noise) {
                neighbour_point.label = cluster;
            }
        }
    }
    return labeled_cloud_ptr;
}
} // namespace lidar_course

#endif // DBSCAN_H