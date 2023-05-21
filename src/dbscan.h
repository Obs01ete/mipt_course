#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <deque>
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_course::dbscan {

// Notation and implementation algorithm were taken from wikipedia.org/wiki/DBSCAN

constexpr const float radius = 1.0f;  // radius for searching neighbours of the point
constexpr const int   min_pts = 10;   // minimum amount of points to define the point as main point
constexpr const int   undefined = -1; // label for points which are not defined
constexpr const int   noise = 0;      // label for points which are lie outside the radius
  
int cluster = 0;  // label for points which are in cluster

using point_cloud     =     typename  pcl::PointCloud<pcl::PointXYZ>;  //raw points
using labeled_point_cloud = typename  pcl::PointCloud<pcl::PointXYZL>; //processed points


// Implementation of algorithm DBSCAN
labeled_point_cloud::Ptr dbscan_segmentation(point_cloud::Ptr input_cloud_ptr)
{
    // Label all points as undefined
    labeled_point_cloud::Ptr labeled_cloud_ptr(new labeled_point_cloud());
    copyPointCloud(*input_cloud_ptr, *labeled_cloud_ptr);
    for (auto &&point: *labeled_cloud_ptr) {
        point.label = undefined;
    }

    // Use KDtree for searching points in radius
    pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
    kd_tree.setInputCloud(labeled_cloud_ptr);

    // Check if point is undefined then continue
    for (auto &&point: *labeled_cloud_ptr) {
        if (point.label != undefined) {
            continue;
        }

    // Label point as noise if it is out of radius otherwise count neighbours of the point for the following algorithm 
        std::vector<pcl::index_t> neighbours;
        std::vector<float> sqr_distances;
        auto num_neighbour_pts = kd_tree.radiusSearch(point, radius, neighbours, sqr_distances);
        if (num_neighbour_pts < min_pts) {
            point.label = noise;
            continue;
        }
    // Add the point to cluster
        point.label = ++cluster;

    // Label the point if it is main point and amount of heighbours more than min_pts
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