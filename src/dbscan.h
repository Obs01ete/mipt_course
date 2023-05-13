#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <deque>
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_course::dbscan {

static constexpr double epsilon = 1.0;
static constexpr int min_pts = 10;

enum PointLabel {
    UNDEFINED = -1,
    NOISE = 0
};

using PointCloud = typename pcl::PointCloud<pcl::PointXYZ>;
using LabeledPointCloud = typename pcl::PointCloud<pcl::PointXYZL>;

LabeledPointCloud::Ptr dbscan_segmentation(PointCloud::Ptr input_cloud_ptr)
{
    LabeledPointCloud::Ptr labeled_cloud_ptr(new LabeledPointCloud());
    copyPointCloud(*input_cloud_ptr, *labeled_cloud_ptr);
    for (auto &&point: *labeled_cloud_ptr) {
        point.label = PointLabel::UNDEFINED;
    }

    pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
    kd_tree.setInputCloud(labeled_cloud_ptr);

    int cluster {0};

    for (auto &&point: *labeled_cloud_ptr) {
        if (point.label != PointLabel::UNDEFINED) {
            continue;
        }

        std::vector<pcl::index_t> neighbours;
        std::vector<float> sqr_distances;
        auto neighbours_cnt = kd_tree.radiusSearch(point, epsilon, neighbours, sqr_distances);
        if (neighbours_cnt < min_pts) {
            point.label = PointLabel::NOISE;
            continue;
        }

        ++cluster;
        point.label = cluster;

        std::deque<pcl::index_t> neighbours_to_label(neighbours.begin(), neighbours.end());

        while (!neighbours_to_label.empty()) {
            auto neighbour_idx = neighbours_to_label.front();
            neighbours_to_label.pop_front();

            auto &neighbour_point = labeled_cloud_ptr->points[neighbour_idx];
            if (neighbour_point.label == PointLabel::UNDEFINED) {
                neighbour_point.label = cluster;
                neighbours_cnt = kd_tree.radiusSearch(neighbour_point, epsilon, neighbours, sqr_distances);
                if (neighbours_cnt >= min_pts) {
                    neighbours_to_label.insert(neighbours_to_label.end(), neighbours.begin(), neighbours.end());
                }
            } else if (neighbour_point.label == PointLabel::NOISE) {
                neighbour_point.label = cluster;
            }
        }
    }
    return labeled_cloud_ptr;
}
} // namespace lidar_course

#endif // DBSCAN_H
