/**
 * MIT License
 *
 * Copyright (c) 2020 Dmitrii Khizbullin <dmitrii.khizbullin@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#pragma once

#include <vector>
#include <deque>
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_course {
namespace dbscan {

enum PointLabel {
    NOISE = 0,
    UNDEFINED = -1
};

static constexpr float radius = 1.0f;
static constexpr int min_points = 10;

using PointCloudLabeled = pcl::PointCloud<pcl::PointXYZL>;
using PointCloudLabeledPtr = PointCloudLabeled::Ptr;

template <typename PointT>
PointCloudLabeledPtr segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
{
    PointCloudLabeledPtr cloud_labeled_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();

    for (const auto &input_point : *input_cloud_ptr) {
        cloud_labeled_ptr->push_back(pcl::PointXYZL(input_point.x, input_point.y, input_point.z, PointLabel::UNDEFINED));
    }
    pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
    kd_tree.setInputCloud(cloud_labeled_ptr);

    size_t cluster_idx = 0;
    for (auto &point : *cloud_labeled_ptr) {
        if(point.label != PointLabel::UNDEFINED) {
            continue;
        }

        std::vector<pcl::index_t> neighbors;
        std::vector<float> sqr_distances;

        if (kd_tree.radiusSearch(point, radius, neighbors, sqr_distances) < min_points) {
            point.label = PointLabel::NOISE;
            continue;
        }
        point.label = ++cluster_idx;

        std::deque<pcl::index_t> points_to_label(neighbors.begin(), neighbors.end());

        while (!points_to_label.empty()) {
            const pcl::index_t idx = points_to_label.front();
            points_to_label.pop_front();

            auto &current_point = cloud_labeled_ptr->points[idx];

            if (current_point.label == PointLabel::NOISE) {
                current_point.label = cluster_idx;
            }
            if (current_point.label != PointLabel::UNDEFINED) {
                continue;
            }
            current_point.label = cluster_idx;

            neighbors.clear();
            sqr_distances.clear();

            if (kd_tree.radiusSearch(current_point, radius, neighbors, sqr_distances) >= min_points) {
                for (const auto &local_neighbors : neighbors) {
                    points_to_label.push_back(local_neighbors);
                }
            }
        }
    }
    return cloud_labeled_ptr;
}

} //  namespace dbscan
} //  namespace lidar_course
