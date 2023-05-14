/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2014-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <deque>
#include <vector>

#include <pcl/kdtree/kdtree.h>

namespace lidar_course {


// https://habr.com/ru/articles/322034/


template<typename PointT>
auto dbscan_segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr) 
{
    // DBSCAN stuff
    const int minPts = 5;
    const float epsilon = 1.0f;

    // In the current implemenation we don't need field's called
    // "BOUND" and "CORE", such a points are those, who are not
    // "NOISE" and "UNDEFINED"
    enum point_label : int {
        UNDEFINED = -1,
        NOISE = 0
    };


    pcl::PointCloud<pcl::PointXYZL>::Ptr dbscan_labeled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();

    // Collecting all the points and putting them into KD tree
    pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
    for (const auto& given_point : *input_cloud_ptr) {
        dbscan_labeled_cloud->push_back(pcl::PointXYZL(given_point.x, given_point.y, given_point.z, point_label::UNDEFINED));
    }
    kd_tree.setInputCloud(dbscan_labeled_cloud);

    // Going through all the points in the cloud
    int cluster_idx = 0;
    for (auto& cloud_point : *dbscan_labeled_cloud) {
        if (cloud_point.label != point_label::UNDEFINED)
            continue;

        // Searching for neighbors
        std::vector<pcl::index_t> neighbors;
        std::vector<float> squared_distances;
        kd_tree.radiusSearch(cloud_point, epsilon, neighbors, squared_distances);

        // Separation of "NOISE" points.
        // Such a points, that have not enough neighbors, are called
        // "NOISE" ones
        if (neighbors.size() < minPts) {
            cloud_point.label = point_label::NOISE;
            continue;
        }

        // Marking core and bound points with non-noise ones.
        ++cluster_idx;
        cloud_point.label = cluster_idx;
        std::deque<pcl::index_t> points_to_label(neighbors.begin(), neighbors.end());

        // Process whole cluster with queue
        while (!points_to_label.empty()) {
            auto& curr_point = dbscan_labeled_cloud->points[points_to_label.front()];
            points_to_label.pop_front();

            // "NOISE" point promotion
            if (curr_point.label == point_label::NOISE)
                curr_point.label = cluster_idx;

            // "UNDEFINED" point case
            if (curr_point.label != point_label::UNDEFINED)
                continue;
            curr_point.label = cluster_idx;

            // Get neighbors of the current point
            neighbors.clear();
            squared_distances.clear();
            kd_tree.radiusSearch(curr_point, epsilon, neighbors, squared_distances);

            // Addition of the neighbors to the queue
            for(const auto& pts : neighbors)
                points_to_label.push_back(pts);
        }
    }
    return dbscan_labeled_cloud;
}


} // namespace lidar_course