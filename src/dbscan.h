#pragma once

#include <queue>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace lidar_course {

template <typename data_type>
class dbscan {

private:
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree{};
    std::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> label_point_cloud{};

enum point_cond {
    UNDEFINED = -1,
    NOISE = 0
};

const double RAD = 1.0;
const uint32_t EPS = 10;

public:
    dbscan(typename pcl::PointCloud<data_type>::Ptr point_cloud) {
        label_point_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();

        for (auto &point : *point_cloud)
            label_point_cloud->push_back(pcl::PointXYZL(point.x, point.y, point.z, point_cond::NOISE));
        
        kdtree.setInputCloud(label_point_cloud);
    }

    auto run_segmentation() {
        uint32_t cluster_id = 1;
        std::queue<size_t> labels_queue{};

        for (auto &point : *label_point_cloud) {
            if (point.label != point_cond::NOISE)
                continue;

            bool is_noise = expand_cluster(point, labels_queue, cluster_id);

            while (labels_queue.size() > 0) {
                size_t cur_label = labels_queue.front();
                labels_queue.pop();

                auto &neighbor = (*label_point_cloud)[cur_label];
                expand_cluster(neighbor, labels_queue, cluster_id);
            }
            if (!is_noise)
                ++cluster_id;
        }

        return label_point_cloud;
    }

private:
    bool expand_cluster(pcl::PointXYZL &point, std::queue<size_t>& labels_queue, uint32_t cluster_id) {
        std::vector<int> found_point_indx;
        std::vector<float> found_point_dist;

        kdtree.radiusSearch(point, RAD, found_point_indx, found_point_dist);
        bool is_noise = found_point_indx.size() < EPS;

        if (!is_noise) {
            point.label = cluster_id;
            for (auto ind : found_point_indx) {
                assert(ind >= 0);
                auto &neighbor_point = (*label_point_cloud)[ind];

                if (neighbor_point.label == point_cond::NOISE) {
                    neighbor_point.label = cluster_id;
                    labels_queue.push(ind);
                }
            }
        }
        return is_noise;
    }
};
}