#pragma once

#include <queue>

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace lidar_course {

namespace {
    // Points without clusters. This label should be equal UNLABELED,
    // so noise points won't be confused with cluster in further processing.
    constexpr std::uint32_t NOISE = 0;
    // End of reserved labels. Start of clusters IDs
    constexpr std::uint32_t LABEL_OFFSET = 1;
};

template <typename PointT>
auto labelPointCloud(typename pcl::PointCloud<PointT>::Ptr pointCloud) {
    // get new point cloud
    auto labeledPointCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL> >();
    labeledPointCloud->is_dense = pointCloud->is_dense;
    for (const auto &point : *pointCloud) {
        labeledPointCloud->push_back(pcl::PointXYZL(point.x, point.y, point.z, NOISE));
    }
    return labeledPointCloud;
}

template <typename PointT>
auto dbscan_segmentation(typename pcl::PointCloud<PointT>::Ptr pointCloud) {
    // DBScan properties
    double radius = 1.0f;
    size_t neighborThreshold = 10;
    uint32_t clusterLabel = LABEL_OFFSET;
    
    auto labeledPointCloud = labelPointCloud<PointT>(pointCloud);
    pcl::KdTreeFLANN<pcl::PointXYZL> kdtree;
    std::queue<size_t> neighborQueue;
    std::vector<int> pointNeighborsIdx;
    std::vector<float> pointNeighborsDistance;

    kdtree.setInputCloud(labeledPointCloud);
    
    auto pushNeighbors = [&] (pcl::PointXYZL &point) {
        pointNeighborsIdx.clear();
        pointNeighborsDistance.clear();
        kdtree.radiusSearch(point, radius, pointNeighborsIdx, pointNeighborsDistance);
        bool isMainPoint = (pointNeighborsIdx.size() >= neighborThreshold);
        if (isMainPoint) {
            point.label = clusterLabel;
            for (int idx : pointNeighborsIdx) {
                auto &neighbor = (*labeledPointCloud)[idx];
                if (neighbor.label == NOISE) {
                    neighbor.label = clusterLabel;
                    neighborQueue.push(idx);
                }
            }
        }
        return isMainPoint;
    };
    
    for (auto &point : *labeledPointCloud) {
        if (point.label != NOISE) {
            continue;
        }
        bool createdCluster = pushNeighbors(point);
        while (!neighborQueue.empty()) {
            int neighborIdx = neighborQueue.front();
            neighborQueue.pop();
            auto &neighbor = (*labeledPointCloud)[neighborIdx];
            pushNeighbors(neighbor);
        }
        if (createdCluster) {
            clusterLabel++;
        }
    }

    return labeledPointCloud;
}

}