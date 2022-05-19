/**
 *  DBSCAN algortithm
 */

#include "dbscan.h"
#include "point_cloud_kernels.h"

namespace lidar_course {

constexpr size_t DBSCAN_NEIGHBOR_MIN_POINTS = 3;
constexpr double DBSCAN_NEIGHBOR_BASE_RADIUS = 0.5;
constexpr double DBSCAN_NEIGHBOR_RADIUS_SCALE_RATE = 0.025;
constexpr double DBSCAN_START_RADIUS = 0.5;
constexpr double DBSCAN_END_RADIUS = 30.;


void LabelPointAndNeighbors(pcl::octree::OctreePointCloudSearch<pcl::PointXYZL> &octree,
                            pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud,
                            pcl::PointXYZL &p,
                            uint32_t label)
{
    assert(label != 0);
    p.label = label;
 
    float linear_distance = std::sqrt(p.x * p.x + p.y * p.y);
    if ((linear_distance > DBSCAN_END_RADIUS) || (linear_distance < DBSCAN_START_RADIUS)) {
        return;
    }

    double neighbor_radius = DBSCAN_NEIGHBOR_BASE_RADIUS + linear_distance * DBSCAN_NEIGHBOR_RADIUS_SCALE_RATE;

    std::vector<int> idxs;
    std::vector<float> distances;
    octree.radiusSearch(p, neighbor_radius, idxs, distances);
    if (idxs.size() > DBSCAN_NEIGHBOR_MIN_POINTS) {
        for (size_t i = 0; i < idxs.size(); i++) {
            auto &n_point = (*labeled_cloud)[idxs[i]];
            if (n_point.label != label) {
                LabelPointAndNeighbors(octree, labeled_cloud, n_point, label);
            }
        }
    }

    return;
}

pcl::PointCloud<pcl::PointXYZL>::Ptr dbscan(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr)
{
    auto labeled_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();
    pcl::copyPointCloud(*input_cloud_ptr, *labeled_cloud);

    float octree_resolution = 3;
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZL> octree(octree_resolution);
    octree.setInputCloud(labeled_cloud);
    octree.addPointsFromInputCloud();

    uint32_t cur_label = 0;
    for (auto &point : *labeled_cloud) {
        if (point.label == 0) {
            cur_label++;
            LabelPointAndNeighbors(octree, labeled_cloud, point, cur_label);
        }
    }
    return labeled_cloud;
}

}  // namespace lidar_course