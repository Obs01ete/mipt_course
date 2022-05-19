#pragma once

#include <cassert>
#include <pcl/common/common.h>
#include <pcl/octree/octree_pointcloud.h>
#include <pcl/octree/octree_search.h>

namespace lidar_course {

void LabelPointAndNeighbors(pcl::octree::OctreePointCloudSearch<pcl::PointXYZL> &octree,
                            pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud,
                            pcl::PointXYZL &p,
                            uint32_t label);

pcl::PointCloud<pcl::PointXYZL>::Ptr dbscan(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr);

}  // namespace lidar_course