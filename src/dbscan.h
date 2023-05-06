#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>

#include <pcl/kdtree/kdtree.h>

namespace lidar_course {

template <typename PointT>
auto dbscan_segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr) {
	constexpr const float radius = 1.0f;
	constexpr const int min_pts = 10;
	constexpr const int undefined = -1;
	constexpr const int noise = 0;

	int idx_of_cluster = 0;

	pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_labeled_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();

	std::transform(input_cloud_ptr->cbegin(), input_cloud_ptr->cend(), std::back_inserter(*cloud_labeled_ptr),
				   [](const auto &point) { return pcl::PointXYZL(point.x, point.y, point.z, undefined);});

	pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
	kd_tree.setInputCloud(cloud_labeled_ptr);

	for (auto &point: *cloud_labeled_ptr) {
		if (point.label != undefined) continue;

		std::vector<pcl::index_t> neighbours_idx;
		std::vector<float> sqr_dist;

		if (kd_tree.radiusSearch(point, radius, neighbours_idx, sqr_dist) < min_pts) {
			point.label = noise;
			continue;
		}

		point.label = ++idx_of_cluster;

		while(!neighbours_idx.empty()) {
			const pcl::index_t idx = neighbours_idx.back();
			neighbours_idx.pop_back();

			auto& cur_point = cloud_labeled_ptr->points[idx];

			if (cur_point.label == noise) cur_point.label = idx_of_cluster;
			if (cur_point.label != undefined) continue;

			cur_point.label = idx_of_cluster;

			sqr_dist.clear();
			std::vector<pcl::index_t> local_neighbours_idx;

			if (kd_tree.radiusSearch(cur_point, radius, local_neighbours_idx, sqr_dist) >= min_pts)
				std::copy(local_neighbours_idx.cbegin(), local_neighbours_idx.cend(), std::back_inserter(neighbours_idx));
		}
	}
	return cloud_labeled_ptr;
}
}

#endif
