#pragma once

#include <pcl/kdtree/kdtree.h>

#include <vector>

template <typename PointT>
auto dbscan_segmentation(typename pcl::PointCloud<PointT>::Ptr input_cloud_ptr)
{
	const float epsilon = 1.0f;
	const size_t min_neighbours = 10;

	enum point_type : int {
		UNDEFINED = 0,
		NOISE = -1
	};

	pcl::PointCloud<pcl::PointXYZL>::Ptr changed_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();

	for (const auto& input_point : *input_cloud_ptr) {
		changed_cloud_ptr->push_back(pcl::PointXYZL(input_point.x, input_point.y, input_point.z, point_type::UNDEFINED));
	}

	pcl::KdTreeFLANN<pcl::PointXYZL> kd_tree;
	kd_tree.setInputCloud(changed_cloud_ptr);

	size_t n_clusters = 0;


	for (auto& point : *changed_cloud_ptr) {
		if (point.label != point_type::UNDEFINED)
			continue;

		std::vector<pcl::index_t> neighbours_idx;
		std::vector<float> neighbours_radius_squared;

		if (kd_tree.radiusSearch(point, epsilon, neighbours_idx, neighbours_radius_squared) < min_neighbours) {
			point.label = point_type::NOISE;
			continue;
		}

		n_clusters++;
		point.label = n_clusters;

		while (!neighbours_idx.empty()) {
			const pcl::index_t idx = neighbours_idx.back();
			neighbours_idx.pop_back();

			auto& neighbour_point = changed_cloud_ptr->points[idx];

			if (neighbour_point.label == point_type::NOISE) {
				neighbour_point.label = n_clusters;
			}
			if (neighbour_point.label != point_type::UNDEFINED) {
				continue;
			}
			neighbour_point.label = n_clusters;

			std::vector<pcl::index_t> neighbours_of_neighbour_idx;
			std::vector<float> neighbours_of_neighbour_radius_squared;

			if (kd_tree.radiusSearch(neighbour_point, epsilon, neighbours_of_neighbour_idx,
			neighbours_of_neighbour_radius_squared) >= min_neighbours) {
				neighbours_idx.insert(neighbours_idx.cend(), neighbours_of_neighbour_idx.cbegin(),
									  neighbours_of_neighbour_idx.cend());
			}
		}
	}
	return changed_cloud_ptr;
}