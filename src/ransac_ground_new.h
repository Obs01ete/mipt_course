#pragma once

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Geometry>
#include <cmath>


namespace lidar_course {


/**
 * A plane is represented with a point on the plane ([base_point])
 * and a [normal] vector to the plane.
 */
struct Plane {
    Eigen::Vector3f base_point;
    Eigen::Vector3f normal;

    Plane(const Eigen::Vector3f& bp, const Eigen::Vector3f& n) :
        base_point(bp), normal(n) {}

    /**
     * Constructs Plane from 3 points
     *
     * @param three_points points from which plane will be received
     */
    Plane(const std::vector <Eigen::Vector3f>& three_points) {
        auto root_point = three_points[0];
        auto vb = three_points[1] - root_point;
        auto vc = three_points[2] - root_point;
        Eigen::Vector3f normal = vb.cross(vc).normalized();
        if (normal.dot(Eigen::Vector3f::UnitZ()) < 0) {
            normal = -normal;
        }
        this->base_point = root_point;
        this->normal = normal;
    }

    /**
     * Finds distance from given point to the Plane
     *
     * @param three_points points from which plane will be received
     * @return distance to point
     */
    float distance_to(const Eigen::Vector3f& point) const {
        return abs(normal.dot(base_point - point));
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * Finds indices of points that are considered inliers,
 * given a plane description and a condition on distance from the plane.
 *
 * @param three_points points from which plane will be received
 * @return distance to point
 */
std::vector <size_t> find_inlier_indices(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr,
        const Plane& plane,
        std::function<bool(float)> condition_z_fn) {
    std::vector<size_t> indices;
    for (size_t i = 0; i < input_cloud_ptr->size(); ++i) {
        auto p = (*input_cloud_ptr)[i];
        float dist = plane.distance_to(p.getVector3fMap());
        if (condition_z_fn(dist)) {
            indices.push_back(i);
        }
    }
    return indices;
}

/**
 * Crop the point cloud by Z coordinate in the range (-rough_filter_thr, rough_filter_thr).
 * Simultaneously performs decimation of the remaining points since the full
 * point cloud is excessive for RANSAC.
 *
 * @param input_cloud_ptr Point cloud
 * @param decimation_rate Tolerance threshold on the distance of an inlier to the plane (meters)
 * @param rough_filter_thr Threshold for rough point dropping by Z coordinate (meters)
 * @return pointer to filtered point cloud
 */
auto crop_and_decimate_pcl(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        const size_t decimation_rate,
        const float rough_filter_thr) {
    std::mt19937::result_type decimation_seed = 41;
    std::mt19937 rng_decimation(decimation_seed);
    auto decimation_gen = std::bind(
            std::uniform_int_distribution<size_t>(0, decimation_rate), rng_decimation);

    auto filtered_ptr = std::make_shared < pcl::PointCloud < pcl::PointXYZ >> ();
    for (const auto &p: *input_cloud_ptr) {
        if ((p.z > -rough_filter_thr) && (p.z < rough_filter_thr)) {
            if (decimation_gen() == 0) {
                filtered_ptr->push_back(p);
            }
        }
    }
    return filtered_ptr;
}

/**
 * Calculate number of RANSAC iteration based on parameters
 *
 * @param inlier_fraction The ration of the inliers to all points in the point cloud
 * @param ransac_precision Precision factor of RANSAC algorithm
 * @return number of RANSAC iteration based on parameters
 */
size_t count_num_of_ransac_iters(const double inlier_fraction, const double ransac_precision) {
    const double result = std::log(1 - ransac_precision) / std::log(1 - std::pow(inlier_fraction, 3));
    return std::lround(result);
}

/**
 * This function filter out all the points that are below the plane + remove_ground_threshold.
 *
 * @param input_cloud_ptr Point cloud
 * @param plane Plane
 * @param remove_ground_threshold How much to decimate the input cloud for RANSAC sampling and inlier counting
 * @return point cloud without inliers
 */
auto pcl_without_inliers(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        const Plane &plane,
        const float remove_ground_threshold) {
    auto cloud_no_ground_ptr = std::make_shared < pcl::PointCloud < pcl::PointXYZ > > ();
    auto inlier_indices = find_inlier_indices(input_cloud_ptr, plane,
                                              [remove_ground_threshold](float z) -> bool {
                                                  return z <= remove_ground_threshold;
                                              });
    std::unordered_set <size_t> inlier_set(inlier_indices.begin(), inlier_indices.end());
    for (size_t i_point = 0; i_point < input_cloud_ptr->size(); i_point++) {
        bool extract_non_ground = true;
        if ((inlier_set.find(i_point) == inlier_set.end()) == extract_non_ground) {
            const auto &p = (*input_cloud_ptr)[i_point];
            cloud_no_ground_ptr->push_back(p);
        }
    }
    return cloud_no_ground_ptr;
}

/**
 * This function performs plane detection with RANSAC sampling of planes
 * that lie on triplets of points randomly sampled from the cloud.
 * Among all trials the plane that is picked is the one that has the highest
 * number of inliers. Inlier points are then removed as belonging to the ground.
 *
 * @param input_cloud_ptr Point cloud
 * @param inlier_fraction The ration of the inliers to all points in the point cloud
 * @param ransac_precision Precision factor of ransac algorithm
 * @param remove_ground_threshold How much to decimate the input cloud for RANSAC sampling and inlier counting
 * @param decimation_rate Tolerance threshold on the distance of an inlier to the plane (meters)
 * @param rough_filter_thr Threshold for rough point dropping by Z coordinate (meters)
 * @param ransac_tolerance After the final plane is found this is the threshold below which all points are discarded as
 * belonging to the ground.
 * @return point cloud without inliers
 */
auto remove_ground_ransac(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        const float inlier_fraction = 0.5f,
        const float ransac_precision = 0.99f,
        const float remove_ground_threshold = 0.2f,
        const size_t decimation_rate = 10,
        const float rough_filter_thr = 0.5f,
        const float ransac_tolerance = 0.1f) {
    auto filtered_ptr = crop_and_decimate_pcl(input_cloud_ptr, decimation_rate, rough_filter_thr);

    // We need a random number generator for sampling triplets of points.
    std::mt19937::result_type sampling_seed = 42;
    std::mt19937 sampling_rng(sampling_seed);
    auto random_index_gen = std::bind(
            std::uniform_int_distribution<size_t>(0, filtered_ptr->size()), sampling_rng);

    const size_t num_iterations = count_num_of_ransac_iters(inlier_fraction, ransac_precision);
    using BestPair = std::pair<size_t, Plane>;
    auto best = std::unique_ptr<BestPair>();
    for (size_t i_iter = 0; i_iter < num_iterations; i_iter++) {
        std::vector<Eigen::Vector3f> rand3_points(3);
        for (auto &p: rand3_points) {
            p = (*filtered_ptr)[random_index_gen()].getVector3fMap();
        }
        Plane plane(rand3_points);
        auto inlier_indices = find_inlier_indices(filtered_ptr, plane,
                                                  [ransac_tolerance](float z) -> bool {
                                                      return (z >= -ransac_tolerance) && (z <= ransac_tolerance);
                                                  });
        if (best == nullptr || inlier_indices.size() > best->first)
            best = std::unique_ptr<BestPair>(new BestPair{inlier_indices.size(), plane});
    }
    return best ? pcl_without_inliers(input_cloud_ptr, best->second, remove_ground_threshold) : input_cloud_ptr;
}


} // namespace lidar_course
