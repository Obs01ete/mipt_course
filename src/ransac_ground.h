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

#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

#include <Eigen/Geometry>


namespace lidar_course {


// A plane is represented with a point on the plane (base_point)
// and a normal vector to the plane.
struct Plane
{
    Eigen::Vector3f base_point {};
    Eigen::Vector3f normal {};

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


// This helper function finds indices of points that are considered inliers,
// given a plane description and a condition on signed distance from the plane.
template<class ConditionZ>
inline std::vector<size_t> find_inlier_indices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr,
    const Plane& plane,
    ConditionZ condition_z)
{
    // Use general form of the equation of a plane:
    // ax + by + cz + d = 0,
    // normal = [a, b, c]
    auto n = plane.normal;
    float d_coeff = -n.dot(plane.base_point);

    float nx = n[0], ny = n[1], nz = n[2];

    // We can apply a simple criterion on signed distance (which is equal
    // to z coordinate in plane coordinate system) to find inliers.
    std::vector<size_t> indices;
    for (size_t i_point = 0, end = input_cloud_ptr->size(); i_point != end; ++i_point)
    {
        const auto& p = (*input_cloud_ptr)[i_point];
        float dist = nx*p.x + ny*p.y + nz*p.z + d_coeff;

        if (condition_z(dist))
        {
            indices.push_back(i_point);
        }
    }
    return indices;
}


// This function performs plane detection with RANSAC sampling of planes
// that lie on triplets of points randomly sampled from the cloud.
// Among all trials the plane that is picked is the one that has the highest
// number of inliers. Inlier points are then removed as belonging to the ground.
inline auto remove_ground_ransac(
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr)
{
    // Threshold for rough point dropping by Z coordinate (meters)
    const float rough_filter_thr = 0.5f;
    // How much to decimate the input cloud for RANSAC sampling and inlier counting
    const size_t decimation_rate = 10;

    // To reduce the number of outliers (non-ground points) we can roughly crop
    // the point cloud by Z coordinate in the range (-rough_filter_thr, rough_filter_thr).
    // Simultaneously we perform decimation of the remaining points since the full
    // point cloud is excessive for RANSAC.
    std::mt19937 rng_decimation(std::random_device {}());
    auto decimation_gen = std::bind(
        std::uniform_int_distribution<size_t>(0, decimation_rate), rng_decimation);

    auto filtered_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    for (const auto& p : *input_cloud_ptr)
    {
        if ((p.z > -rough_filter_thr) && (p.z < rough_filter_thr))
        {
            // Use random number generator to avoid introducing patterns
            // (which are possible with structured subsampling
            // like picking each Nth point).
            if (decimation_gen() == 0)
            {
                filtered_ptr->push_back(p);
            }
        }
    }

    // Number of probes is based on the required errors probability and
    // expected share of plane points in cloud
    const float required_error_prob = 0.001;
    const float expected_plane_pts_share = 0.67;
    const float wrong_sample_prob = 1 - std::pow(expected_plane_pts_share, 3);
    const size_t num_iterations = std::log(required_error_prob) / std::log(wrong_sample_prob);

    // Tolerance threshold on the distance of an inlier to the plane (meters)
    const float ransac_tolerance = 0.1f;

    // We need a random number generator for sampling triplets of points.
    std::mt19937 sampling_rng(std::random_device {}());
    auto random_index_gen = std::bind(
        std::uniform_int_distribution<size_t>(0, filtered_ptr->size()), sampling_rng);

    // The best plane is determined by a pair of (number of inliers, plane specification)
    using BestPair = std::pair<size_t, Plane>;
    // Default value will be replaced by any kind of choosen pair
    BestPair best {};
    for (size_t i_iter = 0; i_iter < num_iterations; i_iter++)
    {
        // Sample 3 random points.
        // pa is special in the sense that is becomes an anchor - a base_point of the plane
        Eigen::Vector3f pa = (*filtered_ptr)[random_index_gen()].getVector3fMap();
        Eigen::Vector3f pb = (*filtered_ptr)[random_index_gen()].getVector3fMap();
        Eigen::Vector3f pc = (*filtered_ptr)[random_index_gen()].getVector3fMap();

        // Here we figure out the normal to the plane which can be easily calculated
        // as a normalized cross product.
        auto vb = pb - pa;
        auto vc = pc - pa;
        Eigen::Vector3f normal = vb.cross(vc).normalized();

        // Flip the normal if points down
        if (normal.dot(Eigen::Vector3f::UnitZ()) < 0)
        {
            normal = -normal;
        }

        Plane plane{pa, normal};

        // Call find_inlier_indices to retrieve inlier indices.
        // We will need only the number of inliers.
        auto inlier_indices = find_inlier_indices(filtered_ptr, plane,
            [ransac_tolerance](float z) -> bool {
                return (z >= -ransac_tolerance) && (z <= ransac_tolerance);
            });

        if (inlier_indices.size() > best.first)
        {
            best = BestPair{inlier_indices.size(), plane};
        }
    }

    // After the final plane is found this is the threshold below which all
    // points are discarded as belonging to the ground.
    const float remove_ground_threshold = 0.2f;

    // For the best plane filter out all the points that are
    // below the plane + remove_ground_threshold.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground_ptr;
    cloud_no_ground_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    auto cloud_no_ground_indices = find_inlier_indices(input_cloud_ptr, best.second,
        [remove_ground_threshold](float z) -> bool {
            return z > remove_ground_threshold;
        });

    for (auto i_no_ground : cloud_no_ground_indices) {
        const auto& p = (*input_cloud_ptr)[i_no_ground];
        cloud_no_ground_ptr->push_back(p);
    }

    return cloud_no_ground_ptr;
}


} // namespace lidar_course
