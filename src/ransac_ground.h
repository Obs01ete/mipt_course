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

#include <cmath>
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
    Eigen::Vector3f base_point;
    Eigen::Vector3f normal;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// This helper function finds indices of points that are considered inliers,
// given a plane description and a condition on distance from the plane.
std::vector<size_t> find_inlier_indices(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr,
    const Plane& plane,
    std::function<bool(float)> condition_z_fn)
{
    using Transform3f = Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign>;

    // Compute the transformation that relocates the point cloud to the position of base_point of the plane
    Transform3f world_to_ransac_base = Transform3f::Identity();
    world_to_ransac_base.translate(-plane.base_point);

    // Apply the transformation to the input cloud
    auto ransac_base_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    pcl::transformPointCloud(*input_cloud_ptr, *ransac_base_cloud_ptr, world_to_ransac_base);

    // Compute the quaternion that rotates the plane's normal to align with the Z coordinate axis
    Eigen::Quaternionf rotate_to_plane_quat = Eigen::Quaternionf::FromTwoVectors(
            plane.normal.normalized(),
            Eigen::Vector3f::UnitZ()
    );

    // Compute the transformation that aligns the cloud with the XY plane
    Transform3f ransac_base_to_ransac = Transform3f::Identity();
    ransac_base_to_ransac.rotate(rotate_to_plane_quat);

    // Apply the transformation to the cloud that matches the candidate plane
    auto aligned_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
    pcl::transformPointCloud(*ransac_base_cloud_ptr, *aligned_cloud_ptr, ransac_base_to_ransac);

    // Find inliers based on the condition on Z coordinate
    std::vector<size_t> indices;
    for (size_t i_point = 0; i_point < aligned_cloud_ptr->size(); i_point++)
    {
        const auto& p = (*aligned_cloud_ptr)[i_point];
        if (condition_z_fn(p.z))
        {
            indices.push_back(i_point);
        }
    }

    return indices;
}

/**
 * In order to decrease the amount of non-ground points, a rough cropping
 * of the point cloud can be done by limiting the Z coordinate to the
 * range of (-rough_filter_thr, rough_filter_thr). Additionally, decimation
 * of the remaining points is carried out to avoid excessive data for RANSAC.
 *
 * @param input_cloud_ptr Point cloud
 * @param decimation_rate Tolerance threshold on the distance of an inlier to the plane (meters)
 * @param rough_filter_thr Threshold for rough point dropping by Z coordinate (meters)
 * @return filtered points.
 */
auto crop_and_decimate_pcl(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        const size_t decimation_rate,
        const float rough_filter_thr)
{
    std::mt19937::result_type decimation_seed = 41;
    std::mt19937 rng_decimation(decimation_seed);
    auto decimation_gen = std::bind(
            std::uniform_int_distribution<size_t>(0, decimation_rate), rng_decimation);

    auto filtered_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
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
    return filtered_ptr;
}

/**
 * Here we figure out the plane which can be calculated
 * via three points.
 *
 * @param three_points points from which plane will be received
 * @return the plane.
 */
auto plane_from_points(std::vector<Eigen::Vector3f> three_points) {
    auto root_point = three_points[0];
    auto vb = three_points[1] - root_point;
    auto vc = three_points[2] - root_point;
    Eigen::Vector3f normal = vb.cross(vc).normalized();

    // Flip the normal if points down
    if (normal.dot(Eigen::Vector3f::UnitZ()) < 0)
    {
        normal = -normal;
    }
    normal = (normal.dot(Eigen::Vector3f::UnitZ()) < 0)
            ?
            -normal
            :
            normal;
    Plane plane{
        root_point,
        normal
    };
    return plane;
}

size_t num_of_iterations(const double ratio, const double precision) {
    return static_cast<size_t>(
            std::log(1 - precision) / std::log(1 - std::pow(ratio, 3))
            );
}

/**
 * This function performs plane detection with RANSAC sampling of planes
 * that lie on triplets of points randomly sampled from the cloud.
 * Among all trials the plane that is picked is the one that has the highest
 * number of inliers. Inlier points are then removed as belonging to the ground.
 *
 * @param input_cloud_ptr Point cloud
 * @param remove_ground_threshold How much to decimate the input cloud for RANSAC sampling and inlier counting
 * @param decimation_rate Tolerance threshold on the distance of an inlier to the plane (meters)
 * @param rough_filter_thr Threshold for rough point dropping by Z coordinate (meters)
 * @param ransac_tolerance After the final plane is found this is the threshold below which all points are discarded as
 * belonging to the ground.
 * @return Cloud without ground points.
 */
auto remove_ground_ransac(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr,
        const float remove_ground_threshold = 0.2f,
        const size_t decimation_rate = 10,
        const float rough_filter_thr = 0.5f,
        const float ransac_tolerance = 0.1f,
        const float inlier_ratio = 0.5f,
        const float ransac_precision = 0.99f)
{
    auto filtered_ptr = crop_and_decimate_pcl(input_cloud_ptr, decimation_rate, rough_filter_thr);

    // We need a random number generator for sampling triplets of points.
    std::mt19937::result_type sampling_seed = 42;
    std::mt19937 sampling_rng(sampling_seed);
    auto random_index_gen = std::bind(
            std::uniform_int_distribution<size_t>(0, filtered_ptr->size()),
            sampling_rng
            );

    // Number of RANSAC trials
    const size_t num_iterations = num_of_iterations(
            inlier_ratio,
            ransac_precision
            );
    // The best plane is determined by a pair of (number of inliers, plane specification)
    using BestPair = std::pair<size_t, Plane>;
    auto best = std::unique_ptr<BestPair>();
    for (size_t i_iter = 0; i_iter < num_iterations; i_iter++)
    {
        // Sample 3 random points.
        // pa is special in the sense that is becomes an anchor - a base_point of the plane
        std::vector<Eigen::Vector3f> rand3_points(3);
        for (auto& p : rand3_points) {
            p = (*filtered_ptr)[random_index_gen()].getVector3fMap();
        }

        Plane plane = plane_from_points(rand3_points);
        // Call find_inlier_indices to retrieve inlier indices.
        // We will need only the number of inliers.
        auto inlier_indices = find_inlier_indices(filtered_ptr, plane,
            [ransac_tolerance](float z) -> bool {
                return (z >= -ransac_tolerance) && (z <= ransac_tolerance);
            });

        // If new best plane is found, update the best
        bool found_new_best = false;
        if (best)
        {
            if (inlier_indices.size() > best->first)
            {
                found_new_best = true;
            }
        }
        else
        {
            // For the first trial update anyway
            found_new_best = true;
        }

        if (found_new_best)
        {
            best = std::unique_ptr<BestPair>(new BestPair{inlier_indices.size(), plane});
        }
    }

    // For the best plane filter out all the points that are
    // below the plane + remove_ground_threshold.
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground_ptr;
    if (best)
    {
        cloud_no_ground_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
        auto inlier_indices = find_inlier_indices(input_cloud_ptr, best->second,
            [remove_ground_threshold](float z) -> bool {
                return z <= remove_ground_threshold;
            });
        std::unordered_set<size_t> inlier_set(inlier_indices.begin(), inlier_indices.end());
        for (size_t i_point = 0; i_point < input_cloud_ptr->size(); i_point++)
        {
            if (!inlier_set.count(i_point))
            {
                const auto& p = (*input_cloud_ptr)[i_point];
                cloud_no_ground_ptr->push_back(p);
            }
        }
    }
    else
    {
        cloud_no_ground_ptr = input_cloud_ptr;
    }

    return cloud_no_ground_ptr;
}


} // namespace lidar_course
