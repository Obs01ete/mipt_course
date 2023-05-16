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
        Eigen::Vector3f base_point;
        Eigen::Vector3f normal;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    float dot_product(const Eigen::Vector3f& lhs, const Eigen::Vector3f& rhs) {
        return lhs(0) * rhs(0) + lhs(1) * rhs(1) + lhs(2) * rhs(2);
    }

    float distance_from_point_to_plane(const Plane& plane, const Eigen::Vector3f& point) {
        float num = dot_product(plane.base_point, plane.normal) + dot_product(point, plane.normal);
        float den = sqrt(dot_product(plane.normal, plane.normal));

        return num / den;
    }

    std::vector<size_t> find_inlier_indices(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &input_cloud_ptr,
        const Plane &plane,
        std::function<bool(float)> condition_z_fn)
    {
        typedef Eigen::Transform<float, 3, Eigen::Affine, Eigen::DontAlign> Transform3f;

        auto base_point = plane.base_point;
        auto normal = plane.normal;

        std::vector<size_t> indices;
        for (size_t i_point = 0; i_point < input_cloud_ptr->size(); i_point++)
        {
            const auto &p = (*input_cloud_ptr)[i_point];
            Eigen::Vector3f p_vec { p.x, p.y, p.z };
            if (condition_z_fn(distance_from_point_to_plane(plane, p_vec)))
            {
                indices.push_back(i_point);
            }
        }
        return indices;
    }

    typedef std::pair<size_t, Plane> BestPair;
    #define MAX_ITERATIONS_DISTANCE 5

    bool find_best_recursion(std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> point_cloud, size_t first, size_t second, BestPair& best)
    {

        size_t third = first + (second - first) / 2; // middle point between first and second
        if (third - first < MAX_ITERATIONS_DISTANCE)
        {
            return false;
        }

        BestPair left = {}, right = {};

        Eigen::Vector3f pa = (*point_cloud)[first].getVector3fMap();
        Eigen::Vector3f pb = (*point_cloud)[second].getVector3fMap();
        Eigen::Vector3f pc = (*point_cloud)[third].getVector3fMap();

        auto vb = pb - pa;
        auto vc = pc - pa;
        Eigen::Vector3f normal = vb.cross(vc).normalized();

        if (normal.dot(Eigen::Vector3f::UnitZ()) < 0)
        {
            normal = -normal;
        }

        Plane plane{ pa, normal };

        const float ransac_tolerance = 0.1f;
        auto inlier_indices = find_inlier_indices(point_cloud, plane,
                                                  [ransac_tolerance](float z) -> bool
                                                  {
                                                      return (z >= -ransac_tolerance) && (z <= ransac_tolerance);
                                                  });

        best = std::make_pair(inlier_indices.size(), plane);

        if (find_best_recursion(point_cloud, first, third, left))
        {
            if (left.first > best.first)
                best = left;
        }
        if (find_best_recursion(point_cloud, third, second, right))
        {
            if (right.first > best.first)
                best = right;
        }

        return true;
    }

    // This function performs plane detection with RANSAC sampling of planes
    // that lie on triplets of points randomly sampled from the cloud.
    // Among all trials the plane that is picked is the one that has the highest
    // number of inliers. Inlier points are then removed as belonging to the ground.
    auto remove_ground_ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_ptr)
    {
        // Threshold for rough point dropping by Z coordinate (meters)
        const float rough_filter_thr = 0.5f;
        // How much to decimate the input cloud for RANSAC sampling and inlier counting
        const size_t decimation_rate = 10;

        // Tolerance threshold on the distance of an inlier to the plane (meters)
        const float ransac_tolerance = 0.1f;
        // After the final plane is found this is the threshold below which all
        // points are discarded as belonging to the ground.
        const float remove_ground_threshold = 0.2f;

        // To reduce the number of outliers (non-ground points) we can roughly crop
        // the point cloud by Z coordinate in the range (-rough_filter_thr, rough_filter_thr).
        // Simultaneously we perform decimation of the remaining points since the full
        // point cloud is excessive for RANSAC.
        std::mt19937::result_type decimation_seed = 41;
        std::mt19937 rng_decimation(decimation_seed);
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

        BestPair best;
        if (!find_best_recursion(filtered_ptr, 0, filtered_ptr->size() -1, best))
        {
            return input_cloud_ptr;
        }

        // For the best plane filter out all the points that are
        // below the plane + remove_ground_threshold.
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground_ptr;
        {
            cloud_no_ground_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
            auto inlier_indices = find_inlier_indices(input_cloud_ptr, best.second,
                                                      [remove_ground_threshold](float z) -> bool
                                                      {
                                                          return z <= remove_ground_threshold;
                                                      });
            std::unordered_set<size_t> inlier_set(inlier_indices.begin(), inlier_indices.end());
            for (size_t i_point = 0; i_point < input_cloud_ptr->size(); i_point++)
            {
                bool extract_non_ground = true;
                if ((inlier_set.find(i_point) == inlier_set.end()) == extract_non_ground)
                {
                    const auto &p = (*input_cloud_ptr)[i_point];
                    cloud_no_ground_ptr->push_back(p);
                }
            }
        }
        return cloud_no_ground_ptr;
    }

} // namespace lidar_course