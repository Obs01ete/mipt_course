/**
 * MIT License
 *
 * Copyright (c) 2023 Roman Glaz <vokerlee@gmail.com>,
 *                    Dmitrii Khizbullin <dmitrii.khizbullin@gmail.com>
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

#include <pcl/surface/convex_hull.h>
#include <stack>
#include <algorithm>
#include <cmath>


namespace lidar_course
{

namespace
{
    const long double eps = 1e-6;
}

template <typename PointInT>
class GrahamHull : public pcl::ConvexHull<PointInT>
{
protected:
    // useful usings from ConvexHull (convex_hull.h)
    using pcl::ConvexHull<PointInT>::input_;
    using pcl::ConvexHull<PointInT>::indices_;
    using pcl::ConvexHull<PointInT>::initCompute;
    using pcl::ConvexHull<PointInT>::deinitCompute;

public:
    // useful usings from ConvexHull (convex_hull.h)
    using Ptr = pcl::shared_ptr<GrahamHull<PointInT>>;
    using ConstPtr = pcl::shared_ptr<const GrahamHull<PointInT>>;

    using PointCloud = pcl::PointCloud<PointInT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using pcl::ConvexHull<PointInT>::setInputCloud;

    /** \brief Empty constructor (ConvexHull). */
    GrahamHull() = default;
    /** \brief Empty destructor (ConvexHull). */
    ~GrahamHull() = default;

    /** \brief Compute a convex hull for all points given.
        *
        * \note In 2D case (i.e. if the input points belong to one plane)
        * the \a polygons vector will have a single item, whereas in 3D
        * case it will contain one item for each hull facet.
        *
        * \param[out] points the resultant points lying on the convex hull.
        * \param[out] polygons the resultant convex hull polygons, as a set of
        * vertices. The Vertices structure contains an array of point indices.
        */
    void reconstruct(PointCloud &points,
                     std::vector<pcl::Vertices> &polygons)
    {
        points.header = input_->header;
        if (!initCompute() || input_->points.empty() || indices_->empty())
        {
            points.clear();
            return;
        }

        // Perform the actual surface reconstruction
        performReconstruction(points, polygons);

        points.width = points.size();
        points.height = 1;
        points.is_dense = true;

        deinitCompute();
    }

private:

    enum class AngleCompResult
    {
        MoreAngle,
        LessAngle,
        Collinear
    };

    const PointInT &getPoint(pcl::index_t index) const
    {
        return input_->at(index);
    }

    double pointDist(pcl::index_t point_index_1,
                     pcl::index_t point_index_2)
    {
        auto point_1 = getPoint(point_index_1);
        auto point_2 = getPoint(point_index_2);

        auto x = point_1.x - point_2.x;
        auto y = point_1.y - point_2.y;

        return x * x + y * y;
    }

    struct AngleComparator
    {
        pcl::index_t comp_point_index;
        const GrahamHull *hull;

        AngleComparator(pcl::index_t idx, const GrahamHull *hull) :
            comp_point_index(idx),
            hull(hull)
        {}

        AngleCompResult operator()(pcl::index_t point_index_1,
                                   pcl::index_t point_index_2) const
        {
            auto &comp_p  = hull->getPoint(comp_point_index);
            auto &point_1 = hull->getPoint(point_index_1);
            auto &point_2 = hull->getPoint(point_index_2);

            auto cross = (comp_p.x - point_1.x) * (point_2.y - point_1.y) -
                         (comp_p.y - point_1.y) * (point_2.x - point_1.x);

            if (std::abs(cross) < eps)
                return AngleCompResult::Collinear;
            if (cross < 0)
                return AngleCompResult::LessAngle;

            return AngleCompResult::MoreAngle;
        }
    };

    pcl::index_t findStartPoint() const
    {
        auto cmp_min_point = [this](const auto &li, const auto &ri) -> bool {
            const auto &l = getPoint(li);
            const auto &r = getPoint(ri);

            if (l.y < r.y)
                return true;
            if (l.x < r.x)
                return true;

            return false;
        };

        auto min_it = std::min_element(indices_->begin(), indices_->end(), cmp_min_point);

        return std::distance(indices_->begin(), min_it);
    }

    void performReconstruction(PointCloud &points,
                               std::vector<pcl::Vertices> &polygons)
    {
        auto indexes = *indices_;

        // Graham scan algorithm start
        std::vector<pcl::index_t> stack;

        auto start_point_index = findStartPoint();
        std::swap(indexes[0], indexes[start_point_index]);
        stack.push_back(indexes.front());

        auto comparator = AngleComparator(indexes.front(), this);

        std::sort(indexes.begin() + 1, indexes.end(), [&comparator, this](auto point_index_1, auto point_index_2)
        {
            auto comp_res = comparator(point_index_1, point_index_2);

            if (comp_res == AngleCompResult::Collinear)
                return pointDist(point_index_1, comparator.comp_point_index) < pointDist(point_index_2, comparator.comp_point_index);

            return comp_res == AngleCompResult::LessAngle;
        });

        stack.push_back(indexes[1]);
        for (auto it = indexes.begin() + 2; it != indexes.end(); ++it)
        {
            while (stack.size() > 1)
            {
                auto top = stack.back();
                auto next_to_top = *std::prev(stack.end(), 2);

                auto comparator = AngleComparator(next_to_top, this);
                auto comp_res = comparator(*it, top);
                if (comp_res == AngleCompResult::MoreAngle || comp_res == AngleCompResult::Collinear)
                    break;

                stack.pop_back();
            }

            stack.push_back(*it);
        }

        points.reserve(stack.size());
        polygons.resize(1);
        polygons[0].vertices.reserve(stack.size());

        for (auto it : stack)
        {
            polygons[0].vertices.push_back(points.size());
            points.push_back(input_->at(it));
        }
    }
};

}
