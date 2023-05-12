/**
 * MIT License
 *
 * Copyright (c) 2023 Dmitriy Nadykto <nadykto.dmitry@gmail.com>,
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
#include <iostream>
#include <stack>

namespace lidar_course
{
    template<typename PointInT>
    class GrahamHull : public pcl::ConvexHull<PointInT>
    {
    protected:
        using pcl::ConvexHull<PointInT>::input_;
        using pcl::ConvexHull<PointInT>::indices_;
        using pcl::ConvexHull<PointInT>::initCompute;
        using pcl::ConvexHull<PointInT>::deinitCompute;

    public:
        using Ptr = pcl::shared_ptr<GrahamHull<PointInT> >;
        using ConstPtr = pcl::shared_ptr<const GrahamHull<PointInT> >;

        using PointCloud = pcl::PointCloud<PointInT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        /** \brief Empty constructor. */
        GrahamHull () : dimension_ (2)
        {
        }

        /** \brief Empty destructor */
        ~GrahamHull () override = default;

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
        void
        reconstruct (PointCloud &points,
                     std::vector<pcl::Vertices> &polygons)
        {
            points.header = input_->header;
            if (!initCompute () || input_->points.empty () || indices_->empty ())
            {
                points.clear ();
                return;
            }
            // Perform the actual surface reconstruction
            performReconstruction (points, polygons, true);
            points.width = points.size ();
            points.height = 1;
            points.is_dense = true;

            deinitCompute ();
        }

        /** \brief Compute a convex hull for all points given.
          * \param[out] points the resultant points lying on the convex hull.
          */
        void
        reconstruct (PointCloud &points)
        {
            points.header = input_->header;
            if (!initCompute () || input_->points.empty () || indices_->empty ())
            {
                points.clear ();
                return;
            }

            // Perform the actual surface reconstruction
            std::vector<pcl::Vertices> polygons;
            performReconstruction (points, polygons, false);

            points.width = points.size ();
            points.height = 1;
            points.is_dense = true;

            deinitCompute ();
        }

        /** \brief Sets the dimension on the input data, 2D or 3D.
          * \param[in] dimension The dimension of the input data.  If not set, this will be determined automatically.
          */
        void
        setDimension (int dimension)
        {
            if ((dimension == 2) || (dimension == 3))
                dimension_ = dimension;
            else
                PCL_ERROR ("[pcl::%s::setDimension] Invalid input dimension specified!\n", getClassName ().c_str ());
        }

        /** \brief Returns the dimensionality (2 or 3) of the calculated hull. */
        inline int
        getDimension () const
        {
            return (dimension_);
        }

    protected:
        /** \brief The actual reconstruction method.
          *
          * \param[out] points the resultant points lying on the convex hull
          * \param[out] polygons the resultant convex hull polygons, as a set of
          * vertices. The Vertices structure contains an array of point indices.
          * \param[in] fill_polygon_data true if polygons should be filled, false otherwise
          */
        void
        performReconstruction (PointCloud &points,
                               std::vector<pcl::Vertices> &polygons,
                               bool fill_polygon_data = false)
        {
//                if (dimension_ == 0)
//                    calculateInputDimension ();
            if (dimension_ == 2)
                performReconstruction2D (points, polygons, fill_polygon_data);
//                else if (dimension_ == 3)
//                    performReconstruction3D (points, polygons, fill_polygon_data);
            else
                PCL_ERROR ("[pcl::%s::performReconstruction] Error: invalid input dimension requested: %d\n",getClassName ().c_str (),dimension_);
        }

        /** \brief The reconstruction method for 2D data.  Does not require dimension to be set.
          *
          * \param[out] points the resultant points lying on the convex hull
          * \param[out] polygons the resultant convex hull polygons, as a set of
          * vertices. The Vertices structure contains an array of point indices.
          * \param[in] fill_polygon_data true if polygons should be filled, false otherwise
          */
        pcl::index_t
        getStartIndex ()
        {
            float ymin = ((*input_)[(*indices_)[0]].y);
            int min = 0;
            for (std::size_t i = 1; i < indices_->size(); i++) {
                float y = (*input_)[(*indices_)[i]].y;

                if ((y < ymin) ||
                    (((*input_)[(*indices_)[i]].x < (*input_)[(*indices_)[min]].x)))
                {
                    ymin = (*input_)[(*indices_)[i]].y;
                    min = i;
                }
            }
            return min;
        }

        /** \brief Finding orientation of ordered triplet (p0, p1, p2).
          * 0 --> collinear
          * 1 --> clockwise
          * 2 --> counterclockwise
          *
          * \param[in] p0, p1, p2 indexes of point indices to use.
          */
        int
        getOrientation (pcl::index_t p0, pcl::index_t p1, pcl::index_t p2)
        {
            float val = ((*input_)[p1].y - (*input_)[p0].y) *
                      ((*input_)[p2].x - (*input_)[p1].x) -
                      ((*input_)[p1].x - (*input_)[p0].x) *
                      ((*input_)[p2].y - (*input_)[p1].y);
            if (std::abs(val) < 0.000001) return 0;
            return (val > 0) ? 1 : 2;
        }

        /** \brief Calculate distance between two points
          *
          * \param[in] p1, p2 indexes of point indices to use.
          */
        float
        getDistance(pcl::index_t p1, pcl::index_t p2)
        {
            return ((*input_)[p1].x - (*input_)[p2].x) *
                   ((*input_)[p1].x - (*input_)[p2].x) +
                   ((*input_)[p1].y - (*input_)[p2].y) *
                   ((*input_)[p1].y - (*input_)[p2].y);
        }

        /** \brief Comparator for sorting point in the Graham scan
          *
          * \param[in] p1, p2 indexes of point indices to use.
          */
        bool
        compare (pcl::index_t p1, pcl::index_t p2)
        {
            int orient = getOrientation(p0_, p1, p2);
            if (orient == 0)
                return getDistance(p0_, p2) >= getDistance(p0_, p1);

            return orient == 2;
        }

        /** \brief The reconstruction method for 2D data.  Does not require dimension to be set.
          *
          * \param[out] points the resultant points lying on the convex hull
          * \param[out] polygons the resultant convex hull polygons, as a set of
          * vertices. The Vertices structure contains an array of point indices.
          * \param[in] fill_polygon_data true if polygons should be filled, false otherwise
          */
        void
        performReconstruction2D (PointCloud &points,
                                 std::vector<pcl::Vertices> &polygons,
                                 bool fill_polygon_data = false)
        {
            std::vector<pcl::index_t> stack;

            pcl::index_t start_index = getStartIndex();
            std::swap((*indices_)[0], (*indices_)[start_index]);
            p0_ = (*indices_)[0];

            std::sort(indices_->begin() + 1, indices_->end(), [this](pcl::index_t a, pcl::index_t b) { return compare(a,b); });

            int m = 1;
            for (std::size_t i = 1; i < indices_->size(); i++)
            {
                while (i < indices_->size() - 1 && getOrientation(p0_, (*indices_)[i], (*indices_)[i + 1]) == 0)
                    i++;

                (*indices_)[m] = (*indices_)[i];
                m++;
            }

            if (m < 3) return;

            stack.push_back((*indices_)[0]);
            stack.push_back((*indices_)[1]);
            stack.push_back((*indices_)[2]);

            for (std::size_t i = 3; i < m; i++)
            {
                while (stack.size() > 1 && getOrientation(*std::prev(stack.end(), 2), stack.back(), (*indices_)[i]) != 2)
                    stack.pop_back();

                stack.push_back((*indices_)[i]);
            }

            points.reserve(stack.size());
            polygons.resize(1);
            polygons[0].vertices.reserve(stack.size());

            for (std::size_t i = 0; i < stack.size(); i++)
            {
                polygons[0].vertices.push_back(points.size());
                points.push_back((*input_)[stack[i]]);
            }
        }

        /** \brief Automatically determines the dimension of input data - 2D or 3D. */
        void
        calculateInputDimension ();

        /** \brief Class get name method. */
        std::string
        getClassName () const override
        {
            return ("GrahamHull");
        }

        /** \brief The dimensionality of the concave hull (2D or 3D). */
        int dimension_;

        /** \brief Start point of the Graham hull. */
        pcl::index_t p0_;


    public:
        PCL_MAKE_ALIGNED_OPERATOR_NEW
    };
}