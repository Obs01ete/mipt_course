/**
 * MIT License
 *
 * Copyright (c) 2023 Dmitrii Khizbullin <dmitrii.khizbullin@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// Source code https://github.com/PointCloudLibrary/

#pragma once

#include <pcl/surface/convex_hull.h>
#include <memory>

namespace lidar_course {

enum class Dimension : int
{
  DIM2,
  DIM3,
  ERROR_VAL
};

template<typename PointInT>
class GrahamHull : public pcl::ConvexHull<PointInT>
{
protected:
  using pcl::ConvexHull<PointInT>::input_;
  using pcl::ConvexHull<PointInT>::indices_;
  using pcl::ConvexHull<PointInT>::initCompute;
  using pcl::ConvexHull<PointInT>::deinitCompute;

public:
  using Ptr = std::shared_ptr<pcl::ConvexHull<PointInT>>;
  using ConstPtr = std::shared_ptr<const pcl::ConvexHull<PointInT>>;

  using pcl::MeshConstruction<PointInT>::reconstruct;

  using PointCloud = pcl::PointCloud<PointInT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

  Dimension dimension_;
  pcl::index_t init_point;

  /** \brief Empty constructor. */
  GrahamHull()
    : dimension_(Dimension::DIM2)
  {}

  /** \brief Empty destructor */
  ~GrahamHull() override = default;

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
  void reconstruct(PointCloud& points, std::vector<pcl::Vertices>& polygons)
  {
    points.header = input_->header;
    if (!initCompute() || input_->points.empty() || indices_->empty()) {
      points.clear();
      return;
    }

    performReconstruction(points, polygons);

    points.width = points.size();
    points.height = 1;
    points.is_dense = true;
    deinitCompute();
  }

  /** \brief Compute a convex hull for all points given.
   * \param[out] points the resultant points lying on the convex hull.
   */
  void reconstruct(PointCloud& points);

  /** \brief Sets the dimension on the input data, 2D or 3D.
   * \param[in] dimension The dimension of the input data.  If not set, this
   * will be determined automatically.
   */
  void setDimension(int dimension)
  {
    if (dimension == 2) {
      dimension_ = Dimension::DIM2;
    } else if (dimension == 3) {
      dimension_ = Dimension::DIM3;
    }
    else
      PCL_ERROR("[pcl::%s::setDimension] Invalid input dimension specified!\n",
                getClassName().c_str());
  }

  /** \brief Class get name method. */
  std::string getClassName() const override { return ("GrahamHull"); }

  /** \brief Returns the dimensionality (2 or 3) of the calculated hull. */
  inline int getDimension() const { return static_cast<int>(dimension_) + 2; }

  double squared_dist(pcl::index_t p0, pcl::index_t p1)
  {
    auto input = *input_;
    return (input[p0].x - input[p1].x) * (input[p0].x - input[p1].x) +
           (input[p0].y - input[p1].y) * (input[p0].y - input[p1].y);
  }

  int cross_product(pcl::index_t p0, pcl::index_t p1, pcl::index_t p2)
  {
    auto input = *input_;
    double res = (input[p1].y - input[p0].y) * (input[p2].x - input[p1].x) -
                 (input[p1].x - input[p0].x) * (input[p2].y - input[p1].y);
    if (std::abs(res) < 1e-6) {
      return 0;
    }
    return res > 0 ? 1 : 2;
  }

protected:
  /** \brief The actual reconstruction method.
   *
   * \param[out] points the resultant points lying on the convex hull
   * \param[out] polygons the resultant convex hull polygons, as a set of
   * vertices. The Vertices structure contains an array of point indices.
   * \param[in] fill_polygon_data true if polygons should be filled, false
   * otherwise
   */
  void performReconstruction(PointCloud& points,
                             std::vector<pcl::Vertices>& polygons,
                             bool fill_polygon_data = false)
  {
    if (dimension_ == Dimension::DIM2) {
      performReconstruction2D(points, polygons, fill_polygon_data);
    } else {
      PCL_ERROR("[pcl::%s::setDimension] Invalid input dimension specified!\n",
                getClassName().c_str());
    }
  }

  /** \brief Finding starting index for Graham Hull */
  pcl::index_t find_initial_index()
  {
    auto indices = *indices_;
    auto input = *input_;
    size_t returned_ind = 0;
    double y, m_y = input[indices[0]].y;

    for (size_t i = 1; i < indices.size(); ++i) {
      y = input[indices[i]].y;
      if ((y < m_y) ||
          ((std::abs(y - m_y) < 1e-6) &&
           (input[indices[i]].x < input[indices[returned_ind]].x))) {
        m_y = input[indices[i]].y;
        returned_ind = i;
      }
    }
    return returned_ind;
  }

  /** \brief The reconstruction method for 2D data.  Does not require dimension
   * to be set.
   *
   * \param[out] points the resultant points lying on the convex hull
   * \param[out] polygons the resultant convex hull polygons, as a set of
   * vertices. The Vertices structure contains an array of point indices.
   * \param[in] fill_polygon_data true if polygons should be filled, false
   * otherwise
   */
  void performReconstruction2D(PointCloud& points,
                               std::vector<pcl::Vertices>& polygons,
                               bool fill_polygon_data = false)
  {
    auto indices = *indices_;
    pcl::index_t initial_index = find_initial_index();
    std::vector<pcl::index_t> stack;
    std::swap(indices[0], indices[initial_index]);
    init_point = indices[0];
    size_t ind = 1;

    std::sort(indices_->begin() + 1,
              indices_->end(),
              [this](pcl::index_t p0, pcl::index_t p1) {
                int orient = cross_product(init_point, p0, p1);
                if (orient == 0) {
                  return squared_dist(init_point, p1) >=
                         squared_dist(init_point, p0);
                }
                return orient == 2;
              });

    for (size_t i = 1; i < indices_->size(); i++) {
      while (i < indices_->size() - 1 &&
             cross_product(init_point, indices[i], indices[i + 1]) == 0) {
        ++i;
      }
      indices[ind++] = indices[i];
    }
    if (ind < 3)
      return;

    for (size_t i = 0; i < 3; ++i) {
      stack.push_back(indices[i]);
    }

    for (size_t i = 3; i < ind; i++) {
      while (stack.size() > 1 && cross_product(*std::prev(stack.end(), 2),
                                               stack.back(),
                                               indices[i]) != 2) {
        stack.pop_back();
      }

      stack.push_back(indices[i]);
    }

    polygons.resize(1);
    for (size_t i = 0; i < stack.size(); ++i) {
      polygons[0].vertices.push_back(points.size());
      points.push_back((*input_)[stack[i]]);
    }
  }

public:
  PCL_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace lidar_course