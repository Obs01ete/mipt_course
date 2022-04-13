#ifndef __GRAHAM_SCAN_H__
#define __GRAHAM_SCAN_H__

#include <stack>
#include <pcl/surface/convex_hull.h>

namespace graham
{

  template <typename PointInT>
  class GrahamHull : public pcl::ConvexHull<PointInT>
  {
  protected:
    using pcl::ConvexHull<PointInT>::input_;
    using pcl::ConvexHull<PointInT>::indices_;
    using pcl::ConvexHull<PointInT>::initCompute;
    using pcl::ConvexHull<PointInT>::deinitCompute;

  public:
    using Ptr = pcl::shared_ptr<GrahamHull<PointInT>>;
    using ConstPtr = pcl::shared_ptr<const GrahamHull<PointInT>>;

    using PointCloud = pcl::PointCloud<PointInT>;
    using PointCloudPtr = typename PointCloud::Ptr;
    using PointCloudConstPtr = typename PointCloud::ConstPtr;

    using pcl::ConvexHull<PointInT>::setInputCloud;

    GrahamHull() = default;
    ~GrahamHull() = default;

    void reconstruct(PointCloud &points, std::vector<pcl::Vertices> &polygons)
    {
      points.header = input_->header;
      if (!initCompute() || input_->points.empty() || indices_->empty())
      {
        points.clear();
        return;
      }

      // Perform the actual surface reconstruction
      performReconstruction(points, polygons, true);

      points.width = points.size();
      points.height = 1;
      points.is_dense = true;

      deinitCompute();
    }

  private:
    pcl::index_t findP0() const
    {
      return {};
    }
    const PointInT &get_point(pcl::index_t index) const
    {
      return input_->at(indicies_->at(index));
    }

    void performReconstruction(PointCloud &points, std::vector<pcl::Vertices> &polygons,
                               bool)
    {
      std::vector<pcl::index_t> stack;

      // Graham scan
      auto P0_index_index = findP0();

      auto indexes = *indicies_;
      std::swap((*indexes)[0], (*indexes)[P0_index_index]);
      stack.push_back(indicies_->front());

      struct compCCW
      {
        index_t i_pivot;

        bool operator()(index_t i, index_t j) const
        {
          auto &pivot_p = get_point(i_pivot);
          auto &p_i = get_point(i);
          auto &p_j = get_point(j);

          auto cross = (pivot_p.x - p_i.x) * (p_j.y - p_i.y) - (pivot_p.y - p_i.y) * (p_j.x - p_i.y);

          return cross > 0;
        }
      };

      std::sort(indexes.begin() + 1, indexes.end(), compCCW(P0_index));

      // remove_collinear()

      stack.push_back(indicies_->at(1));
      for (auto it = indexes.begin() + 2; it != indexes.end(); ++it) 
      {

        for (;;)
        {
          auto top = stack.back();
          auto next_to_top = *std::prev(stack.end(), 2);

          auto ccw = compCCW(next_to_top);

          if (stack.size() <= 1 || ccw(top, *it))
            break;

          stack.pop_back();
        }

        stack.push_back(*it);
      }

      points.reserve(stack.size());
      polygons.resize(1);
      polygons[0].reserve(stack.size())
      for (auto idx : stack)
      {
        polygons[0].push_back(points.size());
        points.push_back(input_->at(idx))
      }
    }
  };
}

#endif // __GRAHAM_SCAN_H__