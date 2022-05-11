#ifndef __GRAHAM_SCAN_H__
#define __GRAHAM_SCAN_H__

#include <algorithm>
#include <cmath>
#include <stack>
#include <pcl/surface/convex_hull.h>

namespace graham
{
  const long double eps = 1e-6;

  template <typename PointInT>
  class GrahamHull : public pcl::ConvexHull<PointInT>
  {
  private:
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
    struct compCCW
    {
        enum Res
        {
          CCW,
          CW,
          COL
        };

        pcl::index_t i_pivot;
        const GrahamHull *th;

        compCCW(pcl::index_t idx, const GrahamHull *gr) : i_pivot(idx), th(gr) {}

        // Return > 0 when pivot, i, j - CW
        // Return < 0 when pivot, i, j - CCW
        // Return 0 when pivot, i, j collinear
        Res operator()(pcl::index_t i, pcl::index_t j) const
        {
          auto &pivot_p = th->get_point(i_pivot);
          auto &p_i = th->get_point(i);
          auto &p_j = th->get_point(j);

          auto cross = (pivot_p.x - p_i.x) * (p_j.y - p_i.y) - (pivot_p.y - p_i.y) * (p_j.x - p_i.x);

          if (std::abs(cross) < eps)
            return COL;
          if (cross < 0)
            return CCW;
          return CW;
        }
    };

    pcl::index_t findP0() const
    {
        auto cmp_min_point = [this](auto &&li, auto &&ri) -> bool {
        const auto &l = get_point(li);
        const auto &r = get_point(ri);

        if (l.y < r.y)
          return true;
        if (l.x < r.x)
          return true;
        return false;
      };
      auto min_it = std::min_element(indices_->begin(), indices_->end(), cmp_min_point);

      return std::distance(indices_->begin(), min_it);
    }

    const PointInT &get_point(pcl::index_t index) const
    {
      return input_->at(index);
    }

    double pointDist(pcl::index_t i, pcl::index_t j)
    {
      auto pi = get_point(i);
      auto pj = get_point(j);

      auto x = pi.x - pj.x;
      auto y = pi.y - pj.y;

      return x * x + y * y;
    }

    void performReconstruction(PointCloud &points, std::vector<pcl::Vertices> &polygons,
                               bool)
    {
      std::vector<pcl::index_t> stack;

      // Graham scan
      auto P0_index_index = findP0();

      auto indexes = *indices_;
      std::swap(indexes[0], indexes[P0_index_index]);

      stack.push_back(indexes.front());

      auto comp_p0 = compCCW(indexes.front(), this);

      std::sort(indexes.begin() + 1, indexes.end(), [&comp_p0, this](auto i, auto j) {
        auto res = comp_p0(i, j);

        if (res == compCCW::COL)
          return pointDist(i, comp_p0.i_pivot) < pointDist(j, comp_p0.i_pivot);
        
         return res == compCCW::CCW;
      });

      stack.push_back(indexes[1]);
      for (auto it = indexes.begin() + 2; it != indexes.end(); ++it) 
      {
        while (stack.size() > 1)
        {
          auto top = stack.back();
          auto next_to_top = *std::prev(stack.end(), 2);

          auto ccw = compCCW(next_to_top, this);
          auto res = ccw(*it, top);
          if (res == compCCW::CW || res == compCCW::COL)
            break;

          stack.pop_back();
        }

        stack.push_back(*it);
      }

      points.reserve(stack.size());
      polygons.resize(1);
      polygons[0].vertices.reserve(stack.size());
      for (auto idx : stack)
      {
        polygons[0].vertices.push_back(points.size());
        points.push_back(input_->at(idx));
      }
    }
  };
}

#endif // __GRAHAM_SCAN_H__