#ifndef __GRAHAM_SCAN_H__
#define __GRAHAM_SCAN_H__

#include <algorithm>
#include <cmath>
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
    struct compCCW
    {
        pcl::index_t i_pivot;
        const GrahamHull *th;

        compCCW(pcl::index_t idx, const GrahamHull *gr) : i_pivot(idx), th(gr) {}

        double operator()(pcl::index_t i, pcl::index_t j) const
        {
          auto &pivot_p = th->get_point(i_pivot);
          auto &p_i = th->get_point(i);
          auto &p_j = th->get_point(j);

          auto cross = (pivot_p.x - p_i.x) * (p_j.y - p_i.y) - (pivot_p.y - p_i.y) * (p_j.x - p_i.x);

          return cross;
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

    typename std::vector<pcl::index_t>::iterator remove_collinear(std::vector<pcl::index_t> &indexes) const
    {
      auto pivot_p = get_point(indexes.front());
      auto prev = *std::next(indexes.begin());
      auto piv_ccw = compCCW(indexes.front(), this);


      auto check_dst = [&prev, &piv_ccw, &pivot_p, this](auto &&idx){
        auto prev_p = get_point(prev);
        auto cur_p = get_point(idx);


        if (std::abs(piv_ccw(prev, idx)) > 1e-6)
        {
          prev = idx;
          return false;
        }

        auto prev_dst = std::abs(prev_p.x - pivot_p.x) + std::abs(prev_p.y - pivot_p.y);
        auto cur_dst = std::abs(cur_p.x - pivot_p.x) + std::abs(cur_p.y - pivot_p.y);
        
        if (prev_dst < cur_dst)
          std::swap(idx, prev);

        prev = idx;
        return true;
      };

      return std::remove_if(std::next(indexes.begin(), 2), indexes.end(), check_dst);
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

      std::sort(indexes.begin() + 1, indexes.end(), [&comp_p0](auto i, auto j) { return comp_p0(i, j) < 0; });

      auto new_end = remove_collinear(indexes);

      indexes.erase(new_end, indexes.end());

      stack.push_back(indexes[1]);
      for (auto it = indexes.begin() + 2; it != indexes.end(); ++it) 
      {

        while (stack.size() > 1)
        {
          auto top = stack.back();
          auto next_to_top = *std::prev(stack.end(), 2);

          auto ccw = compCCW(next_to_top, this);

          if (ccw(*it, top) > 0)
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