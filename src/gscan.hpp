#ifndef __SRC_GSCAN_HH__
#define __SRC_GSCAN_HH__

#include <algorithm>
#include <cmath>
#include <stack>

#include <pcl/surface/convex_hull.h>

namespace graham {
constexpr long double kEps = 1e-6;

using PointT = pcl::PointXYZ;

class GrahamHull final : public pcl::ConvexHull<PointT> {
private:
  using pcl::ConvexHull<PointT>::input_;
  using pcl::ConvexHull<PointT>::indices_;

public:
  using pcl::ConvexHull<PointT>::setInputCloud;
  using PointCloud = pcl::PointCloud<PointT>;

  GrahamHull() = default;
  ~GrahamHull() override = default;

  void reconstruct(PointCloud &points, std::vector<pcl::Vertices> &polygons) {
    points.header = input_->header;
    if (!pcl::ConvexHull<PointT>::initCompute() || input_->points.empty() ||
        indices_->empty()) {
      points.clear();
      return;
    }

    // Perform the actual surface reconstruction
    performReconstruction(points, polygons, true);

    points.width = points.size();
    points.height = 1;
    points.is_dense = true;

    pcl::ConvexHull<PointT>::deinitCompute();
  }

private:
  struct AngleComp {
    enum Res {
      CCW, // counter clockwise
      CW,  // clockwise
      COL  // collinear
    };

    pcl::index_t origin;
    const GrahamHull *gr;

    AngleComp(pcl::index_t idx, const GrahamHull *grahamHull)
        : origin(idx), gr(grahamHull) {}

    Res operator()(pcl::index_t i, pcl::index_t j) const {
      auto &p_o = gr->get_pt(origin);
      auto &p_i = gr->get_pt(i);
      auto &p_j = gr->get_pt(j);

      auto crossProduct =
          (p_o.x - p_i.x) * (p_j.y - p_i.y) - (p_o.y - p_i.y) * (p_j.x - p_i.x);

      if (std::abs(crossProduct) < kEps)
        return COL;
      if (crossProduct < 0)
        return CCW;
      return CW;
    }
  };

  pcl::index_t findLeftBottom() const {
    auto it = std::min_element(indices_->begin(), indices_->end(),
                               [this](auto &lind, auto &rind) -> bool {
                                 auto &l = get_pt(lind);
                                 auto &r = get_pt(rind);

                                 return (l.y < r.y) || (l.x < r.x);
                               });

    return std::distance(indices_->begin(), it);
  }

  const PointT &get_pt(pcl::index_t index) const { return input_->at(index); }

  double pointDist(pcl::index_t i, pcl::index_t j) {
    auto pi = get_pt(i);
    auto pj = get_pt(j);

    auto x = pi.x - pj.x;
    auto y = pi.y - pj.y;

    return x * x + y * y;
  }

  void performReconstruction(PointCloud &points,
                             std::vector<pcl::Vertices> &polygons,
                             bool /* unused */) {
    /* prepare */
    auto indexes = *indices_;
    auto leftBottomIdx = findLeftBottom();
    std::swap(indexes[0], indexes[leftBottomIdx]);

    std::vector<pcl::index_t> stack{};
    stack.push_back(indexes.front());

    { /* sort by angle relative to left bottom point */
      auto cmp = AngleComp(indexes.front(), this);
      std::sort(
          indexes.begin() + 1, indexes.end(), [&cmp, this](auto i, auto j) {
            auto res = cmp(i, j);
            if (res == AngleComp::COL)
              return (pointDist(i, cmp.origin) < pointDist(j, cmp.origin));

            return (res == AngleComp::CCW);
          });
    }

    { /* find rightmost point */
      stack.push_back(indexes[1]);
      std::for_each(indexes.begin() + 2, indexes.end(), [&](auto idx) {
        while (stack.size() > 1) {
          auto top = *std::prev(stack.end(), 1);
          auto b4_top = *std::prev(stack.end(), 2);

          auto ccw = AngleComp(b4_top, this);
          auto res = ccw(idx, top);
          if (res == AngleComp::CW || res == AngleComp::COL)
            break;

          stack.pop_back();
        }

        stack.push_back(idx);
      });
    }

    /* modify input */
    points.reserve(stack.size());
    polygons.resize(1);
    polygons.front().vertices.reserve(stack.size());
    for (auto idx : stack) {
      polygons.front().vertices.push_back(points.size());
      points.push_back(input_->at(idx));
    }
  }
};
} // namespace graham

#endif // __SRC_GSCAN_HH__
