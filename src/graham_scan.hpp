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
  using Ptr = pcl::shared_ptr<GrahamHull<PointInT> >;
  using ConstPtr = pcl::shared_ptr<const GrahamHull<PointInT> >;

  using PointCloud = pcl::PointCloud<PointInT>;
  using PointCloudPtr = typename PointCloud::Ptr;
  using PointCloudConstPtr = typename PointCloud::ConstPtr;

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
  void performReconstruction (PointCloud &points, std::vector<pcl::Vertices> &polygons,
                              bool fill_polygon_data = false)
  {
    // Graham scan
  }

};
}


#endif // __GRAHAM_SCAN_H__