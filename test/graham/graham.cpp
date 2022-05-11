#include <iostream>
#include "graham_scan.hpp"

int main()
{
  auto flat_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

  flat_cloud_ptr->push_back(pcl::PointXYZ(1, 0, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(1, 1, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(1, 2, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(1, 3, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(1, 4, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(1, 5, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(2, 6, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(0, 6, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(0, 2, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(0, 0, 0));
  flat_cloud_ptr->push_back(pcl::PointXYZ(2, 0, 0));

  auto flat_hull_cloud_ptr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  auto flat_polygons_ptr = std::make_shared<std::vector<pcl::Vertices>>();

  graham::GrahamHull<pcl::PointXYZ> graham_hull;
  graham_hull.setInputCloud(flat_cloud_ptr);
  graham_hull.reconstruct(*flat_hull_cloud_ptr, *flat_polygons_ptr);

  for (auto idx : flat_polygons_ptr->at(0).vertices)
  {
    auto pt = flat_hull_cloud_ptr->at(idx);
    std::cout << pt << std::endl;
  }
  return 0;
}