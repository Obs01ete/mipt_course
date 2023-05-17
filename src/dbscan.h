#include <cstdlib>
#include <vector>
#include <deque>

#include <Eigen/Geometry>
#include <pcl/segmentation/cpc_segmentation.h>
#include <pcl/kdtree/kdtree.h>


namespace lidar_course {
    static constexpr const size_t UNKNOWN = 0;
    static constexpr const size_t OUTLIER = 1;
    static constexpr const float EPS = 0.3f;
    static constexpr const size_t MIN_NEIGHBORS = 5;

    inline pcl::PointCloud<pcl::PointXYZL>::Ptr InitLabeledCloud(
            pcl::PointCloud<pcl::PointXYZ>::Ptr sourceCloud) {
        
        pcl::PointCloud<pcl::PointXYZL>::Ptr destCloud = 
            std::make_shared<pcl::PointCloud<pcl::PointXYZL>>();

        pcl::copyPointCloud(*sourceCloud, *destCloud);

        for(auto& point : *destCloud) {
            point.label = UNKNOWN;
        }

        return destCloud;
    }

    inline void RadiusSearch(pcl::KdTreeFLANN<pcl::PointXYZ> const& kdtree,
            pcl::PointXYZL point, 
            std::vector<pcl::index_t>& neighbors, 
            std::vector<float>& distance) {
        neighbors.clear();
        distance.clear();
        
        // damn, why there is no implicit convertion?
        kdtree.radiusSearch(pcl::PointXYZ(point.x, point.y, point.z), 
            EPS, neighbors, distance);
    }

    void QueuePoints(std::deque<pcl::PointXYZL*>& queue, 
            pcl::PointCloud<pcl::PointXYZL>::Ptr& cloud,
            std::vector<pcl::index_t> neighbors, size_t noize) {
        for (const auto& index : neighbors){
            pcl::PointXYZL* cur_point = &(cloud->points[index]);
            if(cur_point->label <= noize) {
                queue.push_back(cur_point);
            }
        }
    }

    inline void ParseQueue(std::deque<pcl::PointXYZL*> queue, 
            pcl::PointCloud<pcl::PointXYZL>::Ptr cloud,
            std::vector<pcl::index_t>& neighbors, 
            std::vector<float>& distance,
            pcl::KdTreeFLANN<pcl::PointXYZ>& kdtree, 
            size_t segnum) {
        while(!queue.empty()) {
            pcl::PointXYZL* cur_point = queue.front();
            queue.pop_front();

            if(cur_point->label == OUTLIER) {
                cur_point->label = segnum;
            }
            if(cur_point->label != UNKNOWN) {
                continue;
            }
            cur_point->label = segnum;

            RadiusSearch(kdtree, *cur_point, neighbors, distance);

            if(neighbors.size() >= MIN_NEIGHBORS) {
                QueuePoints(queue, cloud, neighbors, OUTLIER);
            }
        }
    }

    inline void FixNumeration(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud) {
        for(auto& point : *cloud) {
            if(point.label > 0) point.label--;
        }
    }

    inline pcl::PointCloud<pcl::PointXYZL>::Ptr DBScanSegmentation(
            pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud) {

        pcl::PointCloud<pcl::PointXYZL>::Ptr labeled_cloud = 
            InitLabeledCloud(inputCloud);

        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(inputCloud);

        size_t segnum = OUTLIER;
        std::vector<pcl::index_t> neighbors;
        std::vector<float> distance;
        std::deque<pcl::PointXYZL*> queue;
        for(auto& point : *labeled_cloud) {
            if(point.label != UNKNOWN) {
                continue;
            }

            RadiusSearch(kdtree, point, neighbors, distance);
            if(neighbors.size() < MIN_NEIGHBORS) {
                point.label = OUTLIER;
                continue;
            }

            segnum++;
            point.label = segnum;

            queue.clear();
            QueuePoints(queue, labeled_cloud, neighbors, SIZE_MAX);
            
            ParseQueue(queue, labeled_cloud, neighbors, distance, 
                kdtree, segnum);
        }

        FixNumeration(labeled_cloud);

        return labeled_cloud;
    }
} //namespace lidar_course
