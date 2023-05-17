#include <pcl/surface/convex_hull.h>
#include <iostream>
#include <stack>

namespace lidar_course {

    namespace
    {
        const long double EPS = 1e-6;
    }


    template<typename PointT>
    class GrahamHull : public pcl::ConvexHull<PointT> {
    public:
        using Ptr = boost::shared_ptr<GrahamHull<PointT>>;
        using ConstPtr = boost::shared_ptr<const GrahamHull<PointT>>;
        using PointCloud = pcl::PointCloud<PointT>;
        using PointCloudPtr = typename PointCloud::Ptr;
        using PointCloudConstPtr = typename PointCloud::ConstPtr;

        GrahamHull() : dimension_(2) {}

        void setInputCloud(const PointCloudConstPtr& cloud) {
            input_ = cloud;
        }

        void setIndices(const pcl::IndicesPtr& indices) {
            indices_ = indices;
        }

        bool checkReconstruct() {
            return !initCompute() || input_->points.empty() || indices_->empty();
        }

        void reconstruct(PointCloud& points, std::vector<pcl::Vertices>& polygons) {
            points.header = input_->header;

            if (checkReconstruct()) {
                points.clear();
                return;
            }

            performReconstruction(points, polygons, true);

            points.width = points.size();
            points.height = 1;
            points.is_dense = true;

            deinitCompute();
        }

        void reconstruct(PointCloud& points) {
            points.header = input_->header;

            if (checkReconstruct()) {
                points.clear();
                return;
            }

            std::vector<pcl::Vertices> polygons;
            performReconstruction(points, polygons, false);

            points.width = points.size();
            points.height = 1;
            points.is_dense = true;

            deinitCompute();
        }

        void setDimension(int dimension) {
            if (dimension != 2) {
                std::cerr << "[In setDimension] Invalid dimension (only 2 is acceptable)" << std::endl;
                return
            }
            dimension_ = dimension;
        }

        int getDimension() const {
            return dimension_;
        }

    protected:

        enum class AngleCompResult
        {
            Collinear,
            LessAngle,
            MoreAngle,
        };

        void performReconstruction(PointCloud& points, std::vector<pcl::Vertices>& polygons, bool fill_polygon_data) {   
            std::stack<pcl::index_t> hull_stack;
            pcl::index_t start_index = getStartIndex();
            std::swap((*indices_)[0], (*indices_)[start_index]);

            hull_stack.push(indices_->at(0));


            std::sort(indices_->begin() + 1, indices_->end(), [this](pcl::index_t i, pcl::index_t j) {
                return this->compare(i,j);
            });

            hull_stack.push(indices_->at(1));

            for (std::size_t i = 2; i < indices_->size(); ++i) {
                while (hull_stack.size() > 1) {
                    pcl::index_t p0 = hull_stack.top();
                    hull_stack.pop();
                    pcl::index_t p1 = hull_stack.top();
                    pcl::index_t p2 = indices_->at(i);
                    if (getOrientation(p1, p0, p2) == AngleCompResult::MoreAngle) {
                        hull_stack.push(p0);
                        break;
                    }
                }
                hull_stack.push(indices_->at(i));
            }

            points.resize(hull_stack.size());
            polygons.resize(1);
            polygons[0].vertices.reserve(hull_stack.size());

            std::size_t i = hull_stack.size() - 1;
            for (std::size_t i = 0; i < hull_stack.size(); i++)
            {
                polygons[0].vertices.push_back(points.size());
                points.push_back(input_->at(i));
            }             
        }

        pcl::index_t getStartIndex() {
            float ymin = input_->points[indices_->at(0)].y;
            pcl::index_t min = 0;
            for (std::size_t i = 1; i < indices_->size(); ++i) {
                float y = input_->points[indices_->at(i)].y;
                if (y < ymin || (input_->points[indices_->at(i)].x < input_->points[indices_->at(min)].x)) {
                    ymin = y;
                    min = i;
                }
            }
            return min;
        }

        int getOrientation(pcl::index_t p0, pcl::index_t p1, pcl::index_t p2) {
            float val = (input_->points[p1].y - input_->points[p0].y) *
                        (input_->points[p2].x - input_->points[p1].x) -
                        (input_->points[p1].x - input_->points[p0].x) *
                        (input_->points[p2].y - input_->points[p1].y);
            if (std::abs(val) < EPS)
                return AngleCompResult::Collinear;
            return (val < 0) ? AngleCompResult::LessAngle : AngleCompResult::MoreAngle;
        }

        float getDistance(pcl::index_t pointI1, pcl::index_t pointI2)
        {
            auto point1 = (*input_)[pointI1];
            auto point2 = (*input_)[pointI2];
            auto delta_x = point1.x - point2.x;
            auto delta_y = point1.y - point2.y;
            return delta_x * delta_x + delta_y * delta_y;
        }

        bool cmp(pcl::index_t p1, pcl::index_t p2) {
            int orient = getOrientation(p0_, p1, p2);
            if (orient == AngleCompResult::Collinear)
                return getDistance(p0_, p2) >= getDistance(p0_, p1);

            return orient == AngleCompResult::MoreAngle;
        }

        bool polarAngleComparison(pcl::index_t i, pcl::index_t j) {
            float angle_i = atan2(input_->points[i].y, input_->points[i].x);
            float angle_j = atan2(input_->points[j].y, input_->points[j].x);
            return (angle_i < angle_j);
        }

        PointCloudConstPtr input_;
        pcl::IndicesPtr indices_;
        int dimension_;
};

} //namespace lidar_course