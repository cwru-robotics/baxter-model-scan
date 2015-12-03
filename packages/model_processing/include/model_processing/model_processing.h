#include <string>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <pcl/common/impl/common.hpp>

class ModelProcessing
{
public:

pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcd_reader (std::string filepath);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr remove_outlier (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampler (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
float* bounding_box (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
Eigen::Vector3f computeCentroid (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr object_identification (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
std::string pcd_writer (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filepath);
};
