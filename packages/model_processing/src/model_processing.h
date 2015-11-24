#include <string>

class ModelProcessing
{
public:

pcl::PointCloud<pcl::PointXYZRGB> pcd_reader (std::string filepath);
pcl::PointCloud<pcl::PointXYZRGB> remove_outlier (pcl::PointCloud<pcl::PointXYZRGB> cloud);
pcl::PointCloud<pcl::PointXYZRGB> downsample (pcl::PointCloud<pcl::PointXYZRGB> cloud);
float[] bounding_box (pcl::PointCloud<pcl::PointXYZRGB> cloud);
std::string pcd_writer (pcl::PointCloud<pcl::PointXYZRGB> cloud);
float get_centroid (pcl::PointCloud<pcl::PointXYZRGB> cloud);
};
