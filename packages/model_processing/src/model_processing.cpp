#ifndef MODEL_PROCESSING_H
#define MODEL_PROCESSING_H

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/projection_matrix.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/impl/common.hpp>
#include <model_processing/model_processing.h>


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelProcessing::pcd_reader(std::string filepath)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  // Fill in the cloud data
  pcl::PCDReader reader;
  // Replace the path below with the path where you saved your file
  reader.read<pcl::PointXYZRGB> (filepath, *cloud);

return cloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelProcessing::remove_outlier (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cerr << "Cloud before filtering: " << std::endl;
  std::cerr << cloud << std::endl;

  // Create the filtering object
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setMeanK (50);
  sor.setStddevMulThresh (1.0);
  sor.filter (*cloud_filtered);

  std::cerr << "Cloud after filtering: " << std::endl;
  std::cerr << *cloud_filtered << std::endl;

return cloud_filtered;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelProcessing::downsampler (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height 
       << " data points (" << pcl::getFieldsList (*cloud) << ").";

  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.filter (*cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height 
       << " data points (" << pcl::getFieldsList (*cloud_filtered) << ").";

  return cloud_filtered;
}

float* ModelProcessing::bounding_box (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

pcl::PointXYZRGB min;// = (new pcl::PointXYZRGB());
pcl::PointXYZRGB max;// = (new pcl::PointXYZRGB());

 pcl::getMinMax3D (*cloud, min, max);

float minMax[6];

float x_min = min.x;
float y_min = min.y;
float z_min = min.z;
float x_max = max.x;
float y_max = max.y;
float z_max = max.z;

minMax[0]=x_min;
minMax[1]=y_min;
minMax[2]=z_min;
minMax[3]=x_max;
minMax[4]=y_max;
minMax[5]=z_max;

return minMax;
}


Eigen::Vector3f ModelProcessing::computeCentroid(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud) {
    Eigen::Vector3f centroid;
    centroid << 0, 0, 0;

    int size = pcl_cloud->width * pcl_cloud->height;
    std::cout << "frame: " << pcl_cloud->header.frame_id << std::endl;
    for (size_t i = 0; i != size; ++i) {
        centroid += pcl_cloud->points[i].getVector3fMap();
    }
    if (size > 0) {
        centroid /= ((float) size);
    }
    return centroid;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ModelProcessing::object_identification (pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
  // Read in the cloud data
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB>);

  std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl; //*

  // Create the segmentation object for the planar model and set all the parameters
  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PCDWriter writer;
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_PLANE);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.02);

  int i=0, nr_points = (int) cloud->points.size ();
  while (cloud->points.size () > 0.3 * nr_points)
  {
    // Segment the largest planar component from the remaining cloud
    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (false);

    // Get the points associated with the planar surface
    extract.filter (*cloud_plane);
    std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

    // Remove the planar inliers, extract the rest
    extract.setNegative (true);
    extract.filter (*cloud_f);
    *cloud = *cloud_f;
  }

  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
  ec.setClusterTolerance (0.02); // 2cm
  ec.setMinClusterSize (100);
  ec.setMaxClusterSize (25000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  int j = 0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr the_real_object (new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
    for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud->points[*pit]); //*
    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    std::stringstream ss;
    ss << "cloud_cluster_" << j << ".pcd";
    writer.write<pcl::PointXYZRGB> (ss.str (), *cloud_cluster, false); //*
    j++;

    if (cloud_cluster->points.size() < 10000 && cloud_cluster->points.size() > 2000){
	the_real_object = cloud_cluster;
 	}
}
  return the_real_object;
 
}

std::string ModelProcessing::pcd_writer(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string filepath) {
       
//index strings

std::string Name = "";
for (int i = filepath.length() - 1; i >= 0; i--) {
	
  if (filepath[i] == '/') {
       break;
     }   

  if (i < filepath.length() - 4) {
   	Name = filepath[i] + Name;
     }
}

std::string fileName = Name + "_processed.pcd";

pcl::PCDWriter writer;
writer.write<pcl::PointXYZRGB> (fileName, *cloud, false);

return fileName;

}


#endif
