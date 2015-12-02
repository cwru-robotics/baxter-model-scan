/*
 * planar_pointcloud
 * a ros node to publish a pointcloud based off of a selection of coplanar points
 * 
 * (c) 2015 Luc Bettaieb
 */
#ifndef PLANAR_POINTCLOUD_H
#define PLANAR_POINTCLOUD_H

#include <ros/ros.h>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <Eigen/Eigen>


class PlanarPublisher
{
public:
  PlanarPublisher(ros::NodeHandle &nh);
  ~PlanarPublisher();
  ros::NodeHandle nh_; 

  std::string pc_frame;

  void cloud_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);
  void selected_pts_cb(const sensor_msgs::PointCloud2ConstPtr& cloud);

  pcl::PointCloud<pcl::PointXYZRGB> find_coplanar_above_points();

  bool ppOK();
  void ppNOK();

  std::string getPCFrame(){ return pc_frame; }
  ros::Subscriber selected_pts_sub_;
  ros::Subscriber cloud_sub_;

private:

  CwruPclUtils *utils;
  
  float COPLANAR_TOLERANCE;
  float PLANAR_TOLERANCE;

  bool selected_pts_cb_bool;
  bool got_cloud;
  
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr g_cloud_ptr;
  pcl::PointCloud<pcl::PointXYZ>::Ptr g_selected_ptr;

  Eigen::Vector3f g_plane_normal;
  double g_plane_distance;

  float selected_min_z;

  

  bool pointXYinTolerance(pcl::PointXYZRGB p1, pcl::PointXYZRGB p2);

};

#endif  // PLANAR_POINTCLOUD_H