/*
 * kinect2_interface
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#ifndef KINECT2_INTERFACE_H
#define KINECT2_INTERFACE_H

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>
#include <pcl/features/normal_3d.h>

#include <Eigen/Eigen>
#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>

#include <string>

class Kinect2Interface
{
public:
  Kinect2Interface(ros::NodeHandle &nh);
  virtual ~Kinect2Interface();

  void snapshot(std::string obj_name);

private:
  ros::NodeHandle nh_;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr p_pclKinect;

  void kinectCB(const sensor_msgs::PointCloud2ConstPtr &cloud);
};

#endif  // KINECT2_INTERFACE_H
