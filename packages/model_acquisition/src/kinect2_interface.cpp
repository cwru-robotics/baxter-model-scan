/*
 * kinect2_interface
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 */

#include "model_acquisition/kinect2_interface.h"

ros::Subscriber g_getPointCloud;

std::string g_prev_obj_name;
std::string g_scan_topic;

std::string home = "~/.ros";

// pcl::PointCloud<pcl::PointXYZ>::Ptr p_pclKinect(new pcl::PointCloud<pcl::PointXYZ>);

uint g_snapshot_number;

Kinect2Interface::Kinect2Interface(ros::NodeHandle &nh):p_pclKinect(new pcl::PointCloud<pcl::PointXYZRGB>)
{
  nh_ = nh;
  g_snapshot_number = 0;
  // get topic for kinect2 PointCloud2
  // get PCD directory from config

  if (!nh.getParam("model_acquisition/scan_topic", g_scan_topic))
    g_scan_topic = "/kinect2/qhd/points";  // Default behavior

  g_getPointCloud = nh.subscribe<sensor_msgs::PointCloud2> (g_scan_topic, 1, &Kinect2Interface::kinectCB, this);
}

Kinect2Interface::~Kinect2Interface()
{
  // Do I need to delete things here?
}

void Kinect2Interface::kinectCB(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
  pcl::fromROSMsg(*cloud, *p_pclKinect);
  // ROS_INFO("kinectCB %d * %d points", (int) g_pclKinect->width, (int) g_pclKinect->height);
}

void Kinect2Interface::snapshot(std::string obj_name)
{
  ros::spinOnce();
  std::string file_name;

  file_name = obj_name + std::to_string(g_snapshot_number);

  pcl::io::savePCDFileASCII(file_name + ".pcd", *p_pclKinect);
  
  if (g_snapshot_number > 0 && obj_name.compare(g_prev_obj_name) != 0)
    g_snapshot_number = 0;
  else
  {
    g_snapshot_number++;
    g_prev_obj_name = obj_name;
  }
}
