/*
 * model_acquisition
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 *
 */

#include "model_acquisition/model_acquisition.h"

std::string g_robot;
std::string g_scanner;

double g_increment_degrees;
double g_increment_radians;

std::vector<double> g_scan_pose(7);
Vectorq7x1 g_vec_scan_pose;

BaxterInterface* baxter;
Kinect2Interface* kinect;

bool goToScanPose(model_acquisition::scan_pose::Request &request,
                  model_acquisition::scan_pose::Response &response)
{
  ROS_INFO("Set Scan Pose!");
  baxter->goToPose(g_vec_scan_pose, 1);

  return true;
}

bool acquireModel(model_acquisition::acquire::Request &request,
                  model_acquisition::acquire::Response &response)
{
  ROS_INFO("Acquire Model!");

  for (double d = -M_PI; d < M_PI; d += g_increment_radians)
  {
    g_vec_scan_pose(6, 0) = d;
    baxter->goToPose(g_vec_scan_pose, 1);

    ROS_INFO("snapshot");
    
    kinect->snapshot(request.model_name);
    ros::spinOnce();  // This might not be necessary since it's inside kinect2Interface::snapshot already
  }

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "model_acquisition");
  ros::NodeHandle nh;

  if (!nh.getParam("model_acquisition/robot", g_robot))
    g_robot = "robot_undefined";  // Undefined robot

  if (!nh.getParam("model_acquisition/scanner", g_scanner))
    g_scanner = "scanner_undefined";  // Undefined scanner

  if (!nh.getParam("model_acquisition/increment_degrees", g_increment_degrees))
    g_increment_degrees = 10.0;  // Default value

  if (!nh.getParam("model_acquisition/scan_left_e0", g_scan_pose[0]))
    g_scan_pose[0] = -1.22143;

  if (!nh.getParam("model_acquisition/scan_left_e1", g_scan_pose[1]))
    g_scan_pose[1] = 1.11635;

  if (!nh.getParam("model_acquisition/scan_left_s0", g_scan_pose[2]))
    g_scan_pose[2] = -0.446772;

  if (!nh.getParam("model_acquisition/scan_left_s1", g_scan_pose[3]))
    g_scan_pose[3] = 0.735544;

  if (!nh.getParam("model_acquisition/scan_left_w0", g_scan_pose[4]))
    g_scan_pose[4] = -1.00284;

  if (!nh.getParam("model_acquisition/scan_left_w1", g_scan_pose[5]))
    g_scan_pose[5] = 2.09388;

  if (!nh.getParam("model_acquisition/scan_left_w2", g_scan_pose[6]))
    g_scan_pose[6] = 0.0;

  g_increment_radians = angles::from_degrees(g_increment_degrees);

  baxter = new BaxterInterface(nh);
  kinect = new Kinect2Interface(nh);
  
  // The line order is the order in which looking at the ROS topic gives the joint angles.
  // 'left_e0', 'left_e1', 'left_s0', 'left_s1', 'left_w0', 'left_w1', 'left_w2'
  // There is a mismatch in how WSN's code interprets the joint angles vs. how rethink does.

  g_vec_scan_pose(2, 0) = g_scan_pose[0];
  g_vec_scan_pose(3, 0) = g_scan_pose[1];
  g_vec_scan_pose(0, 0) = g_scan_pose[2];
  g_vec_scan_pose(1, 0) = g_scan_pose[3];
  g_vec_scan_pose(4, 0) = g_scan_pose[4];
  g_vec_scan_pose(5, 0) = g_scan_pose[5];
  g_vec_scan_pose(6, 0) = g_scan_pose[6];

  ros::ServiceServer goToScan = nh.advertiseService("go_to_scan_pose", goToScanPose);
  ros::ServiceServer acquire = nh.advertiseService("acquire_model", acquireModel);

  ros::spin();
}
