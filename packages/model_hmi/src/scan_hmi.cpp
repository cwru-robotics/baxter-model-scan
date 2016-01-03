/*
 * scan_hmi
 *
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 */

#include <ros/ros.h>

#include <string>

#include <model_acquisition/acquire.h>
#include <model_acquisition/scan_pose.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "scan_hmi");
  ros::NodeHandle nh;

  ros::ServiceClient scan_position = nh.serviceClient<model_acquisition::scan_pose>("go_to_scan_pose");
  ros::ServiceClient acquire_model = nh.serviceClient<model_acquisition::acquire>("acquire_model");

  model_acquisition::scan_pose scan_pose_srv;
  scan_pose_srv.request.request = false;

  model_acquisition::acquire acquire_srv;

  ROS_INFO("Welcome to the Baxter Model Acquisition System!");

  while (ros::ok())
  {
    int selection;

    ROS_INFO("enter 1 to assume scan position");
    ROS_INFO("enter 2 to perform model acquisition");
    ROS_INFO("enter any other number to exit");
    std::cout << std::endl;
    std::cin >> selection;

    if (selection == 1)
    {
      scan_position.call(scan_pose_srv);
    }
    else if (selection == 2)
    {
      ROS_INFO("enter a model name string: ");
      std::string name;
      std::cin >> name;

      acquire_srv.request.model_name = name;

      acquire_model.call(acquire_srv);
    }
    else
    {
      return 0;
    }

    ros::spinOnce();
  }
}
