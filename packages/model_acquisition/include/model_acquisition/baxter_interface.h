/* 
 * baxter_interface
 * 
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 * 
 */

#ifndef BAXTER_INTERFACE_H
#define BAXTER_INTERFACE_H

#include "model_acquisition/abstract_robot.h"

#include <baxter_core_msgs/JointCommand.h>
#include <sensor_msgs/JointState.h>
#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <baxter_traj_streamer/baxter_traj_streamer.h>

#include <baxter_traj_streamer/trajAction.h>

#define VECTOR_DIM 7  // e.g., a 7-dof vector

class BaxterInterface : public AbstractRobot
{
public:
  BaxterInterface(ros::NodeHandle &nh);
  virtual ~BaxterInterface();

private:
  ros::NodeHandle nh_;
  // Vectorq7x1 scan_pose;

  virtual void updateLeftJointAngles(const sensor_msgs::JointState& jointstate);

  virtual void doneCb(const actionlib::SimpleClientGoalState& state,
                      const baxter_traj_streamer::trajResultConstPtr& result);

public:

  virtual Vectorq7x1 getLeftArmPose();

  virtual bool goToPose(Vectorq7x1 pose, int lr);

  virtual bool setJointToAngle(int joint, double angle);
};

#endif  // BAXTER_INTERFACE_H
