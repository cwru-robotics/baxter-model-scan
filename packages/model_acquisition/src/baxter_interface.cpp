/* 
 * baxter_interface
 * 
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 */

#include "model_acquisition/baxter_interface.h"

#include <vector>
#include <control_msgs/FollowJointTrajectoryAction.h>

ros::Publisher g_LeftJointPublisher;
ros::Subscriber g_LeftJointListener;

baxter_core_msgs::JointCommand left_cmd;
double leftJointAngles [7];

BaxterInterface::BaxterInterface(ros::NodeHandle &nh)
{
  // load in increment angle from parameter server
  // make sure its divisible by 360, if not, set to closest angle that is divisible by 360
  nh_ = nh;

  left_cmd.mode = baxter_core_msgs::JointCommand::RAW_POSITION_MODE;
  left_cmd.names.push_back("left_s0");
  left_cmd.names.push_back("left_s1");
  left_cmd.names.push_back("left_e0");
  left_cmd.names.push_back("left_e1");
  left_cmd.names.push_back("left_w0");
  left_cmd.names.push_back("left_w1");
  left_cmd.names.push_back("left_w2");

  left_cmd.command.resize(7, 0.0);

  g_LeftJointPublisher = nh_.advertise<baxter_core_msgs::JointCommand>("/robot/limb/left/joint_command", 1);
  g_LeftJointListener = nh_.subscribe("/robot/joint_states", 3, &BaxterInterface::updateLeftJointAngles, this);
}

BaxterInterface::~BaxterInterface()
{
}

void BaxterInterface::doneCb(const actionlib::SimpleClientGoalState& state,
                             const baxter_traj_streamer::trajResultConstPtr& result)
{
  ROS_INFO(" doneCb: server responded with state [%s]", state.toString().c_str());
  ROS_INFO("got return val = %d; traj_id = %d", result->return_val, result->traj_id);
}

Vectorq7x1 BaxterInterface::getLeftArmPose()
{
  Vectorq7x1 pose;

  pose(2, 0) = leftJointAngles[0];
  pose(3, 0) = leftJointAngles[1];
  pose(0, 0) = leftJointAngles[2];
  pose(1, 0) = leftJointAngles[3];
  pose(4, 0) = leftJointAngles[4];
  pose(5, 0) = leftJointAngles[5];
  pose(6, 0) = leftJointAngles[6];

  return pose;
}


void BaxterInterface::updateLeftJointAngles(const sensor_msgs::JointState& jointstate)
{
  for (uint i = 0; i < 7; i++)
  {
    leftJointAngles[i] = jointstate.position.at(i+2);
  }
}

bool BaxterInterface::setJointToAngle(int joint, double angle)
{
  for (uint i = 0; i < 7; i++)
  {
    left_cmd.command[i] = leftJointAngles[i];
  }

  // left_cmd.command.at(joint) = angle;
  g_LeftJointPublisher.publish(left_cmd);
}

bool BaxterInterface::goToPose(Vectorq7x1 pose, int lr)
{
  uint g_count = 0;
  uint ans;
  Eigen::VectorXd q_in_vecxd;

  Vectorq7x1 q_vec_left_arm;

  std::vector<Eigen::VectorXd> des_path;

  trajectory_msgs::JointTrajectory des_trajectory;

  ROS_DEBUG("Instantiating a traj streamer");
  Baxter_traj_streamer ts(&nh_);

  ROS_DEBUG("Warming up callbacks");  // Is this necessary?  Test with and without it.
  for (uint i = 0; i < 100; i++)
  {
    ros::spinOnce();
    ros::Duration(0.01).sleep();
  }

  ROS_DEBUG("Getting current pose.");

  for (uint i = 0; i < 7; i++)
  {
    q_vec_left_arm = getLeftArmPose();  // NEED TO MAKE WORK FOR RIGHT ARM
  }

  q_in_vecxd = q_vec_left_arm;
  des_path.push_back(q_in_vecxd);

  q_in_vecxd = pose;
  des_path.push_back(q_in_vecxd);

  ts.stuff_trajectory(des_path, des_trajectory);

  baxter_traj_streamer::trajGoal goal;
  goal.trajectory = des_trajectory;

  actionlib::SimpleActionClient<baxter_traj_streamer::trajAction> action_client("trajActionServer", true);

  ROS_DEBUG("Waiting for server: ");

  if (!action_client.waitForServer(ros::Duration(5.0)))
  {
    ROS_WARN("Could not connect to server.");
    return false;
  }

  ROS_DEBUG("Connected to action server");

  g_count++;
  goal.traj_id = g_count;

  if (lr != 1 && lr != 0)
  {
    ROS_WARN("left or right arm not selected.  halting.");
    return false;
  }

  goal.left_or_right = lr;

  ROS_DEBUG("Sending traj_id %d", g_count);

  action_client.sendGoal(goal, boost::bind(&BaxterInterface::doneCb, this, _1, _2));

  if (!action_client.waitForResult(ros::Duration(5.0)))
  {
    ROS_WARN("Giving up waiting on result for traj_id: %d", g_count);
    return false;
  }
  else
  {
    ROS_DEBUG("Finished before timeout!");
  }

  return true;
}
