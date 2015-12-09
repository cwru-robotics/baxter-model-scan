#ifndef PCD_WATCHER_PCD_WATCHER_SERVER_H
#define PCD_WATCHER_PCD_WATCHER_SERVER_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <pcd_watcher/new_pcdAction.h>
#include <std_msgs/String.h>

class PcdWatcherServer
{
public:
    PcdWatcherServer();
    void newPcdCB(const actionlib::SimpleActionServer<pcd_watcher::new_pcdAction>::GoalConstPtr& goal);

private:
    ros::NodeHandle nh;
    actionlib::SimpleActionServer<pcd_watcher::new_pcdAction> actionServer;
    pcd_watcher::new_pcdGoal goal;
    pcd_watcher::new_pcdResult result;
    std_msgs::String feedback;
};
#endif  // PCD_WATCHER_PCD_WATCHER_SERVER_H
