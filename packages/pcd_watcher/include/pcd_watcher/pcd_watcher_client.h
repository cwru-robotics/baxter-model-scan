#ifndef PCD_WATCHER_PCD_WATCHER_CLIENT_H
#define PCD_WATCHER_PCD_WATCHER_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <pcd_watcher/new_pcdAction.h>
#include <string>
#include <pcd_watcher/inotify-cxx.h>

class PcdWatcherClient
{
public:
    PcdWatcherClient();
    bool isConnected();
    void getEvents();

private:
    ros::NodeHandle nh;
    pcd_watcher::new_pcdGoal goal;
    actionlib::SimpleActionClient<pcd_watcher::new_pcdAction> actionClient;
    std::string directory;
    Inotify notify;
};
#endif  // PCD_WATCHER_PCD_WATCHER_CLIENT_H
