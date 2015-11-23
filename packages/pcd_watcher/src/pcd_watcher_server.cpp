#include <ros/ros.h>
#include <pcd_watcher/pcd_watcher_server.h>
#include <pcd_watcher/new_pcdAction.h>

PcdWatcherServer::PcdWatcherServer() :
        actionServer(nh, "new_pcd", boost::bind(&PcdWatcherServer::newPcdCB, this, _1), false)
{
    ROS_INFO("In constructor of PcdWatcherServer...");
    ROS_INFO("Starting action server...");
    actionServer.start();
    ROS_INFO("Started action server.");
    ROS_INFO("Exiting PcdWatcherServer constructor");
}

void PcdWatcherServer::newPcdCB(const actionlib::SimpleActionServer<pcd_watcher::new_pcdAction>::GoalConstPtr& goal)
{
    ROS_INFO("In the newPcd callback function...");
    ROS_INFO("Received goal message, new file is %s", goal->newFilepath.c_str());
    ROS_INFO("I should be sending this goal message off to some processing server now...");
    ROS_INFO("Exiting newPcd callback function");
    actionServer.setSucceeded();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_watcher_server");
    PcdWatcherServer server;
    ROS_INFO("Ready to receive new pcd filepaths...");

    while (ros::ok())
    {
        ros::spin();
    }
    return 0;
}
