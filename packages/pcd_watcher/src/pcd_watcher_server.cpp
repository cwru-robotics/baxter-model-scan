#include <ros/ros.h>
#include <pcd_watcher/pcd_watcher_server.h>
#include <pcd_watcher/new_pcdAction.h>
#include <model_processing/model_processing.h>

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
    
    ModelProcessing model_processing;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
    new_cloud = model_processing.pcd_reader(goal->newFilepath);
    new_cloud = model_processing.remove_outlier(new_cloud);
    
    new_cloud = model_processing.downsampler(new_cloud);

    float* minMax = model_processing.bounding_box(new_cloud);
    
    Eigen::Vector3f centroid = model_processing.computeCentroid(new_cloud);

    result.processedFilepath = model_processing.pcd_writer(new_cloud,goal->newFilepath);
    
    ROS_INFO_STREAM(centroid);
    ROS_INFO_STREAM(minMax[0] << "," << minMax[1] << "," << minMax[2] << "," << minMax[3] << "," << minMax[4] << "," << minMax[5]);
    ROS_INFO("Exiting newPcd callback function");
    actionServer.setSucceeded(result);
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
