#include <ros/ros.h>
#include <pcd_watcher/pcd_watcher_client.h>
#include <string>
#include <pcd_watcher/inotify-cxx.h>
#include <exception>

PcdWatcherClient::PcdWatcherClient() :
        actionClient("new_pcd", true)
{
    ROS_INFO("In constructor of PcdWatcherClient...");
    if (!nh.getParam("pcd_watcher_client/directory", directory))
    {
        directory = "/tmp/PCD";
    }
    ROS_INFO("PcdWatcherClient created that is watching %s", directory.c_str());
     
    InotifyWatch watch(directory, IN_CREATE); 
    try
    {
        notify.Add(watch);
    }
    catch (InotifyException &e)
    {
        ROS_WARN("Inotify exception occured: %s", e.GetMessage().c_str());
    }
    catch (std::exception &e)
    {
        ROS_WARN("STL exception occured: %s", e.what());
    }
    catch (...)
    {
        ROS_WARN("Uknown exception occured");
    }
    ROS_INFO("Exiting PcdWatcherClient constructor");
}

bool PcdWatcherClient::isConnected()
{
    return actionClient.waitForServer(ros::Duration(5.0));
}

void PcdWatcherClient::getEvents()
{
    try
    {
        notify.WaitForEvents();
        size_t count = notify.GetEventCount();
        InotifyEvent event;
        bool got_event;
        std::string filename;
        std::string filepath;

        for (int i = 0; i < count; i++)
        {
            got_event = notify.GetEvent(&event);
            if (got_event)
            {
                filename = event.GetName();
                filepath = directory + "/" + filename;
                ROS_INFO("Event detected, new file %s created", filepath.c_str());
                goal.newFilepath = filepath;
                actionClient.sendGoal(goal);
            }
        }
    }
    catch (InotifyException &e)
    {
        ROS_WARN("Inotify exception occured: %s", e.GetMessage().c_str());
    }
    catch (std::exception &e)
    {
        ROS_WARN("STL exception occured: %s", e.what());
    }
    catch (...)
    {
        ROS_WARN("Uknown exception occured");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pcd_watcher_client");
    PcdWatcherClient client;

    ROS_INFO("Waiting for server...");
    bool server_exists = client.isConnected();
    int count = 5;
    while (count > 0 && !server_exists)
    {
        ROS_WARN("Could not connect to pcd_watcher_server, retrying %d more times", count);
        server_exists = client.isConnected();
        count--;
        if (count == 0)
        {
            ROS_WARN("Failed to connect to pcd_watcher_server, giving up");
            return 0;
        }
    }
    ROS_INFO("Connected to pcd_watcher_server!");

    bool got_event = false;
    while (ros::ok)
    {
        client.getEvents();
    }
    return 0;
}
