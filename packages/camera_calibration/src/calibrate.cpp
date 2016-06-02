#include <ros/ros.h>
#include <camera_calibration/calibrate.h>
#include <model_acquisition/model_acquisition.h>
#include <model_acquisition/baxter_interface.h>
#include <Eigen/Core>
#include <iostream>
#include <fstream>
#include <camera_calibration/calibration_pcl_utils.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "calibrate");
    ros::NodeHandle nh;
    Calibrate calibrate(nh, 5);
    calibrate.doCalibration();
}

Calibrate::Calibrate(ros::NodeHandle nh, int numPoses) :
    nh(nh),
    nh_ptr(&nh),
    baxter(nh),
    pcl_utils(nh_ptr),
    pose(7),
    numPoses(numPoses)
{
    ROS_INFO("Constructing Calibrate");
    numPics = 0;
}

void Calibrate::doCalibration()
{
    ROS_INFO("Called doCalibration");
    for (int i = 0; i < numPoses; i++)
    {
        goToPose(i);
        getCameraObservation();
        getRobotObservation();
    }
}

void Calibrate::goToPose(int i)
{
    ROS_INFO("Calling goToPose");
    switch (i)
    {
        case 0:
            pose[0] = -1.22143; 
            pose[1] = 1.11635;
            pose[2] = -.446772;
            pose[3] = .73544;
            pose[4] = -1.00284;
            pose[5] = 2.09388;
            pose[6] = 0.0;
            ROS_INFO("Calling for pose 0");
            baxter.goToPose(convertPoseToVec(), 1); 
            break;
        case 1:
            pose[0] = -.1323;
            pose[1] = -1.5677; 
            pose[2] = -0.33747;
            pose[3] = -.40957;
            pose[4] = -1.75602;
            pose[5] = 2.03827;
            pose[6] = 0.023543;
            ROS_INFO("Calling for pose 1");
            baxter.goToPose(convertPoseToVec(), 1);
            break; 
        case 2:
            pose[0] = -.23009;
            pose[1] = 1.841543; 
            pose[2] = -.37697;
            pose[3] = -.656927;
            pose[4] = -1.71307;
            pose[5] = 2.0969;
            pose[6] = 0.02262;
            ROS_INFO("Calling for pose 2");
            baxter.goToPose(convertPoseToVec(), 1); 
            break;
        case 3:
            pose[0] = -.23239;
            pose[1] = 2.14527; 
            pose[2] = -0.9706;
            pose[3] = -0.72672;
            pose[4] = -2.02332;
            pose[5] = 2.097335;
            pose[6] = 0.253106;
            ROS_INFO("Calling for pose 3");
            baxter.goToPose(convertPoseToVec(), 1);
            break;
        case 4:
            pose[0] = -.05829;
            pose[1] = 1.6195002;
            pose[2] = -1.05576;
            pose[3] = -0.51043;
            pose[4] = -2.45897;
            pose[5] = 2.098102;
            pose[6] = .0398835;
            ROS_INFO("Calling for pose 4");
            baxter.goToPose(convertPoseToVec(), 1);
            break; 
    }
}

Vectorq7x1 Calibrate::convertPoseToVec()
{
    Vectorq7x1 vec_pose;

    vec_pose(2) = pose[0];
    vec_pose(3) = pose[1];
    vec_pose(0) = pose[2];
    vec_pose(1) = pose[3];
    vec_pose(4) = pose[4];
    vec_pose(5) = pose[5];
    vec_pose(6) = pose[6];

    return vec_pose;
}


void Calibrate::getCameraObservation()
{
    ROS_INFO("Getting camera observation...");
    pcl_utils.reset_got_kinect_cloud();

    int count = 0;
    while (!pcl_utils.got_kinect_cloud())
    {
        count++;
        ROS_WARN("No kinect cloud available, retrying...");
        ros::Duration(.5).sleep();
        ros::spinOnce();
        if (count == 20)
        {
            ROS_WARN("Giving up on kinect cloud, could not get camera observation");
            break;
        }
    }
    if (pcl_utils.got_kinect_cloud())
    {
        ROS_INFO_STREAM("Getting camera observation " << numPics);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new PointCloud<pcl::PointXYZRGB>);
        cloud = pcl_utils.getKinectColorCloud();

        std::stringstream initialSnapshot;
        initialSnapshot << "initial_snapshot_" << numPics << ".pcd";
        pcl::io::savePCDFileASCII (initialSnapshot.str(), *cloud); 

        cloud = model_processing.remove_outlier(cloud);

        std::stringstream removedOutliersSnapshot;
        removedOutliersSnapshot << "removed_outliers_snapshot_" << numPics << ".pcd";
        pcl::io::savePCDFileASCII (removedOutliersSnapshot.str(), *cloud); 

        cloud = model_processing.object_identification(pcl_utils.getKinectColorCloud(),12000,100000);

        std::stringstream identifiedObjectSnapshot;
        identifiedObjectSnapshot << "identified_object_snapshot_" << numPics << ".pcd";
        pcl::io::savePCDFileASCII (identifiedObjectSnapshot.str(), *cloud); 

        observationFile.open("cameraObservations.txt", ios::app);
        observationFile << pcl_utils.compute_centroid(cloud) << "\n";
        observationFile.close();
       
        numPics++; 
    }
}

void Calibrate::getRobotObservation()
{
    tf::StampedTransform tf_platform_to_torso;
    tf::TransformListener tf_listener;

    bool tferr = true;
    while (tferr)
    {
        tferr = false;
        try 
        {
            tf_listener.lookupTransform("camera_link", "object_platform", ros::Time(0), tf_platform_to_torso);
        }
        catch (tf::TransformException &exception)
        {
            ROS_ERROR("%s", exception.what());
            tferr = true;
            ros::Duration(0.5).sleep();
            ros::spinOnce();
        }
    }

    Eigen::Affine3f A_platform_wrt_torso = pcl_utils.transformTFToEigen(tf_platform_to_torso);
    ROS_INFO_STREAM("Calculated transform from platform to torso as\n" << A_platform_wrt_torso.linear() << "\n" << A_platform_wrt_torso.translation());
    Eigen::Vector3f point;
        point << 0, 0, 0;
    point = A_platform_wrt_torso * point;
    ROS_INFO_STREAM("Calculated centroid of platform as\n" << point);
    
    observationFile.open("robotObservations.txt", ios::app);
    observationFile << point << "\n";
    observationFile.close();
}
