#ifndef CAMERA_CALIBRATION_CALIBRATE_H
#define CAMERA_CALIBRATION_CALIBRATE_H

#include <ros/ros.h>
#include <model_acquisition/baxter_interface.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <cwru_pcl_utils/cwru_pcl_utils.h>

class Calibrate
{
public:
    Calibrate(ros::NodeHandle nh, int i);
    void doCalibration();

private:
    void getCameraObservation();
    void getRobotObservation();
    void goToPose(int i);
    Vectorq7x1 convertPoseToVec();

    ros::NodeHandle nh;
    ros::NodeHandle* nh_ptr;
    BaxterInterface baxter;
    CwruPclUtils cwru_pcl_utils;
    std::vector<double> pose;
    int numPoses;
    ofstream observationFile;
};
#endif
