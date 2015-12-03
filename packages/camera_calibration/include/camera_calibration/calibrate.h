#ifndef CAMERA_CALIBRATION_CALIBRATE_H
#define CAMERA_CALIBRATION_CALIBRATE_H

#include <ros/ros.h>
#include <model_acquisition/baxter_interface.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <camera_calibration/calibration_pcl_utils.h>
#include <model_processing/model_processing.h>

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
    CalibrationPclUtils pcl_utils;
    ModelProcessing model_processing;
    std::vector<double> pose;
    int numPoses;
    ofstream observationFile;
    int numPics;
};
#endif
