/* 
 * baxter_interface
 * 
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 * 
 */

#ifndef MODEL_ACQUISITION_H
#define MODEL_ACQUISITION_H

#include <ros/ros.h>
#include <angles/angles.h>

#include "model_acquisition/baxter_interface.h"
#include "model_acquisition/kinect2_interface.h"

#include "model_acquisition/scan_pose.h"
#include "model_acquisition/acquire.h"

bool goToScanPose(model_acquisition::scan_pose::Request &request, model_acquisition::scan_pose::Response &response);

bool acquireModel(model_acquisition::acquire::Request &request, model_acquisition::acquire::Response &response);

#endif  // MODEL_ACQUISITION_H
