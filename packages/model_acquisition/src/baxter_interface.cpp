/* 
 * baxter_interface
 * 
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 * 
 */

#include <baxter_interface.h>
#include <ros/ros.h>

BaxterInterface::BaxterInterface()
{
	// load in increment angle from parameter server
	// make sure its divisible by 360, if not, set to closest angle that is divisible by 360
}
BaxterInterface::~BaxterInterface()
{

}

bool BaxterInterface::setJointToAngle(int joint, double angle)
{

}

bool BaxterInterface::goToScanPose()
{

}

bool BaxterInterface::incrementScanAngle()
{

}