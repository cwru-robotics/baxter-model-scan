/* 
 * baxter_interface
 * 
 * Copyright (c) 2015, Luc Bettaieb
 * BSD Licensed
 * 
 */

#ifndef BAXTER_INTERFACE_H
#define BAXTER_INTERFACE_H

class BaxterInterface
{
public:
	BaxterInterface();
	virtual ~BaxterInterface();

private:
	virtual bool setJointToAngle(int joint, double angle);

public:
	virtual bool incrementScanAngle();
	virtual bool goToScanPose();
};

#endif // BAXTER_INTERFACE_H