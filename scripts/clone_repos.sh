#!/bin/bash
# Luc Bettaieb 2015

echo "cloning all repos"

#todo add in $1 check

WS_NAME=$1

if [[ $WS_NAME != "" ]];
then

	mkdir -p ~/$WS_NAME/src/cwru
	mkdir -p ~/$WS_NAME/src/rethink

	# set up cwru things
	#you might want to change this to a fork?
  (cd ~/$WS_NAME/src/cwru && git clone https://github.com/cwru-robotics/cwru_baxter.git)


	(cd ~/$WS_NAME/src/cwru && git clone https://github.com/cwru-robotics/cwru_msgs.git)
	(cd ~/$WS_NAME/src/cwru && git clone https://github.com/catkin/catkin_simple.git)
	(cd ~/$WS_NAME/src/cwru && git clone https://github.com/code-iai/iai_kinect2.git)
	(cd ~/$WS_NAME/src/cwru && git clone https://github.com/OpenKinect/libfreenect2)

	# set up rethink things
	(cd ~/$WS_NAME/src/rethink && git clone https://github.com/RethinkRobotics/baxter_common.git)
	(cd ~/$WS_NAME/src/rethink && git clone https://github.com/RethinkRobotics/baxter_interface.git)
	(cd ~/$WS_NAME/src/rethink && git clone https://github.com/RethinkRobotics/baxter_tools.git)

else
  echo "specify a catkin workspace (e.g. indigo or ros_ws...)"

fi