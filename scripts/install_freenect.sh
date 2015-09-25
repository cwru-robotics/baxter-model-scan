#!/bin/bash
# Luc Bettaieb 2015
echo "installing libfreenect2"

sudo apt-get install -y build-essential libturbojpeg libtool autoconf libudev-dev cmake mesa-common-dev freeglut3-dev libxrandr-dev doxygen libxi-dev libopencv-dev
sudo ln -s /usr/lib/x86_64-linux-gnu/libturbojpeg.so.0 /usr/lib/x86_64-linux-gnu/libturbojpeg.so

DIRECTORY=`locate libfreenect2 | sed -n '1 p'`

echo $DIRECTORY

if [$DIRECTORY != ""]; then

	# check to see if directory is a directory 
	# else throw an error

	(cd $DIRECTORY/depends && ./install_ubuntu.sh)

	(cd $DIRECTORY/depends && sudo dpkg -i libglfw3*_3.0.4-1_*.deb)

	(cd $DIRECTORY && mkdir build)
	(cd $DIRECTORY/build && cmake ../examples/protonect/ -DENABLE_CXX11=ON)
	(cd $DIRECTORY/build && make && sudo make install)
else
	echo "need to clone libfreenect2 somewhere first!!"

fi