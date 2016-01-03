# model_acquisition
This package will contain software to perform model acquisition by moving a robot's wrist joint in front of a point cloud acquiring sensor, placing an object on a platform attached to the joint, then performing incremental scans.

Currently, only the Baxter robot by Rethink Robotics is supported.

### Usage
Just launch this launch file
 - `roslaunch model_acquisition model_acquisition.launch`

PCDs are saved in `~/<ros_ws>/devel/lib/model_acquisition`.
I'll probably need to write another node that watches for and reorganizes them.
