# model_acquisition
This package will contain software to perform model acquisition by moving a robot's wrist joint in front of a point cloud acquiring sensor, placing an object on a platform attached to the joint, then performing incremental scans.

Currently, only the Baxter robot by Rethink Robotics is supported.

### Usage
These two nodes must run in order to use the ROS service calls:
 - `roslaunch model_acquisition kinect.launch`
 - `roslaunch model_acquisition acquire.launch`

To use the ROS services manually testing:
 - `rosservice call /go_to_scan_pose "request: false"`
 - `rosservice call /acquire_model "model_name: 'model_name'"`

PCDs are saved in `~/<ros_ws>/devel/lib/model_acquisition`.
I'll probably need to write another node that watches for and reorganizes them.
>>>>>>> upstream/master
