# UASL lidar stereo gather
This package aims at facilitating data gathering using stereo cameras, LIDAR, and the optitrack system. Note that this is a ROS package, thus you need ROS for execution. This package has been tested for Kinetic version.

## Installation

### Dependencies :
- [OpenCV](http://opencv.org/downloads.html) >= 2.4.8  : Image processing library
- [Velodyne ROS package](http://wiki.ros.org/velodyne) : Package for the Velodyne LIDARs
- [UASL image acquisition](https://github.com/abeauvisage/uasl_image_acquisition) : Package for acquiring images with the cameras. Currently, Bluefox and Tau2 (infrared) are supported. Please make sure to use the branch camTau2, it will soon be merged into the master branch and will be used going forward.
- [Mocap optitrack](http://wiki.ros.org/mocap_optitrack) : Package to stream the optitrack data on the ROS topics. Notable bug : if you try to track more than 1 rigid body, please refer to the fix referenced in the first post of [this issue](https://github.com/ros-drivers/mocap_optitrack/issues/29) to avoid crashes.

## Usage
### How to record stereo, LIDAR, and optitrack data :
- Make sure a roscore is running. 
- If you want to get a separated output for images and lidar data (so 3 different topics), run the launch file named get_lidar_stereo_separated.launch. Else, the launch file get_lidar_stereo.launch combines all 3 into a single message. As parameters, you should specify cam_1_id and cam_2_id. The parameters can be defined in config/params.yaml.


