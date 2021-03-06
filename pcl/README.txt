pcl package:
  a 2D driven (Haar cascade classifier) 3D detector for underwater robots detection and tracking.
  tested with Nvidia Jetson AGX Xavier.

================ Dependency ================
ROS(tested with Melodic)
OpenCV 3.4.9
PCL 1.8

the following dependency packages can be installed seperately:
zed-ros-wrapper(Melodic) 
image_common(Melodic)
vision_opencv(Melodic)


================ Install ================
mkdir ~/bluerov/src
cd ~/bluerov/src
catkin_make

================ Node ================
(1) 2D detector node
cascade_detection
(2) 3D detector node
cloud_detection_node

================ Launch ================
(1) launch all the nodes.
roslaunch pcl all.launch
(2) only launch the 2D detector
roslaunch pcl cascade_detection.launch
(3) only launch the 3D detector
roslaunch pcl cloud_detection.launch

================ Service ================
(1) save the detected target's 3D pose (raw bbx pose in camera frame, 
    raw bbx pose in locked_pose frame, fused pose in locked_pose frame)
rosservice call /cloud_detection/save_log
(2) save the target's 3D Model's reference point clouds
rosservice call /cloud_detection/save_cloud
(3) compute the RMSE (if the ground truth poses are available)
rosservice call /cloud_detection/compute_rmse

================ Parameters ================
(1) 2D detector
..config/cascade_detection.yaml
(2) 3D detector
..config/cloud_detection.yaml

================ Model ================
two types of BlueRov2 Heavy models are pre-trained: withoud a subcabin, and with a subcabin
..cascade/

================ Sample dataset ================
only several seconds duration data are provided due to the huge file size of the 3D point clouds.
you can play the sample data in loop model.

topics:      /zed/zed_node/left/camera_info               xx msgs    : sensor_msgs/CameraInfo 
             /zed/zed_node/left/image_rect_color          xx msgs    : sensor_msgs/Image      
             /zed/zed_node/point_cloud/cloud_registered   xx msgs    : sensor_msgs/PointCloud2

../sampledataset
