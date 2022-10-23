# Navigation of a wheeled mobile robot using hand gestures (master thesis) using Python and ROS
Implementation of mapping (SLAM),localization,path planning. For the global Dijkstra algorithm was implemented and for the local proportional control was used to get to each point. For obstacle avoidance lidar data was used with range division to 5 areas (left, fleft,front,fright,right). 

Tested in both simulation (Gazebo) and real environment.

Gesture recognition is from project https://github.com/muxizju/gestureRecognition_handSegmentation using Tensorflow and OpenCV. Code is adapted to ROS environment and cv_bridge is used to convert ROS Image format to OpenCV format.

## Hardware used
Pioneer3-DX robot base, NVIDIA Jetson TX2 with camera, Sick LMS 200 Lidar



