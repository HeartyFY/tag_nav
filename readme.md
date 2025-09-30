User Manual for ROS-based AprilTag Following and Navigation Module
$1、Overview

1.1 Function Positioning
  The AprilTag navigation and following function enables mobile robots to visually recognize tag labels and navigate to within a certain distance in front of the target position.

1.2 Core Features
  1、Supports custom target tag IDs, configurable navigation distances, and real-time TF coordinate transformation.
  2、Relies on the apriltag_ros tag detection and the move_base navigation stack.
  
$2、Environment and Dependency Requirements
  1、This function package is developed under the ros-noetic system. It is recommended to use ros-noetic on ubuntu20.04.
  2、Core dependent function packages: apriltag_ros, move_base, teb_local_planner.

$3、Environment Deployment
  1、cd tag_nav
  2、Use rosdep： rosdep install --from-paths src -y --ignore-src  or install manually：sudo apt intall ros-noetic-rospy ros-noetic-apriltag-ros ros-noetic-move-base ros-noetic-teb-local-planner

$4、Usage Steps
1、Settings
cd tag_nav
catkin_make
cd src/apriltag_ros
*****
Modify the tag family and the information of the tags to be detected in settings.yaml and tags.yaml under the config directory here.
Modify the camera intrinsic parameters and the topic corresponding to the output image in the continuous_detection.launch file under the launch directory here (Important).
*****
cd src/tag_navigation
*****
Adjust the move_base parameters, map, and amcl positioning parameters under the config directory here according to the actual situation.
*****
2、Usage
Start your own camera node.

cd tag_nav
source devel/setup.bash
roslaunch apriltag_ros continuous_detection.launch 
roslaunch tag_navigation sim.launch (simulation)
roslaunch tag_navigation tag_navigation.launch 
rosrun tag_navigation navigator.py 

The distance from the robot to the tag when it reaches the target point (desired_distance) can be set in navigator.py by yourself.
example:rosrun tag_navigation navigator.py _target_tag_id:=10 _desired_distance:=0.5 _control_frequency:=5.0
