#!/bin/bash
source /opt/ros/noetic/setup.bash
cd /home/vg/PX4-Autopilot
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export PX4_SIM_SPEED_FACTOR=1
roslaunch px4 mavros_posix_sitl.launch my_model_name:=iris_fpv_cam