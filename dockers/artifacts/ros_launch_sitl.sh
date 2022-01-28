#!/bin/bash
yes | cp -rf mavros_posix_sitl.launch /home/vg/PX4-Autopilot/launch/
source /opt/ros/noetic/setup.bash
cd /home/vg/PX4-Autopilot
source Tools/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/sitl_gazebo
export PX4_SIM_SPEED_FACTOR=1
export ROS_IP= $(hostname -I)
roslaunch px4 mavros_posix_sitl.launch my_model_name:=iris_fpv_cam