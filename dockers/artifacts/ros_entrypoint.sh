#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/vg/vg_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.0.208:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

exec "$@"