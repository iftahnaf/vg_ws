#!/bin/bash
set -e
yes | cp -rf mavros_posix_sitl.launch /home/vg/PX4-Autopilot/launch/

source /opt/ros/noetic/setup.bash
source /home/vg/vg_ws/devel/setup.bash

exec "$@"