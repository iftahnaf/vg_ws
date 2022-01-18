#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /home/vg/vg_ws/devel/setup.bash

exec "$@"