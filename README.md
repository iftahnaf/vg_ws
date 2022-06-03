# General Info

Vector Guidance workspace for developing systems based on Vector Guidance

# Table of Contents

1. [About Vector Guidance](#about-vector-guidance)
2. [Installation](#Installation)
3. [Projects](#projects)
4. [References](#references)

# About Vector Guidance

Guidance laws that were originally developed for missile guidance, but fit very well for guiding drones
that can implement acceleration in all spatial directions. The laws also take into account aircraft
limitations, such as velocity or acceleration saturation. The implementation of the laws is simple,
efficient and doesn’t require high processing power, as most of the solutions are analytical and don’t
require complicated numerical solvers. The laws were developed by prof. Shaul Gutman.

# Installation

This repository tested on Ubuntu 18.04 with ROS Melodic and PX4 v1.10.1. 

1. Install PX4, ROS and Gazebo9 with the instructions [from here](https://docs.px4.io/master/en/simulation/ros_interface.html).

2. Check out to v1.10.1:

        cd PX4-Autopilot
        git checkout v1.10.1

3. Edit the launch/mavros_posix_sitl.launch: replace the sdf line:

    `<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg vehicle)/$(arg vehicle).sdf"/>`

    with the following line:

    `<arg name="sdf" default="$(find mavlink_sitl_gazebo)/models/$(arg my_model_name)/$(arg my_model_name).sdf"/>`

4. Clone the `vg_ws` repository:

        git clone https://github.com/iftahnaf/vg_ws.git

5. Build the workspace:

        source /opt/ros/melodic # if it's not been sourced by the .bashrc
        cd vg_ws/
        catkin_make

# Projects

* [Balloon Interception By Drone Using Vector Guidance And Computer Vision](https://www.docdroid.net/jnIfP9l/final-project-poster-pdf)


# References
1. [PX4 on Github](https://github.com/PX4/PX4-Autopilot)
2. [ROS](https://www.ros.org/)
3. S. Gutman and S. Rubinsky, "3D-nonlinear vector guidance and exo-atmospheric interception," in IEEE Transactions on Aerospace and Electronic Systems, vol. 51, no. 4, pp. 3014-3022, Oct. 2015, doi: 10.1109/TAES.2015.140204.
4. S. Gutman, "Rendezvous and Soft Landing in Closed Form via LQ Optimization," 2019 27th Mediterranean Conference on Control and Automation (MED), 2019, pp. 536-540, doi: 10.1109/MED.2019.8798572.
