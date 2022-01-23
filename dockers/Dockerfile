#
# PX4 ROS development environment
#

FROM px4io/px4-dev-simulation-focal:2021-09-08
LABEL maintainer="Nuno Marques <n.marques21@hotmail.com>"

ENV ROS_DISTRO noetic

# setup ros keys
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
	&& sh -c 'echo "deb http://packages.ros.org/ros/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-latest.list' \
	&& sh -c 'echo "deb http://packages.ros.org/ros-shadow-fixed/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros-shadow.list' \
	&& apt-get update \
	&& apt-get -y --quiet --no-install-recommends install \
		geographiclib-tools \
		libeigen3-dev \
		libgeographic-dev \
		libopencv-dev \
		libyaml-cpp-dev \
		python3-rosdep \
		python3-catkin-tools \
		python3-catkin-lint \
		ros-$ROS_DISTRO-gazebo-ros-pkgs \
		ros-$ROS_DISTRO-mavlink \
		ros-$ROS_DISTRO-mavros \
		ros-$ROS_DISTRO-mavros-extras \
		ros-$ROS_DISTRO-octomap \
		ros-$ROS_DISTRO-octomap-msgs \
		ros-$ROS_DISTRO-pcl-conversions \
		ros-$ROS_DISTRO-pcl-msgs \
		ros-$ROS_DISTRO-pcl-ros \
		ros-$ROS_DISTRO-ros-base \
		ros-$ROS_DISTRO-rostest \
		ros-$ROS_DISTRO-rosunit \
		xvfb \
	&& geographiclib-get-geoids egm96-5 \
	&& apt-get -y autoremove \
	&& apt-get clean autoclean \
	&& rm -rf /var/lib/apt/lists/{apt,dpkg,cache,log} /tmp/* /var/tmp/*

RUN pip3 install -U \
		osrf-pycommon

RUN pip3 install imutils

# bootstrap rosdep
RUN rosdep init && rosdep update

# install graphics
RUN apt-get install -y xserver-xorg && \
    dpkg-reconfigure xserver-xorg

RUN apt-get update && apt-get install -y gstreamer1.0-libav

RUN apt-get update
RUN apt-get install -y mesa-utils
RUN apt-get install -y binutils
RUN apt install -y nvidia-driver-470 libnvidia-gl-470 libnvidia-compute-470 libnvidia-decode-470 libnvidia-encode-470 libnvidia-ifr1-470 libnvidia-fbc1-470

# clone PX4 repository from github and make px4_sitl default
RUN git clone https://github.com/PX4/PX4-Autopilot.git /home/vg/PX4-Autopilot

RUN cd /home/vg/PX4-Autopilot && \
	git checkout stable && \
	make px4_sitl_default

# copy workspace and catkin_make
COPY src/ /home/vg/vg_ws/src

SHELL ["/bin/bash", "-c"]

RUN cd /home/vg/vg_ws && \ 
    source /opt/ros/noetic/setup.bash && \
    catkin_make

# copy items
COPY dockers/artifacts/ros_entrypoint.sh /
COPY dockers/artifacts/ros_launch_sitl.sh /
COPY dockers/artifacts/iris.sdf /home/vg/PX4-Autopilot/Tools/sitl_gazebo/models/iris/
COPY dockers/artifacts/mavros_posix_sitl.launch /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]