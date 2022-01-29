# build the docker:

        docker build -t iftahnaf/vector_guidance:px4_ros1 -f dockers/Dockerfile .

# run the docker:

        export DISPLAY=:0
        xhost local:root

        docker run --rm -it --gpus all \
        -e DISPLAY=$DISPLAY \
        -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
        -v /home/iftach/PX4-Autopilot:/home/vg/PX4-Autopilot:rw \
        --privileged --net=host \
        -p 14570:14570/udp \
        --env=LOCAL_USER_ID="$(id -u)" \
        iftahnaf/vector_guidance:px4_ros1 ./ros_launch_sitl.sh

# run px4 compitable docker:

                export DISPLAY=:0
                xhost +
                docker run -it --privileged \
                --env=LOCAL_USER_ID="$(id -u)" \
                -v /home/iftach/PX4-Autopilot:/home/user/PX4-Autopilot:rw \
                -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
                -e DISPLAY=:0 \
                -p 14570:14570/udp \
                --name=iftach px4io/px4-dev-ros-noetic bash
