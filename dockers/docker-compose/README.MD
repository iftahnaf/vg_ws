# Docker compose for balloon intercept simulation

# Running the following dockers:

1. PX4 SITL with ROS1 and Gazebo
2. balloon_killer node

# Run:

        export DISPLAY=:0
        xhost local:root
        cd docker-compose
        UID=${UID} GID=${GID} docker-compose up