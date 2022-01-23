# build the docker:

        docker build -t vector_guidance/ros1:v1.0 -f dockers/Dockerfile .

# run the docker:

        export DISPLAY=:0
        xhost local:root
        docker run --rm -it --gpus all  -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix:ro vector_guidance/ros1:v1.0 