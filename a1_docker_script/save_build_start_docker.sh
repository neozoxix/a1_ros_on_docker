#!/bin/bash

cd ..

sudo docker build -t a1_ros_on_docker .

sudo docker run --name A1 -itd --rm --privileged --net=host --ipc=host \
--device=/dev/dri:/dev/dri -v /dev:/dev \
-v /tmp/.X11-unix:/tmp/.X11-unix  \
-e DISPLAY=$DISPLAY -v $HOME/.Xauthority:/home/$(id -un)/.Xauthority \
-e XAUTHORITY=/home/$(id -un)/.Xauthority  \
-v .:/a1_ros_on_docker a1_ros_on_docker bash

cd a1_docker_script
