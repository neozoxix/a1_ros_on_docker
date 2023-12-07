#!/bin/bash

HOST_PATH=$(dirname "$0")
# HOST_PATH=$(dirname "$(dirname "$0")")

sudo docker cp A1:/root/unitree_ws/src/. ${HOST_PATH}/../a1_docker_ws_src

sudo docker stop A1
