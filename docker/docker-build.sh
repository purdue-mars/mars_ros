#!/bin/bash

DOCKER_DIR=$(realpath $(dirname $0))

docker build --build-arg USER=mars \
             --build-arg PW="robot" \
             --build-arg UID=$(id -u) \
             --build-arg GID=$(id -g) \
             --build-arg ROS_DEPS="$ROS_DEPS" \
             -t mars-icra-2022:local-$USER \
             -f $DOCKER_DIR/Dockerfile \
             $@ \
             $DOCKER_DIR
