#!/bin/bash

WS_DIR=$(realpath $(dirname $0)/../../../)
CONTAINER_NAME="${CONTAINER_NAME:-$USER-mars-dev}"

docker run --rm \
	-it \
	-e USER=mars \
	-e DISPLAY \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v $XAUTHORITY:/home/mars/.Xauthority:ro \
	-v $WS_DIR:/home/mars/catkin_ws \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	`[ -f ~/.gitconfig ] && echo "-v $HOME/.gitconfig:/home/mars/.gitconfig:ro"` \
	-v ~/.ssh:/home/mars/.ssh:ro \
	--name $CONTAINER_NAME \
	$@ \
	mars-icra-2022:local-$USER \
	${DOCKER_CMD:+/bin/bash -c "$DOCKER_CMD"}

