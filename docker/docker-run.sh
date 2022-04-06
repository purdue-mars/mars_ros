#!/bin/bash

WS_DIR=$(realpath $(dirname $0)/../../../)
CONTAINER_NAME="${CONTAINER_NAME:-$USER-mars-dev}"

docker run --rm \
	-it \
	-e USER \
	-e DISPLAY \
	-e NVIDIA_DRIVER_CAPABILITIES=all \
	-v $XAUTHORITY:/home/$USER/.Xauthority:ro \
	-v $WS_DIR:/home/$USER/catkin_ws \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	`[ -f ~/.gitconfig ] && echo "-v $HOME/.gitconfig:/home/$USER/.gitconfig:ro"` \
	-v ~/.ssh:/home/$USER/.ssh:ro \
	--name $CONTAINER_NAME \
	$@ \
	mars-icra-2022:local-$USER \
	${DOCKER_CMD:+/bin/bash -c "$DOCKER_CMD"}

