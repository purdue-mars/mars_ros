FROM osrf/ros:noetic-desktop-full

# Update, Upgrade, Install, and Cleanup
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt upgrade -y && \
    # basics
    DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
    sudo tmux nano vim git wget curl net-tools iputils-ping x11-apps htop \
    mesa-utils python3-dev python3-pip python3-tk software-properties-common \
    pylint clang-format jq swig zsh libeigen3-dev python3-catkin-tools && \
    # pip
    pip3 install --upgrade pip && \
    pip3 install scipy numpy matplotlib \
    pandas jupyterlab autopep8 pyflakes && \
    # Clean Up to reduce image size
    rm -rf /var/lib/apt/lists/* /root/.cache/pip/

# Project-specific dependencies
RUN apt update && \
    DEBIAN_FRONTEND=noninteractive apt install --no-install-recommends -y \
    # orocos
    python3-psutil python3-future \ 
    # gazebo
    ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
    # franka
    ros-noetic-libfranka ros-noetic-franka-ros \
    # realsense
    ros-noetic-realsense2-camera && \
    # clean up
    rm -rf /var/lib/apt/lists/* /root/.cache/pip/

# create user
ARG USER=mars
ARG PW=password
ARG UID=1000
ARG GID=1000
RUN groupadd -o -g $GID $USER && \
    useradd -om -u $UID -g $GID -G sudo -s /bin/bash $USER && \
    echo "$USER:$PW" | chpasswd
WORKDIR /home/$USER

USER $USER
CMD /bin/bash
