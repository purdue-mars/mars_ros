# icra_mars_2022

### Installation/setup in local environment

1. Install ROS Noetic and setup ROS workspace

2. Install dependencies

```
rosdep install --from-paths src --ignore-src --rosdistro=noetic -y
```

3. Build and source

```
catkin build
source ~/catkin_ws/devel/setup.bash
```

### Installation/setup in docker

1. Install docker

2. Run the docker build script:

```
./docker/docker-build.sh
```

3. Run the docker run script:

```
./docker/docker-run.sh
```

4. Build and source

```
cd catkin_ws
catkin build
source ~/catkin_ws/devel/setup.bash
```

### Run Simulator

1. Add board assets to Gazebo Model Path. Add the following line to your `~/.bashrc`

```
export GAZEBO_MODEL_PATH=~/catkin_ws/src/icra_2022_mars/mars_sim/nist_board_gazebo/models:$GAZEBO_MODEL_PATH
```

2. Run the Gazebo simulator

```
roslaunch mars_launch sim.launch
```

3. Send joint commands to the Panda arm

```
rostopic pub -1 /gazebo_panda/effort_joint_position_controller/command std_msgs/Float64MultiArray "data: [1,1,0,0,1,1,1]"
```
