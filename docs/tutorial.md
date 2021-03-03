# Tutorial

This tutorial will walk you through a Turlebot Planning Tutorial with the default [Navigation Server](./concepts/navigation_servers.md) in a Gazebo simulation environment and will make you familiar with the general workflow of using Move Base Flex.

Before you continue, make sure to have followed the [Installation Instructions](./installation.md)

## Setup a ROS workspace and install necessary packages

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```
Install `turtlebot3-navigation`
```bash
$ apt install ros-$ROS_VERSION-turtlebot3-navigation -y
```
Install DWA Planner (Dynamic Window Approach)
```bash
$ apt install ros-$ROS_VERSION-dwa-local-planner
```

## Run tutorial

Clone [mbf_tutorials](https://github.com/uos/mbf_tutorials) 
```bash
$ git clone git@github.com:uos/mbf_tutorials.git ~/catkin_ws/src/mbf_tutorials
```
From source of workspace: 
```bash
$ cd ~/catkin_ws/src
$ catkin_make -j4 && source devel/setup.bash`
```
Define the Turlebot model you want to use:
```bash
$ export TURTLEBOT3_MODEL=burger
```
Launch the appropriate gazebo world
```bash
$ roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Start localization (AMCL)
```bash
$ roslaunch turtlebot3_mbf amcl_demo.launch
```

Launch Rviz
```bash
roslaunch turtlebot3_mbf rviz.launch
```

In RViz, click "

You will now be able to navigate in a similar fashion to this:

![](./img/demo.gif)