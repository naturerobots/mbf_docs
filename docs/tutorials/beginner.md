# Beginner Tutorial

This tutorial will walk you through a Turlebot Planning Tutorial with the default [Navigation Server](../concepts/navigation_servers.md) in a Gazebo simulation environment and will make you familiar with the general planning workflow of using Move Base (Flex) in ROS. 

Before you continue, make sure to have followed the [Installation Instructions](../installation.md)

## Setup a ROS workspace and install necessary packages

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Run tutorial

Clone [mbf_tutorials](https://github.com/uos/mbf_tutorials) 
```bash
git clone git@github.com:uos/mbf_tutorials.git ~/catkin_ws/src/mbf_tutorials
```
Install dependencies with rosdep
```
rosdep install turtlebot3_mbf
```
From source of workspace: 
```bash
cd ~/catkin_ws/src
catkin_make -j4 && source devel/setup.bash`
```
Define the Turlebot model you want to use:
```bash
export TURTLEBOT3_MODEL=burger
```
Launch the appropriate gazebo world
```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```
Start localization (AMCL)
```bash
roslaunch turtlebot3_mbf amcl_demo.launch
```

Launch Rviz
```bash
roslaunch turtlebot3_mbf rviz.launch
```

In RViz

* Set an Initial Pose estimate with the `2D Pose Estimate` Pose
* Finally set your Navigation Goal with the `2D Nav Goal` Pose

You will now be able to navigate in a similar fashion to this:

![](../img/demo.gif)

## Where to go from here
1. Explore the [ROS Navigation Stack](https://wiki.ros.org/navigation/Tutorials/Using%20rviz%20with%20the%20Navigation%20Stack) further
2. Explore Move Base Flex feature in the [Advanced Tutorial](./advanced.md)