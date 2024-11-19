# Beginner Tutorial

This tutorial will walk you through a Turlebot Planning Tutorial based on costmap navigation in a Gazebo simulation environment and will make you familiar with the general planning workflow of using Move Base (Flex) in ROS. 

Before you continue, make sure to have followed the [Installation Instructions](../../installation.md) and you have a ROS workspace set up

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```

## Topics

1. **Basic Navigation** <br> Learn how to move a robot from point a to b with Move Base Flex and RViz. Continue [here](./basic_navigation.md).
2. **Beyond the Relay** <br> Learn to use the Move Base Flex API directly and leave Move Base behind. Continue [here](./beyond_relay.md).
3. **Path Planning** <br> Learn how to plan with pose paths instead of goals for greated flexibility. Continue [here](./path_planning.md)
4. **Parameters and Configuraton** <br> After getting to know a typical planning workflow, learn about parameters and configuration of Move Base Flex. Continue [here](./parameters/overview.md)

<br>

The full source code can be found [here](https://github.com/naturerobots/mbf_tutorials/tree/master/beginner).