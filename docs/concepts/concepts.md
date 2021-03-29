# Concepts

## Actionlib: Action Servers and Clients

Actionlib is a built-in ROS package and provides a standardized interfaces for working with preemptible tasks and is a central piece of communicating 

Typically, a client will request some task to be completed on the server. Because this action will complete at some time in the future, feedback that the client can receive during execution can be anything and is defined in the corresponding ROS `.action`. This architecture is flexible in the sense that the client can chose to block or poll the server, until the goal is reached.

In Move Base Flex, for example, the `mbf_msgs/MoveBaseAction` provides the distance to the goal, angle to the goal, current pose and last command calculated by the controller to the client during path execution.

The [official tutorials](http://wiki.ros.org/actionlib/Tutorials) are a good place to get started.

## Environment Representation 

### Costmap

The default environmental representation of Move Base Flex is a costmap, which is a regular 2D grid of cells representing an occupancy grid of the sensor data. A cells state can generally known to be `unknown`, `free` or `occupied` by an obstacle.

Costmaps can be created from LIDAR, RADAR, sonar, (depth) images and more. 

### Others

Many different environment represetations exist, e.g.

* 3D costmap: represents planning space in 3D
* Mesh maps: the planning space is represented with a surface mesh. You can see a mesh navigation plugin for Move Base Flex [here](https://github.com/uos/mesh_navigation).
* Gradient maps: Typically used for traversibility tasks as surface is represented as gradients

## Planners, Controllers, Recovery Behavior

### Planners
The general task for a planner is to compute a valid and potentially optimal path from the current pose to a goal pose. For this, the planner has access to a global environment representation and sensor data (IMU, Lidar, etc).

The default global planner in the ROS1 Navigation Stack is `navfn/NavfnROS`.

### Controller

A controller, also referred to as *local planner* in the ROS1 navigation stack, computes how to follow the global path provided by a global Planner by generating control signals. Many different controllers exist, that implement different features.

Some popular local planners/controllers in the ROS1 navigation ecosystem are 

* `dwa_local_planner/DWALocalPlannerROS`
* `eband_local_planner/EBandPlannerROS`

### Recovery Behavior

TODO ADVANCED TUTORIAL