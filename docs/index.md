# Move Base Flex: A Highly Flexible Navigation Framework:

Move Base Flex (MBF) is a enhanced and flexible backwards-compatible replacement for [move_base](https://wiki.ros.org/move_base). 

## In Short

MBF, first are foremost, provides *an enhanced version of the planner, controller and recovery plugin ROS interfaces*.

1. It exposes action servers for planning, controlling and recovering, providing detailed information of the current state and the plugin’s feedback. An external executive logic can use MBF and its actions to perform smart and flexible navigation strategies. 
2. MBF enables the use of *other* map representations (besides cost maps), e.g. meshes or grid_map.


## Core Features
 
* Fully backwards-compatible with current ROS navigation.
* Actions for the submodules planning, controlling and recovering, and services to query the costmaps are provided. This interface allows external executives, e.g. SMACH, or Behavior Trees, to run highly flexible and complex navigation strategies.
* Comprehensive result and feedback information on all actions, including error codes and messages from the loaded plugins. For users still relying on a unique navigation interface, we have extended move_base action with detailed result and feedback information (though we still provide the current one).
* Separation between an abstract navigation framework and concrete implementations, allowing faster development of new applications, e.g. 3D navigation.
* Load multiple planners and controllers, selectable at runtime by setting one of the loaded plugin names in the action goal. 
* Concurrency: Parallel planning, recovering, controlling by selecting different concurrency slots when defining the action goal. Only different plugins instances can run in parallel.

## Architecture Details

Continue [here](./concepts/architecture.md)

## Getting Started

* Follow the [MBF Starting Guide](./installation.md) and [Tutorial](./tutorials/overview.md)
* See the [Move Base Flex Documentation and Tutorials](https://wiki.ros.org/move_base_flex) in the ROS wiki. 
* Full code for the showcase and tutorials can be found in [this repository](https://github.com/uos/mbf_tutorials).
