# Move Base Flex: A Highly Flexible Navigation Framework:

This repository contains Move Base Flex (MBF), a backwards-compatible replacement for move_base. MBF can use existing plugins for move_base, and provides an enhanced version of the same ROS interface. It exposes action servers for planning, controlling and recovering, providing detailed information of the current state and the plugin's feedback. An external executive logic can use MBF and its actions to perform smart and flexible navigation strategies. For example, at [Magazino](https://www.magazino.eu/?lang=en) we have successfully deployed MBF at customer facilities to control TORU robots in highly dynamical environments. Furthermore, MBF enables the use of other map representations, e.g. meshes. The core features are:
 
* Fully backwards-compatible with current ROS navigation.
* Actions for the submodules planning, controlling and recovering, and services to query the costmaps are provided. This interface allows external executives, e.g. SMACH, or Behavior Trees, to run highly flexible and complex navigation strategies.
* Comprehensive result and feedback information on all actions, including error codes and messages from the loaded plugins. For users still relying on a unique navigation interface, we have extended move_base action with detailed result and feedback information (though we still provide the current one).
* Separation between an abstract navigation framework and concrete implementations, allowing faster development of new applications, e.g. 3D navigation.
* Load multiple planners and controllers, selectable at runtime by setting one of the loaded plugin names in the action goal. 
* Concurrency: Parallel planning, recovering, controlling by selecting different concurrency slots when defining the action goal. Only different plugins instances can run in parallel.

Please see also the [Move Base Flex Documentation and Tutorials](https://wiki.ros.org/move_base_flex) in the ROS wiki. And [this repository](https://github.com/Rayman/turtlebot3_mbf) contains a working minimal configuration for a turtlebot 3.
