<figure>
  <img src="./img/mbf_logo.png" width="500" />
</figure>
</br>

[//]: # (if there is no header at all, mkdocs will inject a "# Home" -> no good)
# 

**Move Base Flex** (MBF) is an enhanced and highly flexible backwards-compatible replacement for [move_base](https://wiki.ros.org/move_base). 

## In Short

MBF, first are foremost, provides *an enhanced version of the planner, controller and recovery plugin ROS interfaces*.

1. It exposes action servers for planning, controlling and recovering, providing detailed information of the current state and the plugin’s feedback. An external executive logic can use MBF and its actions to perform smart and flexible navigation strategies. 
2. MBF enables the use of *other* map representations (besides cost maps), e.g. meshes or grid_map.

## ROSCon 2017

<div style="position:relative;padding-top:56.25%;">
<iframe src="https://player.vimeo.com/video/236174072" width="640" height="360" frameborder="0" allow="autoplay; fullscreen; picture-in-picture" allowfullscreen style="position:absolute;top:0;left:0;width:100%;height:100%;"></iframe>
</div>

## Publications

!!! abstract "Move Base Flex: A Highly Flexible Navigation Framework for Mobile Robots"
    ``` bibtex
    @inproceedings{puetz18mbf,
  		author = {Sebastian Pütz and Jorge Santos Simón and Joachim Hertzberg},
  		title = {{Move Base Flex}: A Highly Flexible Navigation Framework for Mobile Robots},
  		booktitle = {2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  		year = 2018,
  		month = {October},
  		url = {https://github.com/naturerobots/move_base_flex},
 		note = {Software available at \url{https://github.com/naturerobots/move_base_flex}}
	}
	```

!!! abstract "Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots"
	``` abstract
	@inproceedings{puetz21cvp,
    	author = {Pütz, Sebastian and Wiemann, Thomas and Kleine Piening, Malte and Hertzberg, Joachim},
   		title = {Continuous Shortest Path Vector Field Navigation on 3D Triangular Meshes for Mobile Robots},
    	booktitle = {2021 IEEE International Conference on Robotics and Automation (ICRA)},
    	year = 2021,
  		url = {https://github.com/naturerobots/mesh_navigation},
 		note = {Software available at \url{https://github.com/naturerobots/mesh_navigation}}
	}
	```


## Core Features
 
* Fully backwards-compatible with current ROS1 navigation.
* Actions for the submodules planning, controlling and recovering, and services to query the costmaps are provided. This interface allows external executives, e.g. SMACH, or Behavior Trees, to run highly flexible and complex navigation strategies.
* Comprehensive result and feedback information on all actions, including error codes and messages from the loaded plugins. For users still relying on a unique navigation interface, we have extended move_base action with detailed result and feedback information (though we still provide the current one).
* Separation between an abstract navigation framework and concrete implementations, allowing faster development of new applications, e.g. 3D navigation.
* Load multiple planners and controllers, selectable at runtime by setting one of the loaded plugin names in the action goal. 
* Concurrency: Parallel planning, recovering, controlling by selecting different concurrency slots when defining the action goal. Only different plugins instances can run in parallel.

## Architecture Details

We have created Move Base Flex for a larger target group besides the standard developers and users of move_base and 2D navigation based on costmaps, as well as addressed move_base's limitations. Since robot navigation can be separated into planning and controlling in many cases, even for outdoor scenarios without the benefits of flat terrain, we designed MBF based on ***abstract planner-, controller- and recovery behavior-execution classes for maximal flexibility***. 

To accomplish this goal, we 

1. created abstract base classes for the nav core BaseLocalPlanner, BaseGlobalPlanner and RecoveryBehavior plugin interfaces, extending the API to provide a richer and more expressive interface without breaking the current move_base plugin API. 
***The new abstract interfaces allow plugins to return valuable information in each execution cycle***, e.g. why a valid plan or a velocity command could not be computed. This information is then passed to the external executive logic through MBF planning, navigation or recovering actions’ feedback and result. 

2. The planner, controller and recovery behavior execution is implemented in the abstract execution classes *without* binding the software implementation to 2D costmaps (as in move_base). In our framework, move_base is just a particular implementation of a navigation system: its execution classes implement the abstract ones and bind the system to the costmaps. 
***Thereby, the framework can easily be extended for customized navigation approaches***, e.g. navigation on meshes or 3D occupancy grid maps. However, we provide a SimpleNavigationServer class without a binding to costmaps.

### Flowchart
![MBF architecture](./img/move_base_flex.png)

## Getting Started

* Follow the [MBF Starting Guide](./installation.md) and [Tutorial](./tutorials/overview.md)
* See the [Move Base Flex Documentation and Tutorials](https://wiki.ros.org/move_base_flex) in the ROS wiki. 
* Full code for the showcase and tutorials can be found in [this repository](https://github.com/naturerobots/mbf_tutorials).
