# MBF Parameters

## Dynamically Reconfigurable MBF Parameters

The following parameters are changable at runtime, with `rqt_reconfigure`, or with launch file arguments

### Planners

The global planner handles the actual path planning. There are many options available, but the default is `navfn/NavfnROS`.

| **Name** | **Explanation** | **Default** |
| :-- | --- | --: |
| planners | global planner, e.g. `navfn/NavfnROS` | |
| planner_frequency | The rate in Hz at which to run the planning loop |  0.0 ?? |
| planner_max_retries | How many times we will recall the planner in an attempt to find a valid plan before giving up |  -1. ?? |
| planner_patience | How long the planner will wait in seconds in an attempt to find a valid plan before giving up |  5.0 |

### Controllers

The controller handles generating control signals for the path generated by the global planner. Popular choices are DWALocalPlanner or EBandLocalPlanner.

| **Name** | **Explanation** | **Default** |
| :-- | --- | --: |
| controllers | list of controller, e.g. eband_local_planner/EBandPlannerROS | |
| controller_frequency | The rate in Hz at which to run the control loop and send velocity commands to the base | 20.0 |
| controller_max_retries | How many times we will recall the controller in an attempt to find a valid command before giving up | -1 |
| controller_patience | How long the controller will wait in seconds without receiving a valid control before giving up | 5.0 |

### Oscillation

| **Name** | **Explanation** | **Default** |
| :-- | --- | --: |
| oscillation_distance | How far in meters the robot must move to be considered not to be oscillating |  0.5 |
| oscillation_timeout | How long in seconds to allow for oscillation before executing recovery behaviors |  0.0 |

### Other

| **Name** | **Explanation** | **Default** |
| :-- | --- | --: |
| recovery_enabled | enable the move_base_flex recovery behaviors to attempt to clear out space |  true |
| recovery_patience | How much time we allow recovery behaviors to complete before canceling (or stopping if cancel fails) |  15.0 |
| restore_defaults | Restore to the original configuration |  false |
| shutdown_costmaps | shutdown the costmaps of the node when move_base_flex is in an inactive state |  false |
| shutdown_costmaps_delay | How long in seconds to wait after last action before shutting down the costmaps |  1.0 |


### Example

The beginner tutorials uses this configuration, for example

```
planners:
  - name: navfn/NavfnROS
    type: navfn/NavfnROS

controllers:
  - name: eband_local_planner/EBandPlannerROS
    type: eband_local_planner/EBandPlannerROS

controller_frequency: 5.0
controller_patience: 3.0

planner_frequency: 1.0
planner_patience: 5.0

oscillation_timeout: 10.0
oscillation_distance: 0.2
```

and loads these initial parameters (among others) in the launch file

```xml
<launch>
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>

  <node name="move_base_flex" pkg="mbf_costmap_nav" type="mbf_costmap_nav" required="true" output="screen" clear_params="true">
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/costmap_common_params_$(arg model).yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find turtlebot3_navigation)/param/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find turtlebot3_navigation)/param/global_costmap_params.yaml" command="load" />
=>  <rosparam file="$(find mbf_beginner)/param/move_base_flex.yaml" command="load"/>
  </node>
</launch>
```

#### Sources

* [mbf_abstract_nav](https://github.com/naturerobots/move_base_flex/blob/596ed881bfcbd847e9d296c6d38e4d3fa3b74a4d/mbf_abstract_nav/src/mbf_abstract_nav/__init__.py)
* [mbf_costmap_nav](https://github.com/naturerobots/move_base_flex/blob/596ed881bfcbd847e9d296c6d38e4d3fa3b74a4d/mbf_costmap_nav/cfg/MoveBaseFlex.cfg) 

To use `rqt_reconfigure`, run

```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## Static Parameters

Move Base Flex has a number of parameters, that are not dynamically changable

| **Name** | **Explanation** | **Default** |
| :-- | --- | --: |
`robot_frame` | the frame of the robot, which will be used to determine its position | "base_link" |
`map_frame` | the global frame the robot is controlling in | "map" |
`force_stop_at_goal` | force move base flex to stop the robot once the goal is reached | false |
`force_stop_on_cancel` | force move base flex to stop the robot on navigation cancellation | false |
`mbf_tolerance_check` | force move base flex to check for the goal tolerance| false |
`dist_tolerance` | distance tolerance to the given goal pose | 0.1 |
`angle_tolerance` | angle tolerance to the given goal pose | π / 18.0 |
`tf_timeout` | time before a timeout used for tf requests | 1.0 |

### Global Costmap

TODO

### Local Costmap

TODO