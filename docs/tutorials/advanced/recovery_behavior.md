# Recovery Behavior

WIP: noetic release of `moveback_recovery`.

Recovering from unknown or failure conditions in dynamic planning environment can be implemented with revovery behaviors. Recovery behavior is usually implemented to only trigger when continuous replanning has failed. They can be loaded into Move Base Flex the same way controllers and planners are configured:

```txt
recovery_behaviors:
  - name: rotate_recovery
    type: rotate_recovery/RotateRecovery
  - name: clear_costmap_recovery
    type: clear_costmap_recovery/ClearCostmapRecovery

recovery_enabled: true
recovery_patience: 15.0
```

This means that the robot will first try to rotate itself and replan. If this fails however, the costmap will be cleared. This may be necessary if there are sensor issues.

Recovery behavior is enabled by default, but the specific plugins have to be implemented/loaded by the user.

Many other recovery plugins are available for your ROS distro, which are not explicitely covered here.

## Run the example

!!! Note on Replanning
    Replanning is turned of in this scenario to produce fatal conditions for the robot with less effort. This shouldn't be done in a production system!

Launch the gazebo world for turtlebot:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch Move Base Flex in recovery mode:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch mbf_advanced amcl_demo_mbf_recovery.launch
```

Launch a goal publising node, e.g. the example from the beginner tutorial

```bash
roslaunch mbf_beginner mbf_goal_client.py
```

Now grab the blue box in Gazebo, and drop it somewhere in front of the robot. You will see that the robot will attempt to rotate itself, and if that fails, clears the costmap temporarily!

