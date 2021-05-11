# Continuous Replanning

**WIP**

Up to this point in the MBF Tutorials the general planning flow has been to generate a global plan to the target once, and handing over the path to the local planner/controller to execute the plan. This behavior is inherently unable to handle dynamic changes to the environment the robot is in: After a certain amount of necessary change to the global plan, the local_planner/controller will fail. The amount of change that the navigation stack can handle is highly dependent on the specific controller.

This is were **continuous replanning** comes into play: To make the local planners life easier, we will use the globbal planner to replan from the current state in continous intervals.

## mbf_msgs/MoveBaseAction

Move Base Flex has continuous replanning built into the `mbf_msgs/MoveBaseAction` ROS service call, which makes continuous replanning in simple dynamic environments very easy!

We will use an example for goal-based MBF navigation from previous tutorials as the base for seeing continuous replanning in action.

Launch gazebo

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

as well as the Move Base Flex Action Server 

```bash
export TURTLEBOT3_MODEL=burger
roslaunch mbf_beginner amcl_demo_mbf.launch
```

and client node to send the goals for a circular movement through the turtlebot arena.

```bash
rosrun mbf_beginner mbf_goal_client.py
```

## Force Continuous Replanning

We can use Gazebo built-in grab-and-place features to simulate a real obstacle in the robots path. Launch an obstacle into the gazebo world (a simple box in this case)

```
roslaunch mbf_advanced spawn_box.launch
```

Have a look at the result:

* Adjust the position of the box in gazebo (left) in "Translation Mode" (next to "Select Mode")
* Watch replanning happening in RViz, where the red line is the controller path, and the green line in the path the global planner calculated

</br>

![](../../img/cr.gif)

</br>

Of course, you can play with other tutorials as the base for your experiments, as well. Try the [SMACH](./smach.md) or [Behavior Tree](behavior_tree.md) tutorials, for example.

