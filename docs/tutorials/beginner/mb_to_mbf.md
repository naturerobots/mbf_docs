# From Move Base to Move Base Flex
## Run tutorial

Clone [mbf_tutorials](https://github.com/uos/mbf_tutorials) 
```bash
git clone git@github.com:uos/mbf_tutorials.git ~/catkin_ws/src/mbf_tutorials
```
Install dependencies with rosdep
```bash
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

![](../../img/demo.gif)


## What is happening here? 

We used Move Base Flex by relaying `mb_msgs/MoveBaseAction` to `mbf_msgs/MoveBaseAction`. 

=== "Code"

    ```python
    def simple_goal_cb(msg):
        mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
        rospy.logdebug("Relaying move_base_simple/goal pose to mbf")

        mbf_mb_ac.wait_for_result()

        status = mbf_mb_ac.get_state()
        result = mbf_mb_ac.get_result()

        rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)
        if result.outcome == mbf_msgs.MoveBaseResult.SUCCESS:
            mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")
        else:
            mb_as.set_aborted(mb_msgs.MoveBaseResult(), result.message)

    if __name__ == '__main__':
        rospy.init_node("move_base_relay")

        # move base flex ation client relays incoming mb goals to mbf
        mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        mbf_mb_ac.wait_for_server(rospy.Duration(20))

        # move_base simple topic and action server
        mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)

        rospy.spin()
    ```

=== "Explanation"
    

    MoveBase subscriber
    ```python
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)
    ```
    MoveBase expects goal Messages (`geometry_msgs/Pose`) on topic `move_base_simple/goal`. The subscriber callback `simple_goal_cb` handles the `mbf_msgs.MoveBaseAction` ROS Action Client

    ```python
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    ...

    def simple_goal_cb(msg):
        mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
    ```

At this stage, we are using the global planner and local planner defined in [move_base.yml](https://github.com/uos/mbf_tutorials/blob/master/beginner/param/move_base.yaml).

The full source code can be found [here](https://github.com/uos/mbf_tutorials/tree/master/beginner).

## *mb_msgs/MoveBaseAction* vs *mbf_msgs/MoveBaseAction*

=== "mb_msgs/MoveBaseAction"

    In principle, every Move Base Action is defined as

    ```
    geometry_msgs/PoseStamped target_pose
    ---
    No Result Feedback (default)
    ---
    geometry_msgs/PoseStamped base_position
    ```

    [Source](https://docs.ros.org/en/api/move_base_msgs/html/action/MoveBase.html)

=== "mbf_msgs/MoveBaseAction"

    Move Base Flex uses the same target/result feedback/action feedback structure, but adds new functionality:
    
    * More detailed result feedback
    * More defailed feedback possibilities
    * plugins: controller (local planner), planner (global planner), recovery_behaviors 

    ```
    geometry_msgs/PoseStamped target_pose

    # Controller to use; defaults to the first one specified on "controllers" parameter
    string controller

    # Planner to use; defaults to the first one specified on "planners" parameter
    string planner

    # Recovery behaviors to try on case of failure; defaults to the "recovery_behaviors" parameter value
    string[] recovery_behaviors

    ---

    # Predefined success codes:
    uint8 SUCCESS        = 0

    # Predefined general error codes:
    uint8 FAILURE        = 10
    uint8 CANCELED       = 11
    uint8 COLLISION      = 12
    uint8 OSCILLATION    = 13
    uint8 START_BLOCKED  = 14
    uint8 GOAL_BLOCKED   = 15
    uint8 TF_ERROR       = 16
    uint8 INTERNAL_ERROR = 17
    # 21..49 are reserved for future general error codes

    # Planning/controlling failures:
    uint8 PLAN_FAILURE   = 50
    # 51..99 are reserved as planner specific errors

    uint8 CTRL_FAILURE   = 100
    # 101..149 are reserved as controller specific errors

    uint32 outcome
    string message

    # Configuration upon action completion
    float32 dist_to_goal
    float32 angle_to_goal
    geometry_msgs/PoseStamped final_pose

    ---

    # Outcome of most recent controller cycle. Same values as in MoveBase or ExePath result.
    uint32 outcome
    string message

    float32 dist_to_goal
    float32 angle_to_goal
    geometry_msgs/PoseStamped current_pose
    geometry_msgs/TwistStamped last_cmd_vel  # last command calculated by the controller`
    ```

    [Source](https://github.com/magazino/move_base_flex/blob/master/mbf_msgs/action/MoveBase.action)