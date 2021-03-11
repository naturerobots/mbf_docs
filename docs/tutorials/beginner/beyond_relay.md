# Leaving the Relay Behind

Using a relay from Move Base to Move Base Flex is the easiest way to get started with Move Base Flex. This does, however, make it harder to use advanced features of Move Base Flex. Let's start with understanding the differences between the respective Actions:

## mb_msgs/MoveBaseAction vs mbf_msgs/MoveBaseAction

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
    
    * More detailed result feedback (per default)
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


## Interacting with Move Base Flex

From a client perspective, the primary interface to work with Move Base Flex is the actionlibs SimpleActionServer. If you have never heard of actionlib, the [ROS Wiki](https://wiki.ros.org/actionlib_tutorials) has some good tutorials for it. We did, however, already use actionlib in earlier parts of this tutorial.

In principle, a `SimpleActionServer` expects a name and an *action* (ROS message type) that it will perform. A `SimpleActionClient` can then connect to the Server by name and Action and send respective *goals*, which are just the specific *action* with a ROS header and Goal ID.

## Driving a Circle with Move Base Flex

In the previous example, we used a relay to Move Base with a Move Base `SimpleActionServer`. Using this method, the Move Base Flex is hidden, so to speak, inside the relay, and the corresponding Move Base Client is limited to the functionality of the Move Base Action Server. The following example will use the additional information the Move Base Flex Action Server provides.

We can, however, use the Move Base Flex Action server that is started with Move Base Flex to interact with the framework directly. This is the circle driving robot with Move Base Flex only.

### Code

```python
import actionlib
import rospy
import mbf_msgs.msg as mbf_msgs


def create_goal(x, y, z, xx, yy, zz, ww):
    goal = mbf_msgs.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = xx
    goal.target_pose.pose.orientation.y = yy
    goal.target_pose.pose.orientation.z = zz
    goal.target_pose.pose.orientation.w = ww
    return goal


def move(goal):
    mbf_ac.send_goal(goal)
    mbf_ac.wait_for_result()
    return mbf_ac.get_result()


def drive_circle():
    goals = [   create_goal(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
                create_goal(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
                ...
    ]

    for goal in goals:
        rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
        result = move(goal)

        if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
            rospy.loginfo("Unable to complete action")
            return 

if __name__ == '__main__':
    rospy.init_node("move_base_flex_client")

    mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_ac.wait_for_server(rospy.Duration(10))
    rospy.loginfo("Connected to Move Base Flex action server!")

    drive_circle()

```

### The Code Explained

We start by creating the Move Base Flex Action Client that tries to connect to the server running at `/move_base_flex/move_base`.

```python
mbf_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
mbf_ac.wait_for_server(rospy.Duration(10))
rospy.loginfo("Connected to Move Base Flex action server!")
```

To actually drive the circle, we can create goals of type `mbf_msgs.MoveBaseGoal`

```python
def create_goal(x, y, z, xx, yy, zz, ww):
    goal = mbf_msgs.MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    goal.target_pose.pose.position.z = z
    goal.target_pose.pose.orientation.x = xx
    goal.target_pose.pose.orientation.y = yy
    goal.target_pose.pose.orientation.z = zz
    goal.target_pose.pose.orientation.w = ww
    return goal
```

and send them to the Server

```python
def move(goal):
    mbf_ac.send_goal(goal)
    mbf_ac.wait_for_result()
    return mbf_ac.get_result()
```

and can check for additional, rich result information like outcome, message and others (see first Section overview of `mbf_msgs/MoveBaseAction`)

```python
if result.outcome != mbf_msgs.MoveBaseResult.SUCCESS:
    rospy.loginfo("Unable to complete action: %s", result.message)
    return 
```

<br>

### The Result

![](../../img/turtlebot_mbf_circle.gif)

<br>

The full source code can be found [here](https://github.com/uos/mbf_tutorials/tree/master/beginner).