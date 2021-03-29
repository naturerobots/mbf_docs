# Basic Navigation

## Run tutorial

For a quick demo, just follow the following steps. You will learn how to control the turtlebot in a simulation environment, with the help of RViz. Because this is so simple (in principle), we will relay Move Base Messages to Move Base Flex and let Move Base Flex take over planning. Let's begin.

<br>

Clone [mbf_tutorials](https://github.com/uos/mbf_tutorials) 
```bash
git clone git@github.com:uos/mbf_tutorials.git ~/catkin_ws/src/mbf_tutorials
```
Install dependencies with rosdep (include all tutorials for ease of use)
```bash
rosdep install mbf_beginner mbf_advanced
```

??? question "Using ROS noetic?"
    rosdep install will fail for `eband_local_planner`, because the team hasn't released the version for noetic yet. Simply clone the repository:
    ```
    git clone git@github.com:utexas-bwi/eband_local_planner.git ~/catkin_ws/src/eband_local_planner
    ```

From source of workspace: 
```bash
cd ~/catkin_ws/src
catkin_make -j4 && source devel/setup.bash`
```

Launch the appropriate gazebo world
```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch

```
Start localization (AMCL)
```bash
export TURTLEBOT3_MODEL=burger
roslaunch mbf_beginner amcl_demo_relay_subscriber.launch
```

Launch Rviz
```bash
roslaunch mbf_beginner rviz.launch
```

In RViz

* Set an Initial Pose estimate with the `2D Pose Estimate` Pose
* Finally set your Navigation Goal with the `2D Nav Goal` Pose

You will now be able to navigate in a similar fashion to this:

![](../../img/demo.gif)


<br>

## What is happening here? 

We used Move Base Flex by relaying `mb_msgs/MoveBaseAction` to `mbf_msgs/MoveBaseAction` in a standard Move Base goal callback. This is useful in case you want to use Move Base Flex as a drop-in replacement for Move Base and want to take advantage of continous replanning, which is built into Move Base Flex, but not Move Base.

### Code

```python
import actionlib
import rospy
import nav_msgs.srv as nav_srvs
import mbf_msgs.msg as mbf_msgs
import move_base_msgs.msg as mb_msgs
from geometry_msgs.msg import PoseStamped

def simple_goal_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
    rospy.logdebug("Relaying move_base_simple/goal pose to mbf")

    mbf_mb_ac.wait_for_result()

    status = mbf_mb_ac.get_state()
    result = mbf_mb_ac.get_result()

    rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)

if __name__ == '__main__':
    rospy.init_node("move_base_relay")

    # move base flex ation client relays incoming mb goals to mbf
    mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    mbf_mb_ac.wait_for_server(rospy.Duration(20))

    # move_base simple topic and action server
    mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)

    rospy.spin()
```


### The Code Explained
    

MoveBase subscriber to handle goal events

```python
mb_sg = rospy.Subscriber('move_base_simple/goal', PoseStamped, simple_goal_cb)
```

MoveBase expects goal Messages (`geometry_msgs/Pose`) on topic `move_base_simple/goal`. The subscriber callback `simple_goal_cb` handles the `mbf_msgs.MoveBaseAction` ROS Action Client. The Move Base Flex SimpleActionServer is launched from within Move Base Flex.

```python
mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
...

def simple_goal_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg))
```
and relays the MoveBaseAction to the Move Base Flex action client!

At this stage, we are using the global planner and local planner defined in [move_base.yml](https://github.com/uos/mbf_tutorials/blob/master/beginner/param/move_base.yaml).


## A Relay with more control

While the first example allows you to relay messages to Move Base Flex, the only way to reach goals is by setting a 2D Navigation Goal via RViz, which can be limiting. This examples allows you to send goals directly from a ROS node.

### Code

=== "Server"

    ```python
    import actionlib
    import rospy
    import mbf_msgs.msg as mbf_msgs
    import move_base_msgs.msg as mb_msgs

    def mb_execute_cb(msg):
        mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg.target_pose),
                            feedback_cb=mbf_feedback_cb)

        rospy.logdebug("Relaying move_base goal to mbf")
        mbf_mb_ac.wait_for_result()

        status = mbf_mb_ac.get_state()
        result = mbf_mb_ac.get_result()

        rospy.logdebug("MBF execution completed with result [%d]: %s", result.outcome, result.message)
        if result.outcome == mbf_msgs.MoveBaseResult.SUCCESS:
            mb_as.set_succeeded(mb_msgs.MoveBaseResult(), "Goal reached.")
        else:
            mb_as.set_aborted(mb_msgs.MoveBaseResult(), result.message)

    def mbf_feedback_cb(feedback):
        mb_as.publish_feedback(mb_msgs.MoveBaseFeedback(base_position=feedback.current_pose))

    if __name__ == '__main__':
        rospy.init_node("move_base")

        # move_base_flex get_path and move_base action clients
        mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        mbf_mb_ac.wait_for_server(rospy.Duration(10))

        mb_as = actionlib.SimpleActionServer('move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
        mb_as.start()

        rospy.spin()
    ```

=== "Client"

    ```python
    import rospy
    import actionlib
    import mbf_msgs.msg as mbf_msgs
    import move_base_msgs.msg as mb_msgs
    from actionlib_msgs.msg import GoalStatus

    def create_goal(x, y, z, xx, yy, zz, ww):
        goal = mb_msgs.MoveBaseGoal()
        ...
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        ...
        return goal

    def move(goal):
        client.send_goal(goal)
        client.wait_for_result()
        return client.get_state() == GoalStatus.SUCCEEDED


    def drive_circle():
        goals = [   create_goal(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
                    create_goal(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
                    ...
        ]

        for goal in goals:
            rospy.loginfo("Attempting to drive to %s %s", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y)
            if not move(goal):
                return False

        return True

    if __name__ == '__main__':
        try:
            rospy.init_node('mb_relay_client')
            
            client = actionlib.SimpleActionClient('move_base', mb_msgs.MoveBaseAction)
            client.wait_for_server(rospy.Duration(10))
            rospy.loginfo("Connected to SimpleActionServer 'move_base'")

            result = drive_circle()
            rospy.loginfo("Drove circle with result: %s", result)
            
        except rospy.ROSInterruptException:
            rospy.logerror("program interrupted before completion")
    ```

### The Code Explained

On the server side, we start a standard Move Base Action Server, and connect a Move Base Flex Action Client to the default Move Base Flex Action Server.

```python
mbf_mb_ac = actionlib.SimpleActionClient("move_base_flex/move_base", mbf_msgs.MoveBaseAction)
mbf_mb_ac.wait_for_server(rospy.Duration(10))

mb_as = actionlib.SimpleActionServer('move_base', mb_msgs.MoveBaseAction, mb_execute_cb, auto_start=False)
```

We then relay the goal in the callback of the Move Base Action Server, like in the first subriber callback example

```python
def mb_execute_cb(msg):
    mbf_mb_ac.send_goal(mbf_msgs.MoveBaseGoal(target_pose=msg.target_pose),
                        feedback_cb=mbf_feedback_cb)
```

On the client side, we simply connect to the Move Base Action Server, and send a goal, which is then relayed in the above function.

```python        
client = actionlib.SimpleActionClient('move_base', mb_msgs.MoveBaseAction)
client.wait_for_server(rospy.Duration(10))
rospy.loginfo("Connected to SimpleActionServer 'move_base'")
```

In this example, the robot will follow a (rough) circular path around the turtlebot3\_gazebo world. A goal is created like this:

```python
goal = mb_msgs.MoveBaseGoal()
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

and sent to the action server:

```python
client.send_goal(goal)
client.wait_for_result()
return client.get_state() == GoalStatus.SUCCEEDED
```

### Run the example

Launch gazebo

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

as well as the server 

```bash
export TURTLEBOT3_MODEL=burger
roslaunch mbf_beginner amcl_demo_relay_server.launch
```

and client node to send the goals!

```bash
rosrun mbf_beginner mb_relay_client.py
```

<br>

### The Result

Open RViz with

```bash
roslaunch mbf_beginner rviz.launch
```

![](../../img/turtlebot_mbf_circle.gif)

<br>

The full source code can be found [here](https://github.com/uos/mbf_tutorials/tree/master/beginner).

