# Writing a SMACH

This Tutorial will walk you through writing a simple SMACH. SMACH stands for **S**tate **Mach**ine and is an important concept in the ROS Navigation ecosystem. You can check out the official [ROS SMACH Tutorials](https://wiki.ros.org/smach/Tutorials), as well.

The following SMACH tutorial will perform the circular path you already know from previous tutorials by using path planning with the `/mbf_msgs/GetPath.action` and execution from within a simple SMACH with `/mbf_msgs/ExePath.action`.

## Code

```python
import rospy
import smach
import smach_ros
import threading

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from mbf_msgs.msg import ExePathAction
from mbf_msgs.msg import GetPathAction
from mbf_msgs.msg import RecoveryAction


def create_pose(x, y, z, xx, yy, zz, ww):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    pose.pose.orientation.x = xx
    pose.pose.orientation.y = yy
    pose.pose.orientation.z = zz
    pose.pose.orientation.w = ww
    return pose


def iterate_target_poses():
    target_poses = [   
        create_pose(-1.75, 0.74, 0, 0, 0, 0.539, 0.843),
        create_pose(-0.36, 1.92, 0, 0, 0, -0.020, 0.999),
        create_pose(0.957, 1.60, 0, 0, 0, -0.163, 0.987),
        create_pose(1.8741, 0.3830, 0, 0, 0, -0.70, 0.711),
        create_pose(1.752, -0.928, 0, 0, 0, -0.856, 0.517),
        create_pose(0.418, -2.116, 0, 0, 0, 0.998, 0.0619),
        create_pose(-0.775, -1.80, 0, 0, 0, 0.954, 0.300),
        create_pose(-1.990, -0.508, 0, 0, 0, -0.112, 0.999)
    ]

    for target_pose in target_poses:
        yield target_pose

def create_path_goal(path, tolerance_from_action, dist_tolerance, angle_tolerance):
    goal = mbf_msgs.ExePathGoal()
    goal.path = path
    goal.tolerance_from_action = tolerance_from_action
    goal.dist_tolerance = dist_tolerance
    goal.angle_tolerance = angle_tolerance
    return goal

def main():
    rospy.init_node('mbf_state_machine')

    target_poses = iterate_target_poses()

    # Create SMACH state machine
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])

    # Define userdata
    sm.userdata.target_pose = None
    sm.userdata.path = None
    sm.userdata.error = None
    sm.userdata.clear_costmap_flag = False
    sm.userdata.error_status = None

    with sm:
        # path callback
        def get_path_callback(userdata, goal):
            try:
                goal.target_pose = next(target_poses)
            except StopIteration:
                rospy.logwarn("Reached last target pose")
                rospy.signal_shutdown("Last goal reached. Shutting down")

        # Get path
        smach.StateMachine.add(
            'GET_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/get_path',
                GetPathAction,
                goal_cb=get_path_callback,
                goal_slots=['target_pose'],
                result_slots=['path']
            ),
            transitions={
                'succeeded': 'EXE_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )

        def path_callback(userdata, goal):
            target_pose = goal.path.poses[-1].pose
            rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.position.x, target_pose.position.y)

        # Execute path
        smach.StateMachine.add(
            'EXE_PATH',
            smach_ros.SimpleActionState(
                '/move_base_flex/exe_path',
                ExePathAction,
                goal_cb=path_callback,
                goal_slots=['path']
            ),
            transitions={
                'succeeded': 'GET_PATH',
                'aborted': 'aborted',
                'preempted': 'preempted'
            }
        )

    # Execute SMACH plan
    # Create a thread to execute the smach container
    smach_thread = threading.Thread(target=sm.execute)
    smach_thread.start()

    # Wait for ctrl-c
    rospy.spin()

    # Request the container to preempt
    sm.request_preempt()

    # Block until everything is preempted 
    smach_thread.join()

if __name__=="__main__":
    main()
```

## The Code Explained

Let's define the State Machine with three outcomes: `succeeded`, `aborted` and `preempted`:

```python
# Create SMACH state machine
sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'preempted'])
```

Next, we need to add states to the SMACH. In this simple case, all we need is a state to handle path planning, `GET_PATH` and another to handle the path execution, `EXE_PATH`.

The states use [`smach_ros`](https://github.com/ros/executive_smach)

> The smach_ros package contains extensions for the SMACH library to integrate it tightly with ROS. For example, SMACH-ROS can call ROS services, listen to ROS topics, and integrate with actionlib both as a client, and a provider of action servers. SMACH is a new library that takes advantage of very old concepts in order to quickly create robust robot behavior with maintainable and modular code. 

### GET_PATH State

```python
smach.StateMachine.add(
    'GET_PATH',
    smach_ros.SimpleActionState(
        '/move_base_flex/get_path',
        GetPathAction,
        goal_cb=get_path_callback,
        goal_slots=['target_pose'],
        result_slots=['path']
    ),
    transitions={
        'succeeded': 'EXE_PATH',
        'aborted': 'aborted',
        'preempted': 'preempted'
    },
    remapping={
        'target_pose': 'goal'
    }
)
```

In our case, we need access to

* the goal, to tell out robot about the next waypoint: `goal_slots=['target_pose']`
* the path, to tell the next state which path to execute: `result_slots=['path']`

While the goal_slots define the input values, result_slots indicate the output values.

The seconds parameter tells the SMACH which action server to use to receive a path: `/move_base_flex/get_path` with the `mbf_msgs/GetPathAction` we already know from the beginner tutorials.

The `transitions` parameters configures which states are visited next, depending on the result of the state.

**Note**: If goals/results and userdata are named differently, you can change them with the remapping argument (not necessary in our case). e.g.:

```python
remapping={
    'target_pose': 'goal'
}
```

Next use this callback to set the goal. After we set the goal, the state calls the `/move_base_flex/get_path` action server, and relays the result to the next state (Shown shortly).

```python
def get_path_callback(userdata, goal):
    try:
        goal.target_pose = next(target_poses)
    except StopIteration:
        rospy.logwarn("Reached last target pose")
        rospy.signal_shutdown("Last goal reached. Shutting down")
```

### `EXE_PATH` State

```python
smach.StateMachine.add(
    'EXE_PATH',
    smach_ros.SimpleActionState(
        '/move_base_flex/exe_path',
        ExePathAction,
        goal_cb=path_callback,
        goal_slots=['path']
    ),
    transitions={
        'succeeded': 'GET_PATH',
        'aborted': 'aborted',
        'preempted': 'preempted'
    }
)
```

the `EXE_PATH` state receives a path via userdata an is declared with `goal_slots=['path']`. On success, the path will transition to the `GET_PATH` state and will try to plan a new path from there.

The `exe_path_callback` is used for verbosity only

```python
def exe_path_callback(userdata, goal):
        target_pose = goal.path.poses[-1].pose
        rospy.loginfo("Attempting to reach (%1.3f, %1.3f)", target_pose.position.x, target_pose.position.y)
```


### Userdata

Finally, the result and goal slots we used to determine how the data will be passed between states, need to be set in the [userdata](http://wiki.ros.org/smach/Tutorials/User%20Data):

```python
# Define userdata
sm.userdata.goal = None
sm.userdata.path = None
```

## Run the SMACH

Launch gazebo

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

as well as Move Base Flex

```bash
export TURTLEBOT3_MODEL=burger
roslaunch mbf_advanced amcl_demo_mbf.launch
```

Finally, start the SMACH

```bash
rosrun mbf_advanced circle_smach.py
```
