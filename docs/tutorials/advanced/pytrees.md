# PyTrees Move Base Flex Tutorial (Python)

[`py_trees_ros`](https://py-trees-ros-tutorials.readthedocs.io) is a Python-based behavior tree implementation and may be easier for you to use, depending on your background. If you are looking for C++ based Behavior Trees, try the [previous tutorial](./behavior_tree.md)

## What you will learn 

In this tutorial we address the actions `GetPath`, `ExePath` and `Recovery` provided by Move Base Flex. While GetPath runs the global planner searching for a path to the target pose, `ExePath` runs the local planner executing the planned path. The Recovery action can be used to execute various behaviors for error handling during planning and controlling. We connect these actions by setting up a `py_trees_ros` Behavior Tree (BT from now on) using ActionClient Behaviors. In addition to the actions described above, the implementation of a state that receives a navigation goal by the user is required. The target pose can be easily set via the visualization tool RViz and published on a specific topic. 

This tutorial very closely follows the [ROS Wiki](https://wiki.ros.org/move_base_flex/Tutorials/BehaviourTreesForMoveBaseFlex).

## PyTrees

To learn about BTs and the particular library used here I encourage you to read [`py-trees` documentation](https://py-trees.readthedocs.io/en/devel/), and follow [`py_trees_ros` tutorials](https://py-trees-ros-tutorials.readthedocs.io/en/release-2.0.x/tutorials.html) to learn how to control ROS-based robots with BTs.

As a minimum requirement to understand what's coming next, be sure you understand the following concepts:

* action behavior
* check behavior
* composite
* blackboard 

## The Code 

```python
"""
MBF BT Demo: Behavior tree implementing a really basic navigation strategy,
even simpler than the move_base hardcoded FSM, as it lacks:

* continuous replanning
* oscillation detection

We create on the first place action client behaviors for MBF's planner, controller and recovery action servers
On this simple demo we need to add pretty little additional code to the base ActionClient class
"""

##############################################################################
# Imports
##############################################################################

import functools
import py_trees
import py_trees_ros
import py_trees.console as console
import rospy
import sys

import geometry_msgs.msg as geometry_msgs
import mbf_msgs.msg as mbf_msgs


##############################################################################
# Actions
##############################################################################

class GetPath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get target pose from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.GetPathGoal(target_pose=py_trees.blackboard.Blackboard().get("target_pose"))
        super(GetPath, self).initialise()

    def update(self):
        """
        On success, set the resulting path on the blackboard, so ExePath can use it
        """
        status = super(GetPath, self).update()
        if status == py_trees.Status.SUCCESS:
            py_trees.blackboard.Blackboard().set("path", self.action_client.get_result().path)
        return status

class ExePath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get path from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.ExePathGoal(path=py_trees.blackboard.Blackboard().get("path"))
        super(ExePath, self).initialise()

class Recovery(py_trees_ros.actions.ActionClient):
    def setup(self, timeout):
        """
        Read the list of available recovery behaviors so we can try them in sequence
        """
        self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
        return super(Recovery, self).setup(timeout)

    def update(self):
        """
        Try the next recovery behavior, dropping it from the list
        """
        try:
            self.action_goal = mbf_msgs.RecoveryGoal(behavior=self._behaviors.pop(0)["name"])
            return super(Recovery, self).update()
        except IndexError:
            # recovery behaviors exhausted; fail to abort navigation but restore the list for the next goal
            # TODO: this means that we won't reset the list after a successful recovery, so the list keeps shrinking
            # until fully exhausted; that's clearly not the expected operation, so I need to find a better solution
            self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
            return py_trees.Status.FAILURE


##############################################################################
# Behaviours
##############################################################################

def create_root():
    # Create all behaviours
    bt_root = py_trees.composites.Sequence("MBF BT Demo")
    get_goal = py_trees.composites.Selector("GetGoal")
    fallback = py_trees.composites.Selector("Fallback")
    navigate = py_trees.composites.Sequence("Navigate")
    new_goal = py_trees_ros.subscribers.ToBlackboard(name="NewGoal",
                                                     topic_name="/move_base_simple/goal",
                                                     topic_type=geometry_msgs.PoseStamped,
                                                     blackboard_variables = {'target_pose': None})
    have_goal = py_trees.blackboard.CheckBlackboardVariable(name="HaveGoal", variable_name="target_pose")
    clr_goal1 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    clr_goal2 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
    get_path = GetPath(name="GetPath",
                       action_namespace="/move_base_flex/get_path",
                       action_spec=mbf_msgs.GetPathAction)
    exe_path = ExePath(name="ExePath",
                       action_namespace="/move_base_flex/exe_path",
                       action_spec=mbf_msgs.ExePathAction)
    recovery = Recovery(name="Recovery",
                        action_namespace="/move_base_flex/recovery",
                        action_spec=mbf_msgs.RecoveryAction)

    # Compose tree
    bt_root.add_children([get_goal, fallback])
    get_goal.add_children([have_goal, new_goal])
    navigate.add_children([get_path, exe_path, clr_goal1])
    fallback.add_children([navigate, recovery, clr_goal2])
    return bt_root


def shutdown(behaviour_tree):
    behaviour_tree.interrupt()

if __name__ == '__main__':
    rospy.init_node("mbf_bt_demo")
    root = create_root()
    behaviour_tree = py_trees_ros.trees.BehaviourTree(root)
    rospy.on_shutdown(functools.partial(shutdown, behaviour_tree))
    if not behaviour_tree.setup(timeout=15):
        console.logerror("failed to setup the tree, aborting.")
        sys.exit(1)

    behaviour_tree.tick_tock(500)
```

## The Code Explained

### Behaviors

Our tree requires five action behaviors: `NewGoal`, `ClearGoal`, `GetPath`, `ExePath` and `Recovery` and one check behavior: `HaveGoal`. 

#### NewGoal

Move Base Flex expects a `geometry_msgs/PoseStamped` on topic `/move_base_simple/goal`. This goal can come from Rviz or can be published to topic directly.

To create a `NewGoal` action behavior, add the following lines to your code: 

```python
new_goal = py_trees_ros.subscribers.ToBlackboard(name="NewGoal",
                                                 topic_name="/move_base_simple/goal",
                                                 topic_type=geometry_msgs.PoseStamped,
                                                 blackboard_variables = {'target_pose': None})
```

#### GetPath

The following lines declare the class `GetPath` extending `ActionClient` and create an action behavior we can later add to the tree.

Ensure that you declare the correct namespace for the action (or remap appropriately with the launch file). In this simple demo we need to add very little additional code to the base `ActionClient` class, just gather data from the blackboard that is required to create the goal and add the result back to the blackboard, so other actions can use it. 

```python
class GetPath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get target pose from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.GetPathGoal(target_pose=py_trees.blackboard.Blackboard().get("target_pose"))
        super(GetPath, self).initialise()

    def update(self):
        """
        On success, set the resulting path on the blackboard, so ExePath can use it
        """
        status = super(GetPath, self).update()
        if status == py_trees.Status.SUCCESS:
            py_trees.blackboard.Blackboard().set("path", self.action_client.get_result().path)
        return status

get_path = GetPath(name="GetPath",
                   action_namespace="/move_base_flex/get_path",
                   action_spec=mbf_msgs.GetPathAction)
```

#### ExePath

`ExePath` is very similar to `GetPath`

```python
class ExePath(py_trees_ros.actions.ActionClient):

    def initialise(self):
        """
        Get path from the blackboard to create an action goal
        """
        self.action_goal = mbf_msgs.ExePathGoal(path=py_trees.blackboard.Blackboard().get("path"))
        super(ExePath, self).initialise()

exe_path = ExePath(name="ExePath",
                   action_namespace="/move_base_flex/exe_path",
                   action_spec=mbf_msgs.ExePathAction)
```

Be sure to set the correct namespace. We only use the goal path, not the result.

#### Recovery 

The Recovery action is slightly more complicated because we need to manage the list of available recovery behaviors after retrieving them from ROS parameters server. Every time the action is executed, we try the next recovery behavior, dropping it from the list. Once exhausted, we fail to abort navigation, but also restore the list for the next goal. 

```python
class Recovery(py_trees_ros.actions.ActionClient):
    def setup(self, timeout):
        """
        Read the list of available recovery behaviors so we can try them in sequence
        """
        self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
        return super(Recovery, self).setup(timeout)

    def update(self):
        """
        Try the next recovery behavior, dropping it from the list
        """
        try:
            self.action_goal = mbf_msgs.RecoveryGoal(behavior=self._behaviors.pop(0)["name"])
            return super(Recovery, self).update()
        except IndexError:
            # recovery behaviors exhausted; fail to abort navigation but restore the list for the next goal
            # TODO: this means that we won't reset the list after a successful recovery, so the list keeps shrinking
            # until fully exhausted; that's clearly not the expected operation, so I need to find a better solution
            self._behaviors = rospy.get_param("/move_base_flex/recovery_behaviors")
            return py_trees.Status.FAILURE

recovery = Recovery(name="Recovery",
                    action_namespace="/move_base_flex/recovery",
                    action_spec=mbf_msgs.RecoveryAction)
```

#### Others

The remaining action and check behaviors are much simpler and require a single line of code each. `HaveGoal` simple checks if "target_pose" variable is on the blackboard, while `ClearGoal` (used twice) removes the same variable from the blackboard: 

```
have_goal = py_trees.blackboard.CheckBlackboardVariable(name="HaveGoal", variable_name="target_pose")
clr_goal1 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
clr_goal2 = py_trees.blackboard.ClearBlackboardVariable(name="ClearGoal", variable_name="target_pose")
```

#### Composites

For the simple behavior implemented here we need just four composites, two sequences and two selectors: 

```python
bt_root = py_trees.composites.Sequence("MBF BT Demo")
get_goal = py_trees.composites.Selector("GetGoal")
fallback = py_trees.composites.Selector("Fallback")
navigate = py_trees.composites.Sequence("Navigate")
```

#### Composing the Tree

```python
bt_root.add_children([get_goal, fallback])
get_goal.add_children([have_goal, new_goal])
navigate.add_children([get_path, exe_path, clr_goal1])
fallback.add_children([navigate, recovery, clr_goal2])
```

#### GUI

To display the BT with an rqt viewer, run:

```bash
rosrun rqt_py_trees rqt_py_trees
```

This a very useful tool to verify that your code actually composes the tree you have designed. Additionally, the viewer will help you to debug, as nodes are highlighted with a color scheme to show the current execution state:

* GREEN for SUCCESS
* RED for FAILURE
* BLUE for RUNNING
* GREY for unvisited. 

The GUI also provides:

* Tooltips : hover over a behavior to catch name, type, status and feedback message information
* Timeline : rewind as you wish, note the bars indicating where important events occurred 