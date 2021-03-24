# Continuous Replanning

Up to this point in the MBF Tutorials the general planning flow has been to generate a global plan to the target once, and handing over the path to the local planner/controller to execute the plan. This behavior is inherently unable to handle dynamic changes to the environment the robot is in: After a certain amount of necessary change to the global plan, the local_planner/controller will fail. The amount of change that the navigation stack can handle is highly dependent on the specific controller.

This is were **continuous replanning** comes into play: To make the local planners life easier, we will use the globbal planner to replan from the current state in continous intervals.

## mbf_msgs/MoveBaseAction

Move Base Flex has continuous replanning built into the `mbf_msgs/MoveBaseAction` ROS service call, which makes continuous replanning in simple dynamic environments very easy!