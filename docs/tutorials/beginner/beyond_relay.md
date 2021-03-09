# Leaving the Relay Behind

Using a relay from Move Base to Move Base Flex is the easiest way to get started with Move Base Flex. This does, however, make it harder to use advanced features of Move Base Flex. Let's start with understanding the differences between the respective Actions:

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