# Behavior Trees

Behavior Trees are an interesting alternative to State Machines. Actions between states in a SMACH become leaves in a tree, and moving between states in a SMACH is handles by the root of the tree in Behavior Tree based planning. For more details, visit [this overview](https://www.behaviortree.dev/bt_basics/).

We will use a Behavior Tree based on the [BehaviorTree.CPP library](https://github.com/BehaviorTree/BehaviorTree.CPP) to implement the following algorithm:

![](../../img/bt.png)

You could express the tree in the following way:

> First, drive Home. 

> Once you've reached home, repeat:

> * Attempt to move to next goal

> * If that fails, attempt to skip to next goal.

> * If that fails, attempt to move back.

> * If that fails, attempt to skip over the move back.

> When all of that fails (because we reached end of circle, or fatal planner error), force BT to go drive Home (ForceSucess)


*It is important to understand that this fallback behavior is implemented __separately__ from the Move Base Flex recovery plugin infrastructure covered in the [previous tutorial](./recovery_behavior.md).*

## The Code

```c++
#include <fstream>
#include <ros/ros.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <mbf_advanced/mbf_circle_client.h>

using State = mbf_advanced::MBFCircleClientState;

BT::NodeStatus DriveHome(std::shared_ptr<mbf_advanced::MBFCircleClient>& mbfclient)
{
    ROS_INFO_STREAM("BT: driving home");
    return mbfclient->driveHome() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

class AttemptNext : public BT::SyncActionNode
{
public:
    explicit AttemptNext(const std::string& name)
      : BT::SyncActionNode(name, {})
      , mbfclient_{}
    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    BT::NodeStatus tick() override
    {
        if (mbfclient_)
        {
            ROS_INFO_STREAM("BT: " << this->name());

            while (mbfclient_->next_move() == State::MOVING) {}

            return BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
};

class AttemptSkip : public BT::SyncActionNode
{
public:
    explicit AttemptSkip(const std::string& name)
      : BT::SyncActionNode(name, {})
      , mbfclient_{}
    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    BT::NodeStatus tick() override
    {
        if (mbfclient_)
        {
            ROS_INFO_STREAM("BT: " << this->name());
            return (mbfclient_->next_move() == State::MOVING) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
};

class AttemptPrevious : public BT::SyncActionNode
{
public:
    AttemptPrevious(const std::string& name)
            : SyncActionNode(name, {})
            , mbfclient_{}
    { }

    void attachMBFClient(std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient)
    {
        mbfclient_ = mbfclient;
    }

    BT::NodeStatus tick() override
    {
        if (mbfclient_)
        {
            ROS_INFO_STREAM("BT: " << this->name());
            return (mbfclient_->prev_move() == State::MOVING) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
        }

        return BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<mbf_advanced::MBFCircleClient> mbfclient_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "behavior_tree");
    ros::NodeHandle n;

    auto mbfclient = std::make_shared<mbf_advanced::MBFCircleClient>(std::move(mbf_advanced::loadPoseGoals(POSE_PATH)));

    BT::BehaviorTreeFactory factory;
    factory.registerSimpleCondition("DriveHomeStart", std::bind(DriveHome, std::ref(mbfclient)));
    factory.registerNodeType<AttemptNext>("AttemptNext");
    factory.registerNodeType<AttemptSkip>("AttemptSkip");
    factory.registerNodeType<AttemptPrevious>("AttemptPrevious");
    factory.registerNodeType<AttemptPrevious>("AttemptSkipPrevious");
    factory.registerSimpleCondition("DriveHomeEnd", std::bind(DriveHome, std::ref(mbfclient)));

    auto tree = factory.createTreeFromFile(BT_XML_PATH);

    for( auto& node: tree.nodes)
    {
        if( auto attempt_next = dynamic_cast<AttemptNext*>( node.get() ))
        {
            attempt_next->attachMBFClient(mbfclient);
        }

        if( auto attempt_skip = dynamic_cast<AttemptSkip*>( node.get() ))
        {
            attempt_skip->attachMBFClient(mbfclient);
        }

        if( auto attempt_prev = dynamic_cast<AttemptPrevious*>( node.get() ))
        {
            attempt_prev->attachMBFClient(mbfclient);
        }
    }

    tree.tickRoot();

    return 0;
}
```
and the corresponding `.xml` file that specifies the relations between each node

```xml
 <root main_tree_to_execute = "MainTree" >
    <BehaviorTree ID="MainTree">
        <Sequence name="endless_circle">
            <DriveHomeStart name="drive_home_start"/>
                <ForceSuccess>
                    <Repeat num_cycles="10">
                        <Fallback>
                            <AttemptNext name="attempt_next"/>
                            <AttemptSkip name="attempt_skip"/>
                            <AttemptPrevious name="attempt_previous"/>
                            <AttemptSkipPrevious name="attempt_skip_previous"/>
                        </Fallback>
                    </Repeat>
                </ForceSuccess>
            <DriveHomeEnd name="drive_home_end"/>
        </Sequence>
    </BehaviorTree>
 </root>
```

What isn't shown here is the `mbf_advanced::MBFClient`: it is a C++ node that communicated with the Move Base Flex Action server to retrieve plans (next and previous), very similar to the previous tutorials. The full code for this client can be found [here](https://github.com/uos/mbf_tutorials/blob/master/advanced/include/mbf_advanced/mbf_cpp_client.h).

## The Code explained

First, we want to drive "Home" a.k.a to the robots spawn position. This is done with a simple condition, that returns `BT::NodeStates::SUCCESS` if that happened successfully

```c++
BT::NodeStatus DriveHome(std::shared_ptr<mbf_advanced::MBFClient>& mbfclient)
{
    ROS_INFO_STREAM("BT: driving home");
    return mbfclient->driveHome() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
```

The Fallback node of the tree performs exactly one of the actions `AttemptNext` or `AttemptPrevious` and attempts each from left to right in the tree.

`AttemptNext` performs the planning of the next goal on each `tick()` and only returns `BT::NodeStatus::Failure` when the plan fails or we've reached the end of the circle.

```c++
BT::NodeStatus tick() override
{
    if (mbfclient_)
    {
        ROS_INFO_STREAM("BT: " << this->name());

        while (mbfclient_->next_move() == State::MOVING) {}

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}
```

`AttemptSkip` is very similar to `AttemptNext`, but is only called after a failed `AttemptNext` and therefore skips to next point in circle.

```c++
BT::NodeStatus tick() override
{
    if (mbfclient_)
    {
        ROS_INFO_STREAM("BT: " << this->name());
        return (mbfclient_->next_move() == State::MOVING) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}
```

`AttemptPrevious` attempts to return to the previous state and returns a `BT::NodeStatus::Failure` if that fails or we've reached the end of the circle:

```c++
BT::NodeStatus tick() override
{
    if (mbfclient_)
    {
        ROS_INFO_STREAM("BT: " << this->name());
        return (mbfclient_->prev_move() == State::MOVING) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::FAILURE;
}
```

When all Fallback options fail we want to return "Home" again. This happens after a fatal planning error or after reaching the end of the circle pose queue. By forcing the Fallback node to return Success in the xml with `ForceSuccess`

```xml
<ForceSuccess>
    <Repeat num_cycles="-1">
        <Fallback>
            <AttemptNext name="attempt_next"/>
            ...
        </Fallback>
    </Repeat>
</ForceSuccess>
```

the BT reaches the last state: drive Home.

```c++
BT::NodeStatus DriveHome(std::shared_ptr<mbf_advanced::MBFCircleClient>& mbfclient)
{
    ROS_INFO_STREAM("BT: driving home");
    return mbfclient->driveHome() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}
```

## Try it yourself

!!! Recovery Behavior
    Please note that the recovery behaviors from the [previous tutorial](./recovery_behavior.md) are explicitely disabled to make it easier for you to produce a planner error!

Launch the gazebo world for turtlebot:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Launch Move Base Flex w/o recovery mode:

```bash
export TURTLEBOT3_MODEL=burger
roslaunch mbf_advanced amcl_demo_mbf.launch
```

Launch the behavior tree

```bash
roslaunch mbf_advanced mbf_behavior_tree
```

If you want, you can try to produce fatal plans with the help of an additional square in the Gazebo world:

```
roslaunch mbf_advanced spawn_box.launch
```

Choose the object move mode in gazebo and place the square somewhere into the path of the robot, and whatch RViz!


![](../../img/bt.gif)

## Wrapping up

You may have noticed that this BT uses high level actions (drive to goal, drive home) in its leaves. Because BTs are so flexible, you could also use lower level actions in its leaves: a sequence like *NewGoal*, *ClearGoal*, *GetPath*, *ExePath* and *Recovery*. We'll do this in our [next tutorial based on `py_trees_ros`](./pytrees.md).