# metacontrol_bridge

Metacontrol Bridge is a project that aims to allow a node in ROS1 to change its operating mode from the System Modes in ROS2.

[System Modes](https://github.com/micro-ROS/system_modes) allow you to control a LifeCycle node in ROS2 by:
* Activation/Deactivation.
* Establish some parameters that modulate the operation of the node.


The operating scheme is seen in the following figure:

![metacontrol_bridge](https://user-images.githubusercontent.com/3810011/165271381-ce98ed24-fc69-4b22-a1cc-0a3bc687becb.png)


* Node C is a ROS1 node that we would like to have similar capabilities to a LifeCycle ROS node.
* We also want, through System Modes, to be able to change parameters in C.
* C is a LifeCycle node that makes the new parameters effective when it goes from unconfigured to inactive.
* C' is a proxy for C that acts as a bridge between ROS1 and ROS2 so that C can attend to state changes and dictate the System Modes.
* The figure indicates the workspace where each set of nodes is compiled and executed. Only the ROS1 and ROS2 distributions should be mixed for the bridge nodes.

This repository has two branches: `noetic` and `Foxy` depending on the version of ROS you are targeting.

## Singularity Workspace (ROS1)

The goal is to [have a node](https://github.com/meta-control/metacontrol_bridge/blob/noetic/metacontrol_bridge_demo/scripts/demo_node.py) in ROS1 (only python for now) that looks like this:

```
#!/usr/bin/env python
import rospy
from metacontrol_bridge.metacontrol_bridge import MetacontrolNode

class MyNode(MetacontrolNode):
    def __init__(self):
        MetacontrolNode.__init__(self)
    
    def on_configure(self, previous):
        print("CONFIGURING")

        param0_value = rospy.get_param('~param0', 1.0)
        param1_value = rospy.get_param('~param1', "hello")

        print("param0 value is [%lf]" % param0_value)
        print("param1 value is [%s]" % param1_value)

        return True

    def on_cleanup(self, previous):
        print("CLEANING UP")
        return True

    def on_activate(self, previous):
        print("ACTIVATING")
        return True

    def on_deactivate(self, previous):
        print("DEACTIVATING")
        return True

    def on_shutdown(self, previous):
        print("SHUTTING DOWN")
        return True

if __name__ == '__main__':
  rospy.init_node('demo_bridge_node')
  my_node = MyNode()

  rospy.spin()
```

This is the node whose state and parameters can be changed from ROS2. This node has a `~/change_state` service that allows its state to be changed, using a service in the interfaces of the [`lifecycle_msgs` package](https://github.com/meta-control/metacontrol_bridge/tree/noetic/lifecycle_msgs).

### Build

```
cd <your ros1 workspace>
cd src
git clone -b noetic https://github.com/meta-control/metacontrol_bridge.git`
cd ..
catkin_make
```

## metacontrol_bridge workspace

The bridge node is in [a program](https://github.com/meta-control/metacontrol_bridge/blob/foxy/metacontrol_bridge/src/demo_node.cpp) that creates a metacontrol_bridge::MetacontrolNode node for each node on ROS1 that we want to connect to ROS2. In each of these nodes, the parameters that are in the ROS1 node that are controlled by the System Modes must be declared.

```
class DemoNode : public metacontrol_bridge::MetacontrolNode
{
public:
  DemoNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MetacontrolNode(node_name, options)
  {
    declare_parameter<double>("param0", 2.0);
    declare_parameter<std::string>("param1", "bye");
  }
};
```

### Build

```
source /opt/ros/foxy/setup.bash
source <your ros1 workspace>/devel/setup.bash
mkdir -p metacontrol_bridge_ws/src
cd metacontrol_bridge_ws/src
colcon build --symlink-install
```

## metacontrol workspace

In this workspace, run the ROS2 nodes you need, and the System Modes mechanisms.

