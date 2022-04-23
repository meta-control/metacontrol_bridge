// Copyright 2022 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iostream>
#include <memory>
#include <utility>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "std_msgs/String.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "metacontrol_bridge/MetacontrolNode.hpp"


class DemoNode : public metacontrol_bridge::MetacontrolNode
{
public:
  DemoNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : MetacontrolNode(node_name, options)
  {
  }
};

int main(int argc, char * argv[])
{
  // ROS 2 node
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DemoNode>("demo_bridge");

  // ROS 1 node
  ros::init(argc, argv, "demo_bridge");
  ros::NodeHandle n;

  rclcpp::Rate rate(10);
  while (ros::ok() && rclcpp::ok()) {
    ros::spinOnce();
    rclcpp::spin_some(node->get_node_base_interface());
    rate.sleep();
  }
  
  ros::shutdown();
  rclcpp::shutdown();

  return 0;
}
