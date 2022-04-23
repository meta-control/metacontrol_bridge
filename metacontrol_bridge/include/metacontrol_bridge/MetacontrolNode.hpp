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


#ifndef METACONTROL_BRIDGE__METACONTROLNODE_HPP__
#define METACONTROL_BRIDGE__METACONTROLNODE_HPP__

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/ros.h"
#include "lifecycle_msgs/ChangeState.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace metacontrol_bridge
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class MetacontrolNode: public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit MetacontrolNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

protected:
  ros::NodeHandle nh_;
  ros::ServiceClient trigger_client_;

  CallbackReturn trigger_trasition_ros1(const uint8_t transition);
};

}  // namespace metacontrol_bridge

#endif  // METACONTROL_BRIDGE__METACONTROLNODE_HPP__