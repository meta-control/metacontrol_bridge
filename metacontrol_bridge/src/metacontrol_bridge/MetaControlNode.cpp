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

#include "metacontrol_bridge/MetacontrolNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace metacontrol_bridge
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

MetacontrolNode::MetacontrolNode(
  const std::string & node_name,
  const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, options)
{
  trigger_client_ = nh_.serviceClient<lifecycle_msgs::ChangeState>(node_name + "/set_state");
}

CallbackReturn
MetacontrolNode::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  return trigger_trasition_ros1(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
}

CallbackReturn
MetacontrolNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  return trigger_trasition_ros1(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
}

CallbackReturn
MetacontrolNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  return trigger_trasition_ros1(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
}

CallbackReturn
MetacontrolNode::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void) previous_state;
  return trigger_trasition_ros1(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
}

CallbackReturn
MetacontrolNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
  if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {
    return trigger_trasition_ros1(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  } else if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    return trigger_trasition_ros1(lifecycle_msgs::msg::Transition::TRANSITION_INACTIVE_SHUTDOWN);
  } else if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return trigger_trasition_ros1(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVE_SHUTDOWN);
  }

  return CallbackReturn::FAILURE;
}

CallbackReturn
MetacontrolNode::trigger_trasition_ros1(const uint8_t transition)
{
  if (trigger_client_.waitForExistence(ros::Duration(1))) {
    lifecycle_msgs::ChangeState srv;
    srv.request.transition.id = transition;

    if (trigger_client_.call(srv)) {
      return srv.response.success? CallbackReturn::SUCCESS : CallbackReturn::FAILURE;
    } else {
      RCLCPP_WARN(get_logger(), "Service %s/set_state not available", get_name()); 
      return CallbackReturn::FAILURE;
    }
  } else {
    RCLCPP_WARN(get_logger(), "Service %s/set_state not available", get_name()); 
    return CallbackReturn::FAILURE;
  }
}

}  // namespace metacontrol_bridge
