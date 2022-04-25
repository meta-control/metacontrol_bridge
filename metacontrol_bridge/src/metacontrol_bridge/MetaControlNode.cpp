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

  auto params = list_parameters({}, 10);
  auto param_types = get_parameter_types(params.names);

  for (size_t i = 0; i < params.names.size(); i++) {
    const std::string & param_key = params.names[i];
    std::string ros1_param_name = "/" + std::string(get_name()) + "/" + param_key;

    switch(param_types[i]) {
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL:
        nh_.setParam(ros1_param_name, get_parameter(param_key).get_value<bool>());
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER:
        {
          int int_value = get_parameter(param_key).get_value<int>();
          nh_.setParam(ros1_param_name, int_value);
        }
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE:
        nh_.setParam(ros1_param_name, get_parameter(param_key).get_value<double>());
        break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING:
       nh_.setParam(ros1_param_name, get_parameter(param_key).get_value<std::string>());
       break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_BYTE_ARRAY:
       // nh_.setParam(ros1_param_name, get_parameter(param_key).get_value<std::vector<uint8_t>>());
       RCLCPP_WARN(get_logger(), "bridge for PARAMETER_BYTE_ARRAY not implemented yet");
       break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_BOOL_ARRAY:
       {
         std::vector<bool> vbool_value = get_parameter(param_key).get_value<std::vector<bool>>();
         nh_.setParam(ros1_param_name, vbool_value);
       }
       break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER_ARRAY:
       {
         RCLCPP_WARN(get_logger(), "bridge for PARAMETER_BYTE_ARRAY not implemented yet");
         // std::vector<int> vint_values = get_parameter(param_key).get_value<std::vector<int>>();
         // nh_.setParam(ros1_param_name, vint_values);
       }
       break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE_ARRAY:
       {
         std::vector<double> vdouble_values = get_parameter(param_key).get_value<std::vector<double>>();
         nh_.setParam(ros1_param_name, vdouble_values);
       }
       break;
      case rcl_interfaces::msg::ParameterType::PARAMETER_STRING_ARRAY:
       {
         std::vector<std::string> vstring_values = get_parameter(param_key).get_value<std::vector<std::string>>();
         nh_.setParam(ros1_param_name, vstring_values);
       }
       break;
    };
  }

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
