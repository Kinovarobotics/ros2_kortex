// Copyright 2020 PAL Robotics S.L.
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

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "forward_command_controller/forward_command_controller.hpp"
#include "kortex2_controllers/visibility_control.h"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/subscription.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/twist.hpp"

namespace kortex2_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

/**
 * \brief Forward a twist command to the proprietary Kinova servoing API
 *
 * This jogging is similar in functionality to MoveIt Servo, but Kinova does not expose the low-level controllers in a
 * way that works well with Servo.
 *
 * This controller type inherits from a ForwardCommandController. The difference is, it takes a geometry_msgs/Twist.
 *
 * \param joints Names of the joints to control.
 * \param interface_name Name of the interface to command.
 *
 * Subscribes to:
 * - \b commands (geometry_msgs::msg::Twist) : The commands to apply in the tool frame.
 */
class TwistController : public forward_command_controller::ForwardCommandController
{
};

}  // namespace kortex2_controllers
