// Copyright 2023, PickNik Inc.
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Anthony Baker abake48@gmail.com
 * \date    2023-06-01
 *
 */
//----------------------------------------------------------------------

#ifndef KORTEX2_CONTROLLERS__KORTEX_COMMAND_MODE_CONTROLLER_HPP_
#define KORTEX2_CONTROLLERS__KORTEX_COMMAND_MODE_CONTROLLER_HPP_

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "kortex2_controllers/visibility_control.h"
#include "kortex_msgs/msg/control_mode.hpp"
#include "kortex_msgs/msg/servoing_mode.hpp"
#include "kortex_msgs/srv/set_mode.hpp"
#include "rclcpp_action/create_server.hpp"
#include "realtime_tools/realtime_buffer.h"

namespace kortex2_controllers
{
enum CommandInterfaces
{
  SERVOING_MODE = 0u,
  CONTROL_MODE = 1u,
};

using ServoMode = kortex_msgs::msg::ServoingMode;
using ControlMode = kortex_msgs::msg::ControlMode;
using SetModeCommand = kortex_msgs::srv::SetMode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class KortexCommandModeController : public controller_interface::ControllerInterface
{
public:
  KORTEX2_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  KORTEX2_CONTROLLERS_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_init() override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  KORTEX2_CONTROLLERS_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  KORTEX2_CONTROLLERS_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool updateCommandMode(const SetModeCommand::Request::SharedPtr req, SetModeCommand::Response::SharedPtr resp);

  using ServoModeCommandPtr = realtime_tools::RealtimeBuffer<std::shared_ptr<ServoMode>>;
  ServoModeCommandPtr rt_servo_mode_ptr_;
  rclcpp::Subscription<ServoMode>::SharedPtr servo_mode_command_subscriber_;

  using ControlModeCommandPtr = realtime_tools::RealtimeBuffer<std::shared_ptr<ControlMode>>;
  ControlModeCommandPtr rt_control_mode_ptr_;
  rclcpp::Subscription<ControlMode>::SharedPtr control_mode_command_subscriber_;

  rclcpp::Service<SetModeCommand>::SharedPtr set_mode_command_srv_;

  void resetCommandInterfaces();
};

}


#endif // KORTEX2_CONTROLLERS__KORTEX_COMMAND_MODE_CONTROLLER_HPP_
