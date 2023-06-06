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

#include "kortex2_controllers/kortex_command_mode_controller.hpp"

namespace kortex2_controllers
{

controller_interface::InterfaceConfiguration KortexCommandModeController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("kortex_command/servo_mode");
  config.names.emplace_back("kortex_command/control_mode");

  return config;
}

controller_interface::InterfaceConfiguration KortexCommandModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;

  return config;
}

CallbackReturn KortexCommandModeController::on_init() { return CallbackReturn::SUCCESS; }

CallbackReturn KortexCommandModeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  servo_mode_command_subscriber_ = get_node()->create_subscription<ServoMode>(
    "~/kortex_servo_mode", rclcpp::SystemDefaultsQoS(),
    [this](const ServoMode::SharedPtr msg) { rt_servo_mode_ptr_.writeFromNonRT(msg); });

  control_mode_command_subscriber_ = get_node()->create_subscription<ControlMode>(
    "~/kortex_control_mode", rclcpp::SystemDefaultsQoS(),
    [this](const ControlMode::SharedPtr msg) { rt_control_mode_ptr_.writeFromNonRT(msg); });

  RCLCPP_INFO(get_node()->get_logger(), "configure successful");

  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexCommandModeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  resetCommandInterfaces();
    // Action interface
  set_mode_command_srv_ = get_node()->create_service<SetModeCommand>(
    "~/set_kortex_cmd_mode",
    std::bind(
      &KortexCommandModeController::updateCommandMode, this, std::placeholders::_1, std::placeholders::_2));
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexCommandModeController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  resetCommandInterfaces();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type KortexCommandModeController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  const auto servo_mode = rt_servo_mode_ptr_.readFromRT();
  const auto control_mode = rt_control_mode_ptr_.readFromRT();

  command_interfaces_[CommandInterfaces::SERVOING_MODE].set_value(static_cast<double>((*servo_mode)->mode));
  command_interfaces_[CommandInterfaces::CONTROL_MODE].set_value(static_cast<double>((*control_mode)->mode));

  return controller_interface::return_type::OK;
}

void KortexCommandModeController::resetCommandInterfaces()
{
  // reset command buffer 
  rt_servo_mode_ptr_ = ServoModeCommandPtr(nullptr);
  rt_control_mode_ptr_ = ControlModeCommandPtr(nullptr);

  command_interfaces_[CommandInterfaces::SERVOING_MODE].set_value(static_cast<double>(ServoMode::SINGLE_LEVEL_SERVOING));
  command_interfaces_[CommandInterfaces::CONTROL_MODE].set_value(static_cast<double>(ControlMode::JOINT));
}

bool KortexCommandModeController::updateCommandMode(const SetModeCommand::Request::SharedPtr req, SetModeCommand::Response::SharedPtr resp){
  RCLCPP_INFO(get_node()->get_logger(), "Updating control mode and kortex api servo level");

  command_interfaces_[CommandInterfaces::SERVOING_MODE].set_value(static_cast<double>((req->target_servo_mode.mode)));
  command_interfaces_[CommandInterfaces::CONTROL_MODE].set_value(static_cast<double>((req->target_control_mode.mode)));

  // sleep to let hardware_interface set these values
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  resp->success = true;
  resp->message = "Kortex command mode updated";

  return resp->success;
}

}  // namespace kortex2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex2_controllers::KortexCommandModeController, controller_interface::ControllerInterface)
