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
  config.names.emplace_back("kortex_command/command_mode")

  return config;
}

controller_interface::InterfaceConfiguration KortexCommandModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("kortex_command/internal_servo_mode");
  config.names.emplace_back("kortex_command/internal_command_mode");

  return config;
}

CallbackReturn KortexCommandModeController::on_init() { return CallbackReturn::SUCCESS; }

controller_interface::return_type KortexCommandModeController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (rt_servo_mode_publisher_ && rt_servo_mode_publisher_->trylock())
  {
    servo_state_.data = static_cast<uint32_t>(state_interfaces_[StateInterfaces::SERVOING_MODE].get_value());
    rt_servo_mode_publisher_->msg_.data = servo_state_.data;
    rt_servo_mode_publisher_->unlockAndPublish();
  }
  if (rt_cmd_mode_publisher_ && rt_cmd_mode_publisher_->trylock())
  {
    command_state_.data = static_cast<uint8_t>(state_interfaces_[StateInterfaces::COMMAND_MODE].get_value());
    rt_cmd_mode_publisher_->msg_.data = command_state_.data;
    rt_cmd_mode_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

CallbackReturn KortexCommandModeController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexCommandModeController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interfaces_[CommandInterfaces::SERVOING_MODE].set_value(static_cast<double>(Kinova::Api::Base::SINGLE_LEVEL_SERVOING));
  command_interfaces_[CommandInterfaces::COMMAND_MODE].set_value(static_cast<double>(CARTESIAN));
  try
  {
    servo_pub_ = get_node()->create_publisher<ServoMode>("~/arm_servo_mode", 1);
    cmd_pub_ = get_node()->create_publisher<CommandMode>("~/arm_command_mode", 1);
    rt_servo_mode_publisher_ = std::make_unique<ServoModeStatePublisher>(servo_pub_);
    rt_cmd_mode_publisher_ = std::make_unique<CommandModeStatePublisher>(cmd_pub_);
  }
  catch (const std::exception & e)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message : %s \n",
      e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn KortexCommandModeController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interfaces_[CommandInterfaces::SERVOING_MODE].set_value(static_cast<double>(Kinova::Api::Base::SINGLE_LEVEL_SERVOING));
  command_interfaces_[CommandInterfaces::COMMAND_MODE].set_value(static_cast<double>(CARTESIAN));
  return CallbackReturn::SUCCESS;
}

}  // namespace kortex2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex2_controllers::KortexCommandModeController, controller_interface::ControllerInterface)
