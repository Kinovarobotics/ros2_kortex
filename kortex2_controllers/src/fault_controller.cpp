// Copyright 2021, PickNik Inc.
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
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-11-25
 *
 */
//----------------------------------------------------------------------

#include "kortex2_controllers/fault_controller.hpp"
#include "hardware_interface/loaned_command_interface.hpp"

namespace kortex2_controllers
{
using hardware_interface::LoanedCommandInterface;

FaultController::FaultController()
: controller_interface::ControllerInterface(), trigger_command_srv_(nullptr)
{
}

controller_interface::InterfaceConfiguration FaultController::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names.emplace_back("reset_fault/command");
  config.names.emplace_back("rest_fault/async_success");

  return config;
}

controller_interface::InterfaceConfiguration FaultController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration();
}

CallbackReturn FaultController::on_init()
{
  command_interfaces_[CommandInterfaces::RESET_FAULT_CMD].set_value(NO_CMD);
  command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].set_value(NO_CMD);

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type FaultController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  return controller_interface::return_type::OK;
}

CallbackReturn FaultController::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn FaultController::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  command_interfaces_[CommandInterfaces::RESET_FAULT_CMD].set_value(NO_CMD);
  command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].set_value(NO_CMD);
  trigger_command_srv_ = node_->create_service<std_srvs::srv::Trigger>(
    "~/reset_fault",
    std::bind(&FaultController::resetFault, this, std::placeholders::_1, std::placeholders::_2));

  return CallbackReturn::SUCCESS;
}

CallbackReturn FaultController::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  trigger_command_srv_.reset();
  command_interfaces_[CommandInterfaces::RESET_FAULT_CMD].set_value(NO_CMD);
  command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].set_value(NO_CMD);

  return CallbackReturn::SUCCESS;
}

bool FaultController::resetFault(
  const std_srvs::srv::Trigger::Request::SharedPtr & /*req*/,
  std_srvs::srv::Trigger::Response::SharedPtr & resp)
{
  command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  command_interfaces_[CommandInterfaces::RESET_FAULT_CMD].set_value(ISSUE_CMD);

  RCLCPP_INFO(node_->get_logger(), "Trying to reset faults on kinova controller.");

  while (command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].get_value() ==
         ASYNC_WAITING)
  {
    // Asynchronous wait until the hardware interface has set the io value
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  resp->success = static_cast<bool>(
    command_interfaces_[CommandInterfaces::RESET_FAULT_ASYNC_SUCCESS].get_value());
  return resp->success;
}

}  // namespace kortex2_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  kortex2_controllers::FaultController, controller_interface::ControllerInterface)
