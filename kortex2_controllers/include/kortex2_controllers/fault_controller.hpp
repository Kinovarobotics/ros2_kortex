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

#ifndef KORTEX2_CONTROLLERS__FAULT_CONTROLLER_HPP_
#define KORTEX2_CONTROLLERS__FAULT_CONTROLLER_HPP_

#include <limits>
#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/msg/bool.hpp"
#include "example_interfaces/srv/trigger.hpp"
#include "kortex2_controllers/visibility_control.h"
#include "realtime_tools/realtime_publisher.h"

namespace kortex2_controllers
{
enum CommandInterfaces
{
  RESET_FAULT_CMD = 0u,
  RESET_FAULT_ASYNC_SUCCESS = 1u,
};
enum StateInterfaces
{
  IN_FAULT = 0u,
};
using CmdType = example_interfaces::srv::Trigger;
using FbkType = example_interfaces::msg::Bool;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class FaultController : public controller_interface::ControllerInterface
{
public:
  KORTEX2_CONTROLLERS_PUBLIC
  FaultController();

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
  bool resetFault(const CmdType::Request::SharedPtr req, CmdType::Response::SharedPtr resp);

  using StatePublisher = realtime_tools::RealtimePublisher<FbkType>;
  rclcpp::Publisher<FbkType>::SharedPtr fault_pub_;
  std::unique_ptr<StatePublisher> realtime_publisher_;
  FbkType state_;
  rclcpp::Service<CmdType>::SharedPtr trigger_command_srv_;

  static constexpr double ISSUE_CMD = 1.0;
  static constexpr double ASYNC_WAITING = 2.0;
  static constexpr double NO_CMD = std::numeric_limits<double>::quiet_NaN();
};

}  // namespace kortex2_controllers

#endif  // KORTEX2_CONTROLLERS__FAULT_CONTROLLER_HPP_
