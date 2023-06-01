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

#include "controller_interface/controller_interface.hpp"
#include "kortex2_controllers/visibility_control.h"

// Include Kortex API for enum classes
#include <BaseClientRpc.h>

namespace kortex2_controllers
{
enum CommandInterfaces
{
  SERVOING_MODE = 0u,
  COMMAND_MODE = 1u,
};
enum StateInterfaces
{
  SERVOING_MODE = 0u,
  COMMAND_MODE = 1u,
};

using ServoMode = example_interfaces::msg::UInt32;
using CommandMode = example_interfaces::msg::UInt8;

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
  using ServoModeStatePublisher = realtime_tools::RealtimePublisher<ServoMode>;
  using CommandModeStatePublisher = realtime_tools::RealtimePublisher<CommandMode>;
  rclcpp::Publisher<ServoMode>::SharedPtr servo_pub_;
  rclcpp::Publisher<CommandMode>::SharedPtr cmd_pub_;
  std::unique_ptr<StatePublisher> rt_servo_mode_publisher_;
  std::unique_ptr<CommandMode> rt_cmd_mode_publisher_;
  ServoMode servo_state_;
  CommandMode command_state_;

  static constexpr double CARTESIAN = 0;
  static constexpr double TWIST = 1;
};

}


#endif // KORTEX2_CONTROLLERS__KORTEX_COMMAND_MODE_CONTROLLER_HPP_
