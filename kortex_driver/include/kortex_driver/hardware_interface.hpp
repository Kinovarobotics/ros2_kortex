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
 * \author Marq Rasmussen marq.rasmussen@picknik.ai
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-06-15
 *
 */
//----------------------------------------------------------------------
#ifndef KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
#define KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_

#pragma once

#include <atomic>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "kortex_driver/visibility_control.h"

#include "BaseClientRpc.h"
#include "BaseCyclicClientRpc.h"
#include "RouterClient.h"
#include "SessionManager.h"
#include "TransportClientTcp.h"
#include "TransportClientUdp.h"

namespace hardware_interface
{
constexpr char HW_IF_TWIST[] = "twist";
constexpr char HW_IF_FAULT[] = "fault";

}  // namespace hardware_interface

using hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace k_api = Kinova::Api;

namespace kortex_driver
{
enum class StopStartInterface
{
  NONE,
  STOP_POS_VEL,
  STOP_TWIST,
  STOP_GRIPPER,
  STOP_FAULT_CTRL,
  START_POS_VEL,
  START_TWIST,
  START_GRIPPER,
  START_FAULT_CTRL,
};
class KortexMultiInterfaceHardware : public hardware_interface::SystemInterface
{
public:
  KortexMultiInterfaceHardware();

  RCLCPP_SHARED_PTR_DEFINITIONS(KortexMultiInterfaceHardware);

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  KORTEX_DRIVER_PUBLIC
  return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) final;
  KORTEX_DRIVER_PUBLIC
  return_type perform_command_mode_switch(
    const std::vector<std::string> & /*start_interfaces*/,
    const std::vector<std::string> & /*stop_interfaces*/) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) final;

  KORTEX_DRIVER_PUBLIC
  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) final;

  KORTEX_DRIVER_PUBLIC
  return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) final;

private:
  k_api::TransportClientTcp transport_tcp_;
  k_api::RouterClient router_tcp_;
  k_api::SessionManager session_manager_;
  k_api::TransportClientUdp transport_udp_realtime_;
  k_api::RouterClient router_udp_realtime_;
  k_api::SessionManager session_manager_real_time_;

  // twist temporary command
  Kinova::Api::Base::Twist * k_api_twist_;
  k_api::Base::TwistCommand k_api_twist_command_;

  // Control of the robot arm itself
  k_api::Base::BaseClient base_;
  k_api::BaseCyclic::BaseCyclicClient base_cyclic_;
  k_api::BaseCyclic::Command base_command_;
  std::size_t actuator_count_;
  // To minimize bandwidth we synchronize feedback with the robot only when write() is called
  k_api::BaseCyclic::Feedback feedback_;
  std::vector<double> arm_commands_positions_;
  std::vector<double> arm_commands_velocities_;
  std::vector<double> arm_commands_efforts_;
  std::vector<double> arm_positions_;
  std::vector<double> arm_velocities_;
  std::vector<double> arm_efforts_;

  // twist command interfaces
  std::vector<double> twist_commands_;

  // Gripper
  k_api::GripperCyclic::MotorCommand * gripper_motor_command_;
  double gripper_command_position_;
  double gripper_command_max_velocity_;
  double gripper_command_max_force_;
  double gripper_position_;
  double gripper_velocity_;

  rclcpp::Time controller_switch_time_;
  std::atomic<bool> block_write = false;
  k_api::Base::ServoingMode arm_mode_;

  // Enum defining at which control level we are
  // Dumb way of maintaining the command_interface type per joint.
  enum class integration_lvl_t : std::uint8_t
  {
    UNDEFINED = 0,
    POSITION = 1,
    VELOCITY = 2,
    EFFORT = 3
  };

  std::vector<integration_lvl_t> arm_joints_control_level_;

  // changing active controller on the hardware
  k_api::Base::ServoingModeInformation servoing_mode_hw_;
  // what controller is running
  bool joint_based_controller_running_;
  bool twist_controller_running_;
  bool gripper_controller_running_;
  bool fault_controller_running_;
  // switching auxiliary vars
  // keeping track of which controller is active so appropriate control mode can be adjusted
  // controller manager sends array of interfaces that should be stopped/started and this is the
  // way to internally collect information on which controller should be stopped and started
  // (different controllers claim different interfaces)
  std::vector<StopStartInterface> stop_modes_;
  std::vector<StopStartInterface> start_modes_;
  // switching auxiliary booleans
  bool stop_joint_based_controller_;
  bool stop_twist_controller_;
  bool stop_gripper_controller_;
  bool stop_fault_controller_;
  bool start_joint_based_controller_;
  bool start_twist_controller_;
  bool start_gripper_controller_;
  bool start_fault_controller_;

  // first pass flag
  bool first_pass_;

  // gripper stuff
  std::string gripper_joint_name_;
  bool use_internal_bus_gripper_comm_;

  // temp variables to use in update loop
  float cmd_degrees_tmp_;
  float cmd_vel_tmp_;
  int num_turns_tmp_ = 0;

  // fault control
  double reset_fault_cmd_;
  double reset_fault_async_success_;
  double in_fault_;
  static constexpr double NO_CMD = std::numeric_limits<double>::quiet_NaN();

  void sendTwistCommand();
  void incrementId();
  void sendJointCommands();
  void prepareCommands();
  void sendGripperCommand(
    k_api::Base::ServoingMode arm_mode, double position, double velocity, double force);

  void readGripperPosition();
};

}  // namespace kortex_driver

#endif  // KORTEX_DRIVER__HARDWARE_INTERFACE_HPP_
